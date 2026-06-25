"""
Simulation Initialization Module for Kirigami Deployment

This module handles the initialization of the kirigami simulation, including:
- Loading vertex and constraint data from files
- Creating PyBullet rigid bodies (bricks) and constraints
- Setting up the initial simulation state
- Preparing data structures for runtime simulation control
"""
import numpy as np
import pybullet as p
from utils.config import load_vertices_from_file, load_constraints_from_file, validate_constraints, load_pull_list
from utils.geometry import (create_extruded_geometry, create_brick_body,
    create_constraints_between_bricks, transform_local_to_world_coordinates,
    create_ground_plane, compute_min_z, is_planar_configuration,
    auto_determine_connection_types, align_target_to_initial)
from utils.physics_utils import stabilize_bodies, calculate_vertex_based_forces

class Simulation:
    """
    Handles initialization and setup of the kirigami simulation.

    Attributes:
        args (Namespace): Command-line arguments for the simulation.
        simulation_data (dict): Dictionary to store simulation-related data.
    """
    def __init__(self, args):
        self.args = args
        self.simulation_data = {}
    
    def initialize(self):
        """
        Initialize the kirigami simulation from input files.
        
        Returns:
            dict: Simulation data dictionary with all required objects
        """
        # Create new list for brick IDs
        brick_ids = []
        
        # Load data from file
        vertices = load_vertices_from_file(self.args.vertices_file)
        constraints = []
        if getattr(self.args, 'constraints_file', None):
            constraints = load_constraints_from_file(self.args.constraints_file)
        else:
            print("No constraints file provided — tiles will be independent.")
        original_initial_vertices = vertices  # keep copy before any transform
        
        # Validate constraints for initial vertices
        validate_constraints(vertices, constraints)
        
        # Load target vertices if using target-based deployment
        target_vertices = None
        if self.args.target_vertices_file:
            target_vertices = load_vertices_from_file(self.args.target_vertices_file)
            if len(target_vertices) != len(vertices):
                print(f"Warning: Target vertices count ({len(target_vertices)}) doesn't match initial vertices count ({len(vertices)})")
                print("Disabling target-based deployment")
                target_vertices = None
            else:
                # Validate constraints in target vertices
                validate_constraints(target_vertices, constraints)
                # Align target to initial via Kabsch (handles mirror/rotation)
                # We then apply the INVERSE transform to initial so that the
                # target keeps its original 3D position (better for camera).
                if len(vertices) > 0 and len(vertices) == len(target_vertices):
                    R, t, _ = align_target_to_initial(vertices, target_vertices)
                    # Transform initial vertices to target's frame:
                    #   initial' = R^T @ (initial - t)
                    new_vertices = []
                    for tile in vertices:
                        new_tile = [(R.T @ (np.array(v) - t)).tolist() for v in tile]
                        new_vertices.append(new_tile)
                    vertices = new_vertices
                    # target_vertices stays unchanged (original target)
        
        print(f"Loaded {len(vertices)} bricks and {len(constraints)} constraints")
        
        # ---- Auto-detect connection types when target is available ----
        auto_detect = getattr(self.args, 'auto_detect_connections', False)
        if target_vertices is not None and auto_detect:
            initial_planar = is_planar_configuration(original_initial_vertices, tol=1e-9)
            target_planar = is_planar_configuration(target_vertices, tol=1e-9)
        
            # Deduplicate constraints by their first 4 columns
            seen = set()
            unique_constraints = []
            for c in constraints:
                key = tuple(c[:4])
                if key not in seen:
                    seen.add(key)
                    unique_constraints.append(c)
            if len(unique_constraints) < len(constraints):
                print(f"Deduplicated constraints: {len(constraints)} → {len(unique_constraints)}")
            constraints = unique_constraints
            
            if initial_planar and target_planar:
                print("Both initial and target configurations are planar → connecting both faces (type 3)")
                constraints = [[c[0], c[1], c[2], c[3], 3] for c in constraints]
            else:
                print("Auto-detecting connection types from target geometry...")
                raw_constraints = [c[:4] for c in constraints]
                constraints = auto_determine_connection_types(vertices, target_vertices, raw_constraints)
                n_bottom = sum(1 for c in constraints if c[4] == 1)
                n_top = sum(1 for c in constraints if c[4] == 2)
                n_both = sum(1 for c in constraints if c[4] == 3)
                print(f"  → {n_bottom} bottom, {n_top} top, {n_both} both-face connections")
        elif target_vertices is not None:
            # Target exists but auto-detection disabled — use file types as-is
            print("Auto-detection disabled → using constraint-file types")
        # If no target, constraints keep their file-specified types (default type=1)
        

        # Create bricks
        local_vertices = []  # Store local vertices for each brick
        visual_meshes = []  # Store visual meshes for export

        # Create each brick
        for vertices_per_tile in vertices:
            # Create extruded 3D geometry from polygon vertices
            (local_verts_per_tile, vis_indices, center, vis_normals, vis_vertices) = create_extruded_geometry(
                vertices_per_tile, self.args.brick_thickness
            )
            
            # Create brick body in physics engine
            brick_id = create_brick_body(local_verts_per_tile, vis_indices, center, vis_normals, vis_vertices)
            
            brick_ids.append(brick_id)
            local_vertices.append(local_verts_per_tile)

            # Collect visual mesh for later export (OBJ/MTL)
            visual_meshes.append(
                {
                    'vertices_local': vis_vertices,  # duplicated verts for flat shading
                    'normals_local': vis_normals,     # one normal per emitted vertex
                    'indices': vis_indices         # flat list, 3 per triangle
                }
            )


        # Stabilize bricks
        stabilize_bodies(brick_ids, 
                        linear_damping=self.args.linear_damping, 
                        angular_damping=self.args.angular_damping)

        if getattr(self.args, 'ground_plane', False):
            min_z = min(compute_min_z(vertices), compute_min_z(target_vertices) if target_vertices else float('inf'))
            create_ground_plane(z = min_z, friction=self.args.ground_friction)

        # Create constraints between bricks
        filter_coll = getattr(self.args, 'no_collision', False)
        constraint_ids = create_constraints_between_bricks(
            brick_ids, constraints, local_vertices,
            filter_collision=filter_coll
        )
        
        local_bottom_vertices = []
        for local_verts_per_tile in local_vertices:
            num_bottom = len(local_verts_per_tile) // 2 
            local_bottom = local_verts_per_tile[:num_bottom]
            local_bottom_vertices.append(local_bottom)

        # ---- Auto-position camera ----
        self._setup_camera(vertices, target_vertices)

        # ---- Load pull-list if present (tiles to apply forces to) ----
        pull_file = self.args.vertices_file.replace('vertices.txt', 'pull.txt')
        pull_tiles = load_pull_list(pull_file)
        if pull_tiles is not None:
            print(f"Loaded pull-list: {len(pull_tiles)} tiles")

        # Prepare simulation data
        self.simulation_data = {
            'args': self.args,
            'bricks': brick_ids,
            'local_coords': local_bottom_vertices, # we only need local bottom vertices after brick creation
            'constraint_ids': constraint_ids,
            'visual_mesh': visual_meshes,
            'target_vertices': target_vertices,  # None if not using target-based deployment
            'pull_tiles': pull_tiles,  # None or set of tile indices to pull
        }
        
        return self.simulation_data
    
    def apply_forces(self, active_tiles=None, stiffness_mult=1.0):
        """
        Choose and apply forces.  *stiffness_mult* scales spring stiffness.
        """
        args = self.simulation_data.get('args')

        if self.simulation_data.get('target_vertices') and getattr(args, 'target_vertices_file', None):
            self._apply_target_based_forces(active_tiles, stiffness_mult)
            return

        if getattr(args, 'cm_expansion', False):
            self._apply_expansion_forces()
    
    def _apply_target_based_forces(self, active_tiles=None, stiffness_mult=1.0):
        """Apply target-based forces with optional stiffness scaling."""
        body_ids = self.simulation_data.get('bricks', [])
        target_bottom_vertices = self.simulation_data.get('target_vertices')
        if not body_ids or not target_bottom_vertices:
            return

        # Transform local bottom vertices to world coordinates for current positions
        current_bottom_vertices = transform_local_to_world_coordinates(
            body_ids, self.simulation_data['local_coords']
        )

        for i, body_id in enumerate(body_ids):
            if (i >= len(current_bottom_vertices) or i >= len(target_bottom_vertices)):
                continue
            
            # Skip tiles not in the active set (for staged deployment)
            if active_tiles is not None and i not in active_tiles:
                continue
                
            # Get current state
            center_pos, _ = p.getBasePositionAndOrientation(body_id)
            linear_v, _ = p.getBaseVelocity(body_id)
            
            # Calculate vertex-based forces (bottom face only)
            applied_force, total_torque = calculate_vertex_based_forces(
                current_bottom_vertices[i], target_bottom_vertices[i],
                self.args.spring_stiffness * stiffness_mult
            )
            
            total_force = applied_force - np.array(linear_v) * self.args.force_damping
            
            # Apply forces to the body
            if np.linalg.norm(total_force) > 0:
                p.applyExternalForce(
                    body_id, -1, 
                    total_force.tolist(), 
                    center_pos, 
                    flags=p.WORLD_FRAME
                )
            
            if np.linalg.norm(total_torque) > 0:
                p.applyExternalTorque(
                    body_id, -1,
                    total_torque.tolist(),
                    flags=p.WORLD_FRAME
                )
    
    
    def _apply_expansion_forces(self):
        """
        Apply center-of-mass based expansion forces to each brick:
        F = k * (pos - whole_center) - c * vel
        where k = spring_stiffness and c = force_damping.
        """
        body_ids = self.simulation_data.get('bricks', [])
        if not body_ids:
            return
        
        # Compute overall center of the structure (average of brick centers)
        centers = []
        for body_id in body_ids:
            pos, _ = p.getBasePositionAndOrientation(body_id)
            centers.append(np.array(pos))
        whole_center = np.mean(centers, axis=0)

        k = float(self.args.spring_stiffness)
        c = float(self.args.force_damping)

        for body_id in body_ids:
            pos, _ = p.getBasePositionAndOrientation(body_id)
            vel, _ = p.getBaseVelocity(body_id)
            pos = np.array(pos)
            vel = np.array(vel)
            force = k * (pos - whole_center) - c * vel
            if np.linalg.norm(force) > 0:
                p.applyExternalForce(
                    body_id, -1, force.tolist(), pos.tolist(), flags=p.WORLD_FRAME
                )

    def _setup_camera(self, initial_vertices, target_vertices=None):
        """Position the PyBullet debug camera to frame the geometry.

        Uses the combined initial + target point cloud when a target is
        available, otherwise uses the initial vertices alone.  Computes a
        camera target at the centroid and a distance proportional to the
        point-cloud extent.
        """
        import numpy as np

        # Collect all points
        all_pts = []
        for tile in initial_vertices:
            all_pts.extend(tile)
        if target_vertices is not None:
            for tile in target_vertices:
                all_pts.extend(tile)
        if not all_pts:
            return
        pts = np.array(all_pts, dtype=float)

        centroid = np.mean(pts, axis=0)
        bb_min = np.min(pts, axis=0)
        bb_max = np.max(pts, axis=0)
        extent = np.linalg.norm(bb_max - bb_min)
        distance = max(extent, 2.0)

        p.resetDebugVisualizerCamera(
            cameraDistance=distance,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=centroid.tolist()
        )
        print(f"Camera: target={centroid.tolist()}, distance={distance:.1f}")

    def compute_max_error(self, active_tiles=None):
        """Return the maximum Euclidean vertex error for the given tiles.

        Compares current world positions of bottom-face vertices against
        target positions.  Used for stall detection in the controller.
        """
        body_ids = self.simulation_data.get('bricks', [])
        target = self.simulation_data.get('target_vertices')
        local_coords = self.simulation_data.get('local_coords')
        if not body_ids or target is None or not local_coords:
            return float('inf')

        current = transform_local_to_world_coordinates(body_ids, local_coords)
        max_err = 0.0
        tiles = active_tiles if active_tiles is not None else range(len(body_ids))
        for i in tiles:
            if i >= len(current) or i >= len(target):
                continue
            for cv, tv in zip(current[i], target[i]):
                err = np.linalg.norm(np.array(cv) - np.array(tv))
                if err > max_err:
                    max_err = err
        return max_err