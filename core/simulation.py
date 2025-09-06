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
from utils.config import load_vertices_from_file, load_constraints_from_file, validate_constraints
from utils.geometry import create_extruded_geometry, create_brick_body, create_constraints_between_bricks, transform_local_to_world_coordinates, create_ground_plane, compute_min_z
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
        constraints = load_constraints_from_file(self.args.constraints_file)
        
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
        
        print(f"Loaded {len(vertices)} bricks and {len(constraints)} constraints")
        

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
            create_ground_plane(z = min_z)

        # Create constraints between bricks
        constraint_ids = create_constraints_between_bricks(
            brick_ids, constraints, local_vertices
        )
        
        local_bottom_vertices = []
        for local_verts_per_tile in local_vertices:
            num_bottom = len(local_verts_per_tile) // 2 
            local_bottom = local_verts_per_tile[:num_bottom]
            local_bottom_vertices.append(local_bottom)

        # Prepare simulation data
        self.simulation_data = {
            'args': self.args,
            'bricks': brick_ids,
            'local_coords': local_bottom_vertices, # we only need local bottom vertices after brick creation
            'constraint_ids': constraint_ids,
            'visual_mesh': visual_meshes,
            'target_vertices': target_vertices,  # None if not using target-based deployment
        }
        
        return self.simulation_data
    
    def apply_forces(self):
        """
        Choose and apply forces based on configuration:
        - If target vertices are provided, apply target-driven vertex forces.
        - Else if cm_expansion is enabled, apply center-of-mass expansion forces.
        """
        args = self.simulation_data.get('args') # to check whether cm_expansion is enabled

        # Target-driven deployment takes precedence when available
        if self.simulation_data.get('target_vertices') and getattr(args, 'target_vertices_file', None):
            self._apply_target_based_forces()
            return

        # Center-of-mass expansion as an alternative mode
        if getattr(args, 'cm_expansion', False):
            self._apply_expansion_forces()
    
    def _apply_target_based_forces(self):
        """
        Apply target-based forces to all bodies in the simulation.
        Uses only bottom face vertices since top vertices are derived from bottom + thickness.
        """
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
                
            # Get current state
            center_pos, _ = p.getBasePositionAndOrientation(body_id)
            linear_v, _ = p.getBaseVelocity(body_id)
            
            # Calculate vertex-based forces (bottom face only)
            applied_force, total_torque = calculate_vertex_based_forces(
                current_bottom_vertices[i], target_bottom_vertices[i], self.args.spring_stiffness
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

    