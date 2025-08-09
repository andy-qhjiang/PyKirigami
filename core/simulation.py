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
from utils.geometry import create_extruded_geometry, create_brick_body, create_constraints_between_bricks, transform_local_to_world_coordinates
from utils.physics_utils import stabilize_bodies, calculate_vertex_based_forces
from utils.geometry import create_ground_plane

class Simulation:
    """
    Handles initialization and setup of the kirigami simulation.
    
    This class is responsible for:
    - Loading vertex and constraint data from input files
    - Creating PyBullet physics bodies and constraints
    - Setting up the initial simulation state
    - Preparing data structures for runtime control
    
    The class does NOT handle:
    - Runtime simulation stepping (handled by SimulationController)
    - User interactions (handled by InteractionController)
    - Force application during simulation (handled by SimulationController)
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
        
        # Load data
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
        
        # Optionally compute min z for later ground placement
        all_z = []
        for flat in vertices:
            # flat list length multiple of 3
            z_vals = flat[2::3]
            all_z.extend(z_vals)
        if target_vertices:
            for flat in target_vertices:
                all_z.extend(flat[2::3])
        min_z = min(all_z) if all_z else 0.0

        # Create bricks
        local_verts_list = []  # Store local vertices for each brick
        
        # Create each brick
        for i, brick_vertices in enumerate(vertices):
            # Create extruded 3D geometry from polygon vertices
            (local_verts, visual_indices, center, normals) = create_extruded_geometry(
                brick_vertices, self.args.brick_thickness
            )
            
            # Create brick body in physics engine
            brick_id = create_brick_body(local_verts, visual_indices, center, normals)
            
            brick_ids.append(brick_id)
            local_verts_list.append(local_verts)
            
            # Process target vertices if available (convert to same format as current_bottom)
            if target_vertices and i < len(target_vertices):
                target_brick_vertices = target_vertices[i]
                # Convert flat target vertices to format like current_bottom
                target_vertices_reshaped = []
                for j in range(0, len(target_brick_vertices), 3):
                    target_vertices_reshaped.append([
                        target_brick_vertices[j],     # x
                        target_brick_vertices[j+1],   # y
                        target_brick_vertices[j+2]    # z
                    ])
                # Store target vertices for this brick
                if 'target_bottom_vertices' not in locals():
                    target_bottom_vertices = []
                target_bottom_vertices.append(target_vertices_reshaped)
        
        # Initialize target_bottom_vertices if not created
        if 'target_bottom_vertices' not in locals():
            target_bottom_vertices = None
            
        # Stabilize bricks
        stabilize_bodies(brick_ids, 
                        linear_damping=self.args.linear_damping, 
                        angular_damping=self.args.angular_damping)

        ground_id = None
        if getattr(self.args, 'ground_plane', False):
            # Place ground slightly below lowest geometry to avoid interference
            offset = min(0.1, self.args.brick_thickness)
            ground_id = create_ground_plane(min_z - 2 * self.args.brick_thickness, offset)
        
        # Create constraints between bricks
        constraint_ids = create_constraints_between_bricks(
            brick_ids, constraints, local_verts_list
        )
        
        local_bottom_vertices = []
        for local_verts in local_verts_list:
            num_bottom = len(local_verts) // 2  # Compute number of bottom vertices
            # Bottom vertices are the first num_bottom vertices in local_verts
            local_bottom = local_verts[:num_bottom]
            local_bottom_vertices.append(local_bottom)

        # Prepare simulation data
        self.simulation_data = {
            'args': self.args,
            'target_vertices': target_bottom_vertices,
            'bricks': brick_ids,
            'local_coords': local_bottom_vertices,  # Optimized for coordinate transforms
            'constraint_ids': constraint_ids
        }
        
        return self.simulation_data
    
    def apply_forces(self):
        """
        Choose and apply forces based on configuration:
        - If target vertices are provided, apply target-driven vertex forces.
        - Else if auto_expansion is enabled, apply center-of-mass expansion forces.
        """
        args = self.simulation_data.get('args') # to check whether auto_expansion is enabled

        # Target-driven deployment takes precedence when available
        if self.simulation_data.get('target_vertices') and getattr(args, 'target_vertices_file', None):
            self._apply_target_based_forces()
            return

        # Auto expansion as an alternative mode
        if getattr(args, 'auto_expansion', False):
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
            
            total_force = np.array(applied_force) - np.array(linear_v) * self.args.force_damping
            
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
                    total_torque,
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
