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
from utils.physics_utils import stabilize_bodies

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
        
        # Create bricks
        local_verts_list = []  # Store local vertices for each brick
        
        # Create each brick
        for i, brick_vertices in enumerate(vertices):
            # Create extruded 3D geometry from polygon vertices
            (local_verts, visual_indices, center) = create_extruded_geometry(
                brick_vertices, self.args.brick_thickness
            )
            
            # Create brick body in physics engine
            brick_id = create_brick_body(local_verts, visual_indices, center)
            
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
            'constraint_ids': constraint_ids,
        }
        
        return self.simulation_data
    
    def apply_forces(self):
        """
        Apply target-based forces to guide simulation toward target configuration.
        Integrates all target-based force logic internally.
        """
        current_bricks = self.simulation_data['bricks']
        
        # Check if we should use target-based deployment
        if self.simulation_data.get('target_vertices'):
            
            # Transform local bottom vertices to world coordinates
            bottom_world_coords = transform_local_to_world_coordinates(
                current_bricks, self.simulation_data['local_coords']
            )
            
            # Apply target-based forces to all bricks
            self._apply_vertex_based_forces(
                current_bricks, 
                bottom_world_coords,
                self.simulation_data['target_vertices']
            )
    
    def _apply_vertex_based_forces(self, body_ids, current_bottom_vertices, target_bottom_vertices):
        """
        Apply vertex-based target forces to all bodies in the simulation.
        Uses only bottom face vertices since top vertices are derived from bottom + thickness.
        
        Args:
            body_ids: List of PyBullet body IDs
            current_bottom_vertices: Current bottom vertex positions for each body  
            target_bottom_vertices: Target bottom vertex positions for each body
        """
        
        for i, body_id in enumerate(body_ids):
            if (i >= len(current_bottom_vertices) or i >= len(target_bottom_vertices)):
                continue
                
            # Get current state
            center_pos, _ = p.getBasePositionAndOrientation(body_id)
            linear_v, _ = p.getBaseVelocity(body_id)
            
            # Calculate vertex-based forces (bottom face only)
            applied_force, total_torque = self._calculate_vertex_based_forces(
                current_bottom_vertices[i], target_bottom_vertices[i]
            )
            
            total_force = np.array(applied_force) - np.array(linear_v) * self.args.target_damping
            
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
    
    def _calculate_vertex_based_forces(self, current_vertices, target_vertices):
        """
        Calculate individual forces for each vertex, then compute the resultant force and torque.
        
        Args:
            current_vertices: List of current vertex positions
            target_vertices: List of target vertex positions  
            
        Returns:
            tuple: (net_force, net_torque) for the face center
        """
        current_vertices = np.array(current_vertices)
        target_vertices = np.array(target_vertices)
        
        if len(current_vertices) != len(target_vertices):
            return [0, 0, 0], [0, 0, 0]
        
        current_center = np.mean(current_vertices, axis=0)
        
        # Calculate force for each vertex
        net_force = np.array([0.0, 0.0, 0.0])
        net_torque = np.array([0.0, 0.0, 0.0])
        
        for i in range(len(current_vertices)):
            # Force on this vertex toward its target
            vertex_force = self.args.target_stiffness * (target_vertices[i] - current_vertices[i])
            
            # Add to net force
            net_force += vertex_force
            
            # Calculate torque contribution: r × F
            # r is vector from face center to vertex
            r_vector = current_vertices[i] - current_center
            torque_contribution = np.cross(r_vector, vertex_force)
            net_torque += torque_contribution
        
        return net_force.tolist(), net_torque.tolist()
