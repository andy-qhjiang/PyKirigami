"""
Core simulation class for kirigami deployment.
Handles initialization and force application for the simulation.
"""
import numpy as np
import pybullet as p
from utils.setup import load_vertices_from_file, load_constraints_from_file
from utils.geometry import create_extruded_geometry, create_brick_body, create_constraints_between_bricks
from utils.physics_utils import stabilize_bodies, validate_constraints

class Simulation:
    def __init__(self, args):
        self.args = args
        self.simulation_data = {}
    
    def initialize(self):
        """
        Initialize the kirigami simulation from input files.
        
        Returns:
            dict: Simulation data dictionary with all required objects
        """
        # Create new local lists for bricks
        local_bricks = []
        
        # Load data
        vertices = load_vertices_from_file(self.args.vertices_file)
        constraints = load_constraints_from_file(self.args.constraints_file)
        
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
            
            local_bricks.append(brick_id)
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
        stabilize_bodies(local_bricks, 
                        linear_damping=self.args.linear_damping, 
                        angular_damping=self.args.angular_damping)
        
        # Create constraints between bricks
        constraint_ids = create_constraints_between_bricks(
            local_bricks, constraints, local_verts_list
        )
        
        # Prepare simulation data
        self.simulation_data = {
            'args': self.args,
            'target_vertices': target_bottom_vertices,
            'bricks': local_bricks,
            'local_verts_list': local_verts_list,
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
            
            # Compute local bottom vertices on-demand from stored local_verts
            local_verts_list = self.simulation_data['local_verts_list']
            
            # Create local bottom vertices for each brick
            local_bottom_vertices = []
            for local_verts in local_verts_list:
                num_bottom = len(local_verts) // 2  # Compute number of bottom vertices
                # Bottom vertices are the first num_bottom vertices in local_verts
                local_bottom = local_verts[:num_bottom]
                local_bottom_vertices.append(local_bottom)
            
            # Update current vertex positions from simulation
            current_bottom = self._update_current_vertices_from_simulation(
                current_bricks, local_bottom_vertices
            )
            
            # Apply target-based forces to all bricks
            self._apply_vertex_based_forces(
                current_bricks, 
                current_bottom,
                self.simulation_data['target_vertices']
            )
    
    def _update_current_vertices_from_simulation(self, body_ids, local_bottom_vertices):
        """
        Update the current bottom vertex positions based on the current state of the simulation.
        This function transforms local vertex coordinates to world coordinates.
        
        Args:
            body_ids: List of PyBullet body IDs
            local_bottom_vertices: Local bottom vertex coordinates for each body
            
        Returns:
            list: updated_bottom_vertices in world coordinates  
        """
        updated_bottom = []
        
        for i, body_id in enumerate(body_ids):
            if i >= len(local_bottom_vertices):
                continue
                
            # Get current pose of this brick
            current_pos, current_orn = p.getBasePositionAndOrientation(body_id)
            
            # Convert quaternion to rotation matrix
            rotation_matrix = np.array(p.getMatrixFromQuaternion(current_orn)).reshape(3, 3)
            
            # Vectorized transformation of all vertices for this brick
            local_verts = np.array(local_bottom_vertices[i])  # Shape: (n_vertices, 3)
            world_verts = (rotation_matrix @ local_verts.T).T + np.array(current_pos)  # Shape: (n_vertices, 3)
            
            # Add this brick's transformed vertices to the result
            updated_bottom.append(world_verts.tolist())
        
        return updated_bottom
    
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
            linear_v, angular_v = p.getBaseVelocity(body_id)
            
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
