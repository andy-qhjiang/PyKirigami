"""
Target-based force application for kirigami deployment using vertex-based forces.
This module applies individual vertex forces to guide each face toward its desired final configuration.
"""
import numpy as np
import pybullet as p

def calculate_vertex_based_forces(current_vertices, target_vertices, stiffness=500.0):
    """
    Alternative approach: Calculate individual forces for each vertex,
    then compute the resultant force and torque for the face center.
    
    Args:
        current_vertices: List of current vertex positions
        target_vertices: List of target vertex positions  
        stiffness: Spring stiffness for vertex attraction
        
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
        vertex_force = stiffness * (target_vertices[i] - current_vertices[i])
        
        # Add to net force
        net_force += vertex_force
        
        # Calculate torque contribution: r × F
        # r is vector from face center to vertex
        r_vector = current_vertices[i] - current_center
        torque_contribution = np.cross(r_vector, vertex_force)
        net_torque += torque_contribution
    
    return net_force.tolist(), net_torque.tolist()

def apply_target_based_forces(body_ids, current_bottom_vertices, target_bottom_vertices, 
                             force_params):
    """
    Apply vertex-based target forces to all bodies in the simulation.
    Uses only bottom face vertices since top vertices are derived from bottom + thickness.
    
    Args:
        body_ids: List of PyBullet body IDs
        current_bottom_vertices: Current bottom vertex positions for each body  
        target_bottom_vertices: Target bottom vertex positions for each body
        force_params: Dictionary with force parameters {'stiffness': float, 'damping': float}
    """
    
    for i, body_id in enumerate(body_ids):
        if (i >= len(current_bottom_vertices) or i >= len(target_bottom_vertices)):
            continue
            
        # Get current state
        center_pos, _ = p.getBasePositionAndOrientation(body_id)

        linear_v, angular_v = p.getBaseVelocity(body_id)
        
        # Use vertex-based forces (bottom face only)
        applied_force, total_torque = calculate_vertex_based_forces(
            current_bottom_vertices[i], target_bottom_vertices[i], 
            force_params['stiffness']
        )
        
        total_force = np.array(applied_force) - np.array(linear_v) * force_params['damping']
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

def update_current_vertices_from_simulation(body_ids, local_bottom_vertices):
    """
    Update the current bottom vertex positions based on the current state of the simulation.
    This function transforms local vertex coordinates to world coordinates.
    
    Args:
        body_ids: List of PyBullet body IDs
        local_bottom_vertices: Local bottom vertex coordinates for each body
                              Structure: [ brick0_vertices, brick1_vertices, ... ]
                              where brick0_vertices = [[x,y,z], [x,y,z], [x,y,z], [x,y,z]]
        
    Returns:
        list: updated_bottom_vertices in world coordinates  
              Structure: [ brick0_world_vertices, brick1_world_vertices, ... ]
              where brick0_world_vertices = [[x,y,z], [x,y,z], [x,y,z], [x,y,z]]
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
