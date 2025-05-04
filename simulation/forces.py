"""
Force utilities for the kirigami simulation project.
"""
import numpy as np
import pybullet as p

def calculate_outward_direction(center_pos, whole_center):
    """
    Calculate direction from whole center to a position.
    
    Args:
        center_pos: Position to calculate direction to
        whole_center: Center of the whole structure
        
    Returns:
        list: Normalized direction vector
    """
    direction = [
        center_pos[0] - whole_center[0],
        center_pos[1] - whole_center[1],
        center_pos[2] - whole_center[2]
    ]
    norm = np.linalg.norm(direction)
    if norm > 0:
        return [d / norm for d in direction]
    else:
        return [0, 0, 0]

def calculate_normal_direction(body_id, center_pos, orientation, normal):
    """
    Calculate normal direction in world coordinates.
    
    Args:
        body_id: Body ID
        center_pos: Center position
        orientation: Orientation quaternion
        normal: Normal vector in local coordinates
        
    Returns:
        list: Normalized normal vector in world coordinates
    """
    rotation_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)
    world_normal = np.dot(rotation_matrix, normal)
    world_normal = world_normal / np.linalg.norm(world_normal)
    return world_normal.tolist()

def get_force_direction_function(force_type):
    """
    Get the appropriate force direction function based on the specified type.
    
    Args:
        force_type: Type of force ('normal' or 'outward')
        
    Returns:
        function: The force direction function
    """
    if force_type == 'normal':
        return calculate_normal_direction
    elif force_type == 'outward':
        return calculate_outward_direction
    else:
        print(f"Warning: Unknown force type '{force_type}', defaulting to normal")
        return calculate_normal_direction
    
def apply_force_to_bodies(body_ids, force_function, force_magnitude, normals=None):
    """
    Apply forces to bodies using a force function.
    
    Args:
        body_ids: List of body IDs to apply forces to
        force_function: Function that calculates the force direction for each body
        force_magnitude: Magnitude of the force to apply
        normals: Optional list of normal vectors for each body
    """
    for i, body_id in enumerate(body_ids):
        center_pos, orientation = p.getBasePositionAndOrientation(body_id)
        
        # If normals are provided, pass to force function
        if normals and i < len(normals):
            normal = normals[i]
            force_dir = force_function(body_id, center_pos, orientation, normal)
        else:
            force_dir = force_function(body_id, center_pos, orientation)
            
        force = [force_magnitude * d for d in force_dir]
        p.applyExternalForce(body_id, -1, force, center_pos, flags=p.WORLD_FRAME)