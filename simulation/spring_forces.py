"""
Force utilities for the kirigami simulation project.
Simplified version focusing on spring forces.
"""
import numpy as np
import pybullet as p

def calculate_spring_force(center_pos, whole_center, velocity, spring_radius, stiffness, damping):
    """
    Calculate spring-like forces for auto-expansion behavior.
    
    Args:
        center_pos: Current position of the body
        whole_center: Center of the whole structure
        velocity: Current velocity of the body
        spring_radius: Target radius for expansion
        stiffness: Spring stiffness coefficient
        damping: Damping coefficient
        
    Returns:
        list: Calculated force vector [fx, fy, fz]
    """
    # Calculate current displacement from center
    direction = np.array([
        center_pos[0] - whole_center[0],
        center_pos[1] - whole_center[1],
        center_pos[2] - whole_center[2]
    ])
    
    current_dist = np.linalg.norm(direction)
    
    # Avoid division by zero
    if current_dist < 1e-6:
        return [0, 0, 0]
    
    # Normalize direction
    norm_direction = direction / current_dist
    
    # Calculate target position based on the normalized direction
    target_radius = spring_radius
    
    # Calculate target position
    target_pos = whole_center + norm_direction * target_radius
    
    # Calculate displacement from current to target position
    delta_pos = target_pos - np.array(center_pos)
    
    # Calculate spring force (F = k*x)
    spring_force = stiffness * delta_pos
    
    # Calculate damping force (F = -c*v)
    damping_force = -damping * np.array(velocity)
    
    # Combine spring and damping forces
    total_force = spring_force + damping_force
    
    return total_force.tolist()

def apply_spring_forces(body_ids, whole_center, spring_params):
    """
    Apply spring forces to bodies for expansion.
    
    Args:
        body_ids: List of body IDs to apply forces to
        whole_center: Center of the whole structure
        spring_params: Dictionary of spring parameters
    """
    for body_id in body_ids:
        center_pos, _ = p.getBasePositionAndOrientation(body_id)
        
        # Get velocity for damping calculation
        linear_velocity, _ = p.getBaseVelocity(body_id)
        
        # Calculate spring force
        force = calculate_spring_force(
            center_pos, 
            whole_center, 
            linear_velocity, 
            spring_params['radius'],
            spring_params['stiffness'],
            spring_params['damping'],
        )
        
        # Apply the force
        p.applyExternalForce(body_id, -1, force, center_pos, flags=p.WORLD_FRAME)
