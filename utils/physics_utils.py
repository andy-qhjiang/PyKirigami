"""
Physics utilities for object manipulation in PyBullet simulations.
Provides common functions for fixing/unfixing objects to/from world frame.
"""
import sys
import numpy as np
import pybullet as p


def fix_object_to_world(object_id, max_force=1e10):
    """
    Fix an object to the world frame using a JOINT_FIXED constraint.
    
    Args:
        object_id: PyBullet body ID to fix
        max_force: Maximum force the constraint can apply (default: 1e10)
        
    Returns:
        int: Constraint ID that can be used to remove the constraint later
    """
    # Get the current position and orientation
    pos, orn = p.getBasePositionAndOrientation(object_id)
    
    # Create fixed constraint to world frame
    constraint_id = p.createConstraint(
        parentBodyUniqueId=object_id,
        parentLinkIndex=-1,
        childBodyUniqueId=-1,  # World frame
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],  # Relative to object's CoM
        childFramePosition=pos,         # Target position in world
        parentFrameOrientation=[0, 0, 0, 1], # Relative orientation
        childFrameOrientation=orn         # Target orientation in world
    )
    
    # Set high max force for stability
    p.changeConstraint(constraint_id, maxForce=max_force)
    
    return constraint_id


def unfix_object_from_world(constraint_id):
    """
    Remove a constraint that was fixing an object to the world frame.
    
    Args:
        constraint_id: The constraint ID returned by fix_object_to_world
        
    Returns:
        bool: True if constraint was successfully removed, False otherwise
    """
    try:
        p.removeConstraint(constraint_id)
        return True
    except Exception as e:
        print(f"Warning: Could not remove constraint {constraint_id}: {e}")
        return False


def create_visual_indicator(position, radius=0.08, color=[1, 0, 0, 0.8]):
    """
    Create a visual indicator (sphere) at a specific position.
    
    Args:
        position: [x, y, z] position for the indicator
        radius: Radius of the sphere (default: 0.08)
        color: RGBA color [r, g, b, a] (default: red with 80% opacity)
        
    Returns:
        int: Body ID of the created indicator
    """
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color
    )
    
    indicator_id = p.createMultiBody(
        baseMass=0,  # Zero mass so physics don't affect it
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
        baseOrientation=[0, 0, 0, 1],
        useMaximalCoordinates=True
    )
    
    return indicator_id


def remove_visual_indicator(indicator_id):
    """
    Remove a visual indicator body.
    
    Args:
        indicator_id: Body ID of the indicator to remove
        
    Returns:
        bool: True if indicator was successfully removed, False otherwise
    """
    try:
        p.removeBody(indicator_id)
        return True
    except Exception as e:
        print(f"Warning: Could not remove indicator {indicator_id}: {e}")
        return False


def fix_multiple_objects_to_world(object_ids, max_force=1e10):
    """
    Fix multiple objects to the world frame.
    
    Args:
        object_ids: List of PyBullet body IDs to fix
        max_force: Maximum force each constraint can apply
        
    Returns:
        dict: Dictionary mapping object_id -> constraint_id
    """
    constraints = {}
    for object_id in object_ids:
        constraint_id = fix_object_to_world(object_id, max_force)
        constraints[object_id] = constraint_id
    return constraints


def unfix_multiple_objects_from_world(constraints_dict):
    """
    Remove multiple constraints.
    
    Args:
        constraints_dict: Dictionary mapping object_id -> constraint_id
        
    Returns:
        bool: True if all constraints were successfully removed
    """
    success = True
    for object_id, constraint_id in constraints_dict.items():
        if not unfix_object_from_world(constraint_id):
            success = False
    return success


def validate_constraints(vertices, constraints, max_distance=0.1):
    """
    Validate that constraints in target vertices don't have excessive distances.
    
    This function checks if constraint endpoints in the target configuration
    are close enough together to avoid physics instabilities.
    
    Args:
        vertices: List of vertex arrays for each brick
        constraints: List of constraint definitions [face1, vertex1, face2, vertex2, ...]
        max_distance: Maximum allowed distance between constraint endpoints (default: 0.1)
        
    Raises:
        SystemExit: If any constraint has distance > max_distance
        
    Returns:
        None: Function either passes validation or exits the program
    """
    if vertices is None:
        return
        
    for i, constraint in enumerate(constraints):
        f1, v1, f2, v2 = constraint[:4]
        
        # Get vertex positions for constraint endpoints
        pos1 = np.array(vertices[f1][3*v1:3*v1+3])
        pos2 = np.array(vertices[f2][3*v2:3*v2+3])
        
        # Calculate distance using numpy
        dist = np.linalg.norm(pos1 - pos2)
        
        if dist > max_distance:
            error_msg = f"ERROR: Constraint {i} between faces {f1} and {f2} with vertices {v1} and {v2} has a large distance ({dist:.3f}) in target vertices."
            print(error_msg)
            print(f"This exceeds the maximum allowed distance of {max_distance:.3f} and will lead to unexpected behavior.")
            print("Please check your target vertices file.")
            sys.exit(1)
