"""
Physics utilities for object manipulation in PyBullet simulations, including:
- Setting up the physics engine with gravity, timestep, and substeps 
- stabilizing bodies with damping   
- Fixing and unfixing object to/from the world frame
"""
import numpy as np
import pybullet as p
import pybullet_data
import math

def setup_physics_engine(gravity=(0, 0, -9.81), timestep=1/240, substeps=10, gui=True):
    """
    Set up the physics engine with basic parameters.
    
    Args:
        gravity: Tuple of gravity components (default: (0, 0, -9.81))
        timestep: Physics timestep (default: 1/240)
        substeps: Number of sub-steps in the physics engine (default: 10)
        gui: Whether to use GUI or direct mode (default: True)
        
    Returns:
        int: The PyBullet client ID    """
    if gui:
        client_id = p.connect(p.GUI)
    else:
        client_id = p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(*gravity)
    p.setPhysicsEngineParameter(fixedTimeStep=timestep, numSubSteps=substeps)
   
    return client_id


def stabilize_bodies(body_ids, linear_damping=1, angular_damping=1):
    """
    Apply damping to bodies to stabilize them.
    
    Args:
        body_ids: List of body IDs to stabilize
        linear_damping: Linear damping factor (default: 1)
        angular_damping: Angular damping factor (default: 1)
    """
    for body_id in body_ids:
        p.resetBaseVelocity(body_id, [0, 0, 0], [0, 0, 0])
        p.changeDynamics(body_id, -1, linearDamping=linear_damping, angularDamping=angular_damping)


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

def getRayFromTo(mouseX, mouseY, camera_info):
    """
    generate the ray from the camera to the mouse position in the scene. 
    This is a standard technique to determine the 3D position of the mouse in the world.

    I learn this from one of the examples of pybullet
    """

    width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = camera_info
    camPos = [
        camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
        camTarget[2] - dist * camForward[2]
    ]
    farPlane = 10000
    rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
    invLen = farPlane * 1. / (math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] *
                                        rayForward[1] + rayForward[2] * rayForward[2]))
    rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
    rayFrom = camPos
    oneOverWidth = float(1) / float(width)
    oneOverHeight = float(1) / float(height)
    dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
    dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
    rayToCenter = [
        rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
    ]
    rayTo = [
        rayFrom[0] + rayForward[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] -
        float(mouseY) * dVer[0], rayFrom[1] + rayForward[1] - 0.5 * horizon[1] + 0.5 * vertical[1] +
        float(mouseX) * dHor[1] - float(mouseY) * dVer[1], rayFrom[2] + rayForward[2] -
        0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
    ]
    return rayFrom, rayTo


def calculate_vertex_based_forces(current_vertices, target_vertices, spring_stiffness):
    """
    Compute net force and torque from per-vertex springs toward target positions.

    Args:
        current_vertices: Iterable of current vertex positions [[x,y,z], ...]
        target_vertices: Iterable of target vertex positions [[x,y,z], ...]
        spring_stiffness: Scalar stiffness k for the spring model

    Returns:
        (net_force, net_torque): both as [fx, fy, fz] lists in world frame.
    """
    current_vertices = np.array(current_vertices)
    target_vertices = np.array(target_vertices)

    if len(current_vertices) != len(target_vertices) or len(current_vertices) == 0:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

    current_center = np.mean(current_vertices, axis=0)

    net_force = np.array([0.0, 0.0, 0.0])
    net_torque = np.array([0.0, 0.0, 0.0])

    for i in range(len(current_vertices)):
        vertex_force = spring_stiffness * (target_vertices[i] - current_vertices[i])
        net_force += vertex_force

        r_vector = current_vertices[i] - current_center
        torque_contribution = np.cross(r_vector, vertex_force)
        net_torque += torque_contribution

    return net_force.tolist(), net_torque.tolist()



