"""
Physics setup utilities for the kirigami simulation project.
"""
import pybullet as p
import pybullet_data
import numpy as np

def setup_physics_engine(gravity=(0, 0, -9.81), timestep=1/240, substeps=10, gui=True):
    """
    Set up the physics engine with basic parameters.
    
    Args:
        gravity: Tuple of gravity components (default: (0, 0, -9.81))
        timestep: Physics timestep (default: 1/240)
        substeps: Number of sub-steps in the physics engine (default: 10)
        gui: Whether to use GUI or direct mode (default: True)
        
    Returns:
        int: The PyBullet client ID
    """
    if gui:
        client_id = p.connect(p.GUI)
    else:
        client_id = p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(*gravity)
    p.setPhysicsEngineParameter(fixedTimeStep=timestep, numSubSteps=substeps)
    
    return client_id

def create_ground_plane():
    """
    Create a ground plane in the simulation.
    
    Returns:
        int: The ground plane body ID
    """
    return p.loadURDF("plane.urdf")

def stabilize_bodies(body_ids, linear_damping=25, angular_damping=25):
    """
    Apply damping to bodies to stabilize them.
    
    Args:
        body_ids: List of body IDs to stabilize
        linear_damping: Linear damping factor (default: 25)
        angular_damping: Angular damping factor (default: 25)
    """
    for body_id in body_ids:
        p.resetBaseVelocity(body_id, [0, 0, 0], [0, 0, 0])
        p.changeDynamics(body_id, -1, linearDamping=linear_damping, angularDamping=angular_damping)

def apply_force_to_bodies(body_ids, force_function, force_magnitude):
    """
    Apply forces to bodies using a force function.
    
    Args:
        body_ids: List of body IDs to apply forces to
        force_function: Function that calculates the force direction for each body
        force_magnitude: Magnitude of the force to apply
    """
    for body_id in body_ids:
        center_pos, orientation = p.getBasePositionAndOrientation(body_id)
        force_dir = force_function(body_id, center_pos, orientation)
        force = [force_magnitude * d for d in force_dir]
        p.applyExternalForce(body_id, -1, force, center_pos, flags=p.WORLD_FRAME)