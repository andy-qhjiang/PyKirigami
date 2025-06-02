"""
Setup utilities for the kirigami simulation project, including argument parsing.
"""
import pybullet as p
import pybullet_data
import numpy as np
import argparse

DEFAULT_BRICK_THICKNESS = 0.02
DEFAULT_FORCE_MAGNITUDE = 50

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
    
    # Disable profiling to prevent generation of timings_*.json files
    p.setPhysicsEngineParameter(enableFileCaching=0, reportSolverAnalytics=0, deterministicOverlappingPairs=1)
    
    return client_id

def create_ground_plane():
    """
    Create a ground plane in the simulation.
    
    Returns:
        int: The ground plane body ID
    """
    return p.loadURDF("plane.urdf")

def stabilize_bodies(body_ids, linear_damping=2.5, angular_damping=2.5):
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

# Keep the existing initialize_pybullet function for backward compatibility
def initialize_pybullet(use_gui=True, gravity=(0, 0, -9.81), add_ground_plane=True):
    """
    Initialize the PyBullet physics simulation.
    
    Args:
        use_gui: Whether to use GUI or direct mode
        gravity: Gravity vector (x, y, z)
        add_ground_plane: Whether to add a ground plane
        
    Returns:
        int: PyBullet client ID
        int: Ground plane ID if added, None otherwise    """
    client_id = p.connect(p.GUI if use_gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(*gravity)
    
    # Simulation parameters for stability
    p.setPhysicsEngineParameter(fixedTimeStep=1/240, numSubSteps=10)
    
    # Disable profiling to prevent generation of timings_*.json files
    p.setPhysicsEngineParameter(enableFileCaching=0, reportSolverAnalytics=0, deterministicOverlappingPairs=1)
    
    plane_id = None
    if add_ground_plane:
        plane_id = p.loadURDF("plane.urdf")
        
    return client_id, plane_id


def parse_arguments():
    """Parse command-line arguments for the simulation"""
    parser = argparse.ArgumentParser(description='Run kirigami simulation')
    
    # Input files
    parser.add_argument('--vertices_file', required=True, help='File containing vertex data')
    parser.add_argument('--constraints_file', required=True, help='File with connectivity constraints')
    
    # Physics simulation parameters
    parser.add_argument('--gravity', type=float, default=0, help='Gravity constant')
    parser.add_argument('--timestep', type=float, default=1/240, help='Physics simulation timestep')
    parser.add_argument('--substeps', type=int, default=20, help='Physics substeps per step')
    parser.add_argument('--linear_damping', type=float, default=0.1, help='Linear damping')
    parser.add_argument('--angular_damping', type=float, default=0.1, help='Angular damping')
    
    # Force parameters
    parser.add_argument('--force_type', choices=['normal', 'outward'], default='normal',
                       help='Type of force to apply (normal, or outward)')
    parser.add_argument('--force_magnitude', type=float, default=100, 
                       help='Magnitude of the force applied to each brick')
    parser.add_argument('--force_bricks', type=int, nargs='+',
                       help='Indices of specific bricks to apply forces to (default: all)')
    
    # Geometry parameters
    parser.add_argument('--brick_thickness', type=float, default=0.1,
                       help='Thickness of the brick (z-height)')
  
    # Visual options
    parser.add_argument('--ground_plane', action='store_true',
                       help='Add a ground plane to the simulation')
    
    return parser.parse_args()
