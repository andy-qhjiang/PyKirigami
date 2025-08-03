"""
Setup utilities for the kirigami simulation project, including argument parsing.
"""
import pybullet as p
import pybullet_data
import numpy as np
import argparse


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
    p.setPhysicsEngineParameter(
        enableFileCaching=0, 
        reportSolverAnalytics=0, 
        deterministicOverlappingPairs=1,
        enableConeFriction=0  # Additional performance improvement
    )
    
    # # Explicitly disable timing file generation  
    # p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    # p.configureDebugVisualizer(p.COV_ENABLE_PROFILER, 0)
    
    return client_id

def create_ground_plane():
    """
    Create a ground plane in the simulation.
    
    Returns:
        int: The ground plane body ID
    """
    return p.loadURDF("plane.urdf")

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



def parse_arguments():
    """Parse command-line arguments for the simulation"""
    parser = argparse.ArgumentParser(description='Run kirigami simulation')
    
    # Input files
    parser.add_argument('--vertices_file', required=True, help='File containing vertex data')
    parser.add_argument('--constraints_file', required=True, help='File with connectivity constraints')
    parser.add_argument('--target_vertices_file', help='File containing target vertex positions for deployment (optional)')
    
    # Physics simulation parameters
    parser.add_argument('--gravity', type=float, default=0, help='Gravity constant')
    parser.add_argument('--timestep', type=float, default=1/240, help='Physics simulation timestep')
    parser.add_argument('--substeps', type=int, default=20, help='Physics substeps per step')
    
    
    # Target-based deployment parameters
    parser.add_argument('--target_stiffness', type=float, default=500.0,
                       help='Stiffness coefficient for target-based forces')
    parser.add_argument('--target_damping', type=float, default=50.0,
                       help='Damping coefficient for target-based forces')
    
    
    
    # Geometry parameters
    parser.add_argument('--brick_thickness', type=float, default=0.02,
                       help='Thickness of the brick (z-height)')
    parser.add_argument('--linear_damping', type=float, default=1, help='Linear damping')
    parser.add_argument('--angular_damping', type=float, default=1, help='Angular damping')
    
    
  
    # Visual options
    parser.add_argument('--ground_plane', action='store_true',
                       help='Add a ground plane to the simulation')
    parser.add_argument('--camera_distance', type=float, default=8.0,
                       help='Distance of the camera from the origin')
    
    return parser.parse_args()
