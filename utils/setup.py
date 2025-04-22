"""
Setup utilities for the kirigami simulation project, including argument parsing.
"""
import pybullet as p
import pybullet_data
import numpy as np
import argparse

DEFAULT_BRICK_THICKNESS = 0.02
DEFAULT_FORCE_MAGNITUDE = 500

def initialize_pybullet(use_gui=True, gravity=(0, 0, -9.81), add_ground_plane=True):
    """
    Initialize the PyBullet physics simulation.
    
    Args:
        use_gui: Whether to use GUI or direct mode
        gravity: Gravity vector (x, y, z)
        add_ground_plane: Whether to add a ground plane
        
    Returns:
        int: PyBullet client ID
        int: Ground plane ID if added, None otherwise
    """
    client_id = p.connect(p.GUI if use_gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(*gravity)
    
    # Simulation parameters for stability
    p.setPhysicsEngineParameter(fixedTimeStep=1/240, numSubSteps=10)
    
    plane_id = None
    if add_ground_plane:
        plane_id = p.loadURDF("plane.urdf")
        
    return client_id, plane_id

def stabilize_object(object_id, linear_damping=25, angular_damping=25):
    """
    Apply stabilization to a PyBullet object.
    
    Args:
        object_id: PyBullet object ID
        linear_damping: Linear damping coefficient
        angular_damping: Angular damping coefficient
    """
    p.resetBaseVelocity(object_id, [0, 0, 0], [0, 0, 0])
    p.changeDynamics(object_id, -1, linearDamping=linear_damping, angularDamping=angular_damping)

def parse_arguments():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description='Unified Kirigami Simulation')
    
    # File paths
    parser.add_argument('--vertices_file', type=str, required=True,
                      help='Path to 3D vertices file (12 values per line: x,y,z for 4 vertices)')
    parser.add_argument('--constraints_file', type=str, required=True,
                      help='Path to constraints file')
    parser.add_argument('--hull_file', type=str, default=None,
                      help='Path to hull file (optional)')
    
    # Force parameters
    parser.add_argument('--force_type', type=str, default='vertical',
                      choices=['vertical', 'normal', 'outward'],
                      help='Type of force to apply')
    parser.add_argument('--force_magnitude', type=float, default=DEFAULT_FORCE_MAGNITUDE,
                      help=f'Magnitude of applied forces (default: {DEFAULT_FORCE_MAGNITUDE})')
    parser.add_argument('--force_tiles', type=int, nargs='+',
                      help='Indices of specific tiles to apply forces to')
    
    # Brick parameters
    parser.add_argument('--brick_thickness', type=float, default=DEFAULT_BRICK_THICKNESS,
                      help=f'Thickness of bricks (default: {DEFAULT_BRICK_THICKNESS})')
    
    # Physics parameters
    parser.add_argument('--gravity', type=float, default=-9.81,
                      help='Gravity in z direction')
    parser.add_argument('--linear_damping', type=float, default=25,
                      help='Linear damping for bodies')
    parser.add_argument('--angular_damping', type=float, default=25,
                      help='Angular damping for bodies')
    
    # Simulation control
    parser.add_argument('--timestep', type=float, default=1/240,
                      help='Physics timestep')
    parser.add_argument('--substeps', type=int, default=10,
                      help='Physics substeps')
    parser.add_argument('--sim_steps', type=int, default=2400,
                      help='Number of simulation steps')
    parser.add_argument('--ground_plane', action='store_true',
                      help='Add ground plane')
    parser.add_argument('--keep_open', action='store_true',
                      help='Keep window open after simulation')
    
    
    return parser.parse_args()