"""
Setup functions for the kirigami simulation.

This module includes functions to set up the physics engine and prepare
the simulation environment.
"""
import argparse
import pybullet as p
import pybullet_data
import platform
import psutil

def parse_arguments():
    """Parse command-line arguments for the simulation"""
    parser = argparse.ArgumentParser(description='Run kirigami simulation')
    
    # Input files
    parser.add_argument('--vertices_file', required=True, help='File containing vertex data')
    parser.add_argument('--constraints_file', required=True, help='File with connectivity constraints')
    parser.add_argument('--hull_file', help='Optional file specifying hull tiles for force application')
    
    # Physics simulation parameters
    parser.add_argument('--gravity', type=float, default=0, help='Gravity constant')
    parser.add_argument('--timestep', type=float, default=0.001, help='Physics simulation timestep')
    parser.add_argument('--substeps', type=int, default=10, help='Physics substeps per step')
    parser.add_argument('--linear_damping', type=float, default=0.1, help='Linear damping')
    parser.add_argument('--angular_damping', type=float, default=0.1, help='Angular damping')
    
    # Force parameters
    parser.add_argument('--force_type', choices=['vertical', 'normal', 'outward'], default='normal',
                       help='Type of force to apply (vertical, normal, or outward)')
    parser.add_argument('--force_magnitude', type=float, default=100, 
                       help='Magnitude of the force applied to each tile')
    parser.add_argument('--force_tiles', type=int, nargs='+',
                       help='Indices of specific tiles to apply forces to (default: all)')
    
    # Geometry parameters
    parser.add_argument('--brick_thickness', type=float, default=0.1,
                       help='Thickness of the brick (z-height)')
    parser.add_argument('--connection_mode', choices=['top', 'bottom', 'both'], default='both',
                       help='How bricks should connect: top layer, bottom layer, or both')
    
    # Visual and performance options
    parser.add_argument('--no_labels', action='store_true', help='Disable tile labels')
    parser.add_argument('--performance_mode', action='store_true', 
                       help='Enable performance optimizations')
    parser.add_argument('--ground_plane', action='store_true',
                       help='Add a ground plane to the simulation')
    parser.add_argument('--num_threads', type=int, default=0,
                       help='Number of CPU threads to use (0=auto-detect)')
    parser.add_argument('--use_gpu', action='store_true',
                       help='Enable GPU acceleration for physics if available')
    
    return parser.parse_args()

def setup_physics_engine(gravity=(0, 0, 0), timestep=0.001, substeps=10, num_threads=0, use_gpu=False):
    """
    Set up the PyBullet physics engine.
    
    Args:
        gravity: Tuple of (x, y, z) gravity vector (default: no gravity)
        timestep: Simulation timestep (default: 0.001s)
        substeps: Number of substeps per simulation step
        num_threads: Number of threads to use for physics calculation (0=auto)
        use_gpu: Whether to use GPU acceleration for physics
        
    Returns:
        client_id: The ID of the PyBullet client
    """
    # Use GUI interface
    client_id = p.connect(p.GUI)
    
    # Set up path for PyBullet data
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set gravity
    p.setGravity(gravity[0], gravity[1], gravity[2])
    
    # Auto-detect optimal thread count if not specified
    if num_threads <= 0:
        num_threads = psutil.cpu_count(logical=False)  # Physical cores only
        if num_threads < 2:
            num_threads = 2  # Minimum 2 threads
    
    # Configure physics simulation parameters - avoid using numThreads directly
    p.setPhysicsEngineParameter(
        fixedTimeStep=timestep,
        numSolverIterations=substeps,
        numSubSteps=substeps,
        solverResidualThreshold=1e-8  # More precise simulation
    )
    
    # Set number of threads separately using direct API call - this is more version-compatible
    try:
        # Try the direct thread count setting first (older PyBullet versions)
        p.setNumThreads(num_threads)
        print(f"Set physics thread count to {num_threads} using setNumThreads")
    except Exception as e:
        # Fall back to environment variable as a last resort
        print(f"Could not set thread count directly: {e}")
        print(f"Setting thread count via OMP_NUM_THREADS environment variable")
        import os
        os.environ['OMP_NUM_THREADS'] = str(num_threads)
    
    # Try to enable GPU acceleration if requested
    if use_gpu:
        try:
            # Set solver to use GPU (experimental in PyBullet)
            p.setPhysicsEngineParameter(enableSAT=1)  # Enable GPU-based collision detection
            p.setPhysicsEngineParameter(useAabb=1)  # Use axis-aligned bounding box
            p.setPhysicsEngineParameter(allowedCcdPenetration=0.0001)  # Better continuous collision detection
            print(f"Attempting to enable GPU acceleration with {num_threads} threads")
        except Exception as e:
            print(f"Warning: Could not enable full GPU acceleration: {e}")
    else:
        print(f"Running with CPU physics using {num_threads} threads")
    
    # Debug info about physics parameters
    params = p.getPhysicsEngineParameters()
    print(f"Physics engine parameters: {params.get('numThreads', 'unknown')} threads, {timestep}s timestep, {substeps} substeps")
    
    # Set additional parameters for better performance
    p.setPhysicsEngineParameter(enableConeFriction=1)
    p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)
    
    # Configure the visualizer for best performance vs. quality trade-off
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    
    return client_id

def create_ground_plane():
    """Create a ground plane in the simulation"""
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_PLANE, rgbaColor=[0.9, 0.9, 0.9, 1]),
        baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_PLANE),
        basePosition=[0, 0, -0.5]
    )

def stabilize_bodies(body_ids, linear_damping=0.1, angular_damping=0.1):
    """
    Apply damping to bodies to stabilize the simulation.
    
    Args:
        body_ids: List of body IDs to apply damping to
        linear_damping: Linear damping coefficient
        angular_damping: Angular damping coefficient
    """
    for body_id in body_ids:
        p.changeDynamics(
            body_id,
            -1,
            linearDamping=linear_damping,
            angularDamping=angular_damping
        )