"""
Unified Kirigami Simulation Script

This script provides a unified interface for kirigami simulation with different force types:
- vertical: Forces in the z direction 
- normal: Forces along the normal direction of each face
- outward: Forces radiating from the center of the structure

Note: This script expects 3D vertex data (12 values per line: x,y,z for 4 vertices).
      For 2D data, users must preprocess files by adding z=0 to each point.

Usage:
    python run_sim.py --vertices_file rigid_3by3_pattern_contracted_vertices.txt --constraints_file rigid_3by3_constraints.txt 
    --hull_file rigid_3by3_hull.txt --force_type outward --connection_mode both --ground_plane --brick_thickness 0.2 --gravity -50 --force_magnitude 100


    python run_sim.py --vertices_file cube2sphere_contracted_vertices.txt --constraints_file cube2sphere_constraints.txt
    --force_type normal  --force_magnitude 500 
"""
import os
import sys
import time
import numpy as np
import pybullet as p

# Ensure modules can be found
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import from existing modules
from utils.load_data import (load_vertices_from_file, load_constraints_from_file, load_hull_tiles)
from utils.setup import (parse_arguments, setup_physics_engine, create_ground_plane, stabilize_bodies)
from simulation.geometry import (create_3d_brick, create_brick_body, create_constraints_between_bricks)
from simulation.forces import get_force_direction_function, apply_force_to_bodies
from simulation.event_handler import EventHandler


def run_simulation(args):
    """Run the kirigami simulation with the specified parameters"""
    # Use local variables instead of global or nonlocal
    bricks = []
    normals = []
    
    # Store original simulation parameters
    original_sim_data = {}
    
    # Initialize physics engine
    client_id = setup_physics_engine(
        gravity=(0, 0, args.gravity),
        timestep=args.timestep,
        substeps=args.substeps
    )
    
    # Determine if we should show labels (disable for better performance with large simulations)
    show_labels = not (args.no_labels or args.performance_mode)
    if not show_labels:
        print("Tile labels disabled for better performance")
    
    # Set additional performance optimizations
    if args.performance_mode:
        print("Performance mode enabled - optimizing simulation settings")
        p.setPhysicsEngineParameter(enableConeFriction=0)
        p.setPhysicsEngineParameter(numSolverIterations=4)  # Default is 10
        # Disable debug visualization for better performance
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    
    # Set up camera
    p.resetDebugVisualizerCamera(
        cameraDistance=6.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0]
    )
    
    if args.ground_plane:
        create_ground_plane()
        
    # Define the simulation initialization function
    def initialize_simulation():
        nonlocal original_sim_data
        # Create new local lists for bricks and normals
        local_bricks = []
        local_normals = []
        
        # Load data
        vertices = load_vertices_from_file(args.vertices_file)
        constraints = load_constraints_from_file(args.constraints_file)
        
        # Store original data for reset capability
        original_sim_data['vertices'] = vertices.copy()
        original_sim_data['constraints'] = constraints.copy()
        
        # Load hull tiles if specified
        if args.hull_file and os.path.exists(args.hull_file):
            force_tiles = load_hull_tiles(args.hull_file)
        else:
            force_tiles = args.force_tiles if args.force_tiles else list(range(len(vertices)))
        
        print(f"Loaded {len(vertices)} tiles and {len(constraints)} constraints")
        print(f"Forces will be applied to {len(force_tiles)} tiles")
        
        # Get the appropriate force direction function
        force_function = get_force_direction_function(args.force_type)
        
        # Create bricks
        brick_centers = []
        top_vertices = []
        bottom_vertices = []
        
        # Create each brick
        for i, tile_vertices in enumerate(vertices):
            # Create brick geometry
            verts, indices, center, tile_bottom_vertices, tile_top_vertices, normal = create_3d_brick(
                tile_vertices, args.brick_thickness
            )
            
            # Create brick body in physics engine
            brick_id = create_brick_body(verts, indices, center)
            
            local_bricks.append(brick_id)
            brick_centers.append(center)
            top_vertices.append(tile_top_vertices)
            bottom_vertices.append(tile_bottom_vertices)
            local_normals.append(normal)
        
        # Stabilize bricks
        stabilize_bodies(local_bricks, 
                        linear_damping=args.linear_damping, 
                        angular_damping=args.angular_damping)
        
        # Create constraints between bricks
        connection_mode = 'bottom' if args.connection_mode is None else args.connection_mode
        create_constraints_between_bricks(local_bricks, constraints, bottom_vertices, top_vertices, brick_centers, connection_mode)
        
        # Prepare simulation data for event handler
        sim_data = {
            'vertices_file': args.vertices_file,
            'constraints_file': args.constraints_file,
            'hull_file': args.hull_file,
            'args': args,
            'vertices': vertices,
            'constraints': constraints,
            'force_tiles': force_tiles,
            'force_function': force_function,
            'bricks': local_bricks,
            'normals': local_normals,
            'brick_centers': brick_centers,
            'top_vertices': top_vertices,
            'bottom_vertices': bottom_vertices,
            'local_top_vertices': [[
                [v[k] - center[k] for k in range(3)]
                for v in tile_top_vertices
            ] for tile_top_vertices, center in zip(top_vertices, brick_centers)],
            'original_data': original_sim_data
        }
        
        return sim_data, force_tiles, force_function
    
    # Define the force application function
    def apply_forces(whole_center):
        
         force_tiles = event_handler.simulation_data['force_tiles']
         force_function = event_handler.simulation_data['force_function']
         force_magnitude = args.force_magnitude
         
         # Use the current brick and normal lists directly from the event handler
         current_bricks = event_handler.simulation_data['bricks']
         current_normals = event_handler.simulation_data['normals']
         
         # Apply forces to specific tiles
         force_brick_ids = [brick_id for idx, brick_id in enumerate(current_bricks) 
                         if idx in force_tiles and idx < len(current_bricks)]
         
         force_normals = [norm for idx, norm in enumerate(current_normals) 
                       if idx in force_tiles and idx < len(current_normals)]
         
         # Apply appropriate forces based on force type
         if args.force_type == 'outward':
             for i, body_id in enumerate(force_brick_ids):
                 center_pos, orientation = p.getBasePositionAndOrientation(body_id)
                 force_dir = force_function(center_pos, whole_center)
                 force = [force_magnitude * d for d in force_dir]
                 p.applyExternalForce(body_id, -1, force, center_pos, flags=p.WORLD_FRAME)
         else:
             apply_force_to_bodies(force_brick_ids, force_function, force_magnitude, force_normals)
    
    # Initialize simulation for the first time
    sim_data, force_tiles, force_function = initialize_simulation()
    
    # Create simulation functions dict
    simulation_functions = {
        'initialize_simulation': initialize_simulation,
        'apply_forces': apply_forces
    }
    
    # Create event handler
    event_handler = EventHandler(sim_data, simulation_functions, show_labels=show_labels)
    
    # Wait for the scene to stabilize
    for _ in range(100):
        p.stepSimulation()
        time.sleep(args.timestep)
    
    # Set up UI controls and start interactive simulation
    print("Starting interactive simulation...")
    print("Controls available through GUI sliders:")
    print("  - Reset: Drag 'Reset simulation' slider above 0.5")
    print("  - Save: Drag 'Save vertices' slider above 0.5")
    print("  - Delete: Enter the tile index shown on the tile, then press the delete button")
    print("  - Show/Hide Labels: Toggle tile index labels visibility")
    print("  - Label update frequency: Control how often labels are updated")
    
    # Performance tips
    num_bricks = len(sim_data['bricks'])
    if num_bricks > 100 and show_labels:
        print(f"\nPerformance tip: Your simulation has {num_bricks} tiles.")
        print("For better performance with large simulations, use --no-labels or --performance-mode")
    
    # Set up UI controls
    event_handler.setup_ui_controls()
    
    # Main simulation loop
    try:
        while p.isConnected():
            # Handle UI events (buttons, sliders, etc.)
            result = event_handler.handle_ui_events()
            if result:  # If simulation was reset
                sim_data = result
                # No need to update local variables as we now use event_handler's data directly
            
            # Step the simulation (includes force application and label updates)
            event_handler.step_simulation()
            
            # Pause to maintain frame rate
            time.sleep(args.timestep)
            
    except KeyboardInterrupt:
        print("Exiting simulation...")
    finally:
        p.disconnect()


if __name__ == "__main__":
    args = parse_arguments()
    
    # Handle relative paths for data files
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
    
    if not os.path.isabs(args.vertices_file):
        potential_path = os.path.join(data_dir, args.vertices_file)
        if os.path.exists(potential_path):
            args.vertices_file = potential_path
    
    if not os.path.isabs(args.constraints_file):
        potential_path = os.path.join(data_dir, args.constraints_file)
        if os.path.exists(potential_path):
            args.constraints_file = potential_path
    
    if args.hull_file and not os.path.isabs(args.hull_file):
        potential_path = os.path.join(data_dir, args.hull_file)
        if os.path.exists(potential_path):
            args.hull_file = potential_path
    
    # Run the simulation
    run_simulation(args)