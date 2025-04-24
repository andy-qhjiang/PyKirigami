"""
Unified Kirigami Simulation Script

This script provides a unified interface for kirigami simulation with different force types:
- vertical: Forces in the z direction 
- normal: Forces along the normal direction of each face
- outward: Forces radiating from the center of the structure

Note: This script expects 3D vertex data (12 values per line: x,y,z for 4 vertices).
      For 2D data, users must preprocess files by adding z=0 to each point.

Usage:
    python unified_sim.py --vertices_file [file] --constraints_file [file] --force_type [type]
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
    global bricks, normals, event_handler
    
    # Store original simulation parameters for potential reset
    original_sim_data = {}
    
    # Initialize physics
    client_id = setup_physics_engine(
        gravity=(0, 0, args.gravity),
        timestep=args.timestep,
        substeps=args.substeps
    )
    
    # Set up camera for better visualization
    p.resetDebugVisualizerCamera(
        cameraDistance=6.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0]
    )
    
    if args.ground_plane:
        create_ground_plane()
    
    def initialize_simulation():
        nonlocal original_sim_data
        
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
        bricks.clear()
        normals.clear()
        brick_centers = []
        top_vertices = []
        
        for i, tile_vertices in enumerate(vertices):
            # Create brick geometry
            verts, indices, center, tile_top_vertices, normal = create_3d_brick(
                tile_vertices, args.brick_thickness
            )
            
            # Create brick body in physics engine
            brick_id = create_brick_body(verts, indices, center)
            
            bricks.append(brick_id)
            brick_centers.append(center)
            top_vertices.append(tile_top_vertices)
            normals.append(normal)
        
        # Stabilize bricks
        stabilize_bodies(bricks, 
                        linear_damping=args.linear_damping, 
                        angular_damping=args.angular_damping)
        
        # Create constraints between bricks
        create_constraints_between_bricks(bricks, constraints, top_vertices, brick_centers)
        
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
            'bricks': bricks,
            'normals': normals,
            'brick_centers': brick_centers,
            'top_vertices': top_vertices,
            'original_data': original_sim_data
        }
        
        return sim_data, force_tiles, force_function
    
    # Initialize simulation for the first time
    sim_data, force_tiles, force_function = initialize_simulation()
    
    # Create event handler
    event_handler = EventHandler(sim_data)
    
    # Wait for the scene to stabilize
    for _ in range(100):
        p.stepSimulation()
        time.sleep(args.timestep)
    
    # Interactive simulation loop
    print("Starting interactive simulation...")
    print("Controls:")
    print("  'r' - Reset simulation")
    print("  'v' - Save vertices to file for MATLAB")
    print("  'd' - Toggle delete mode (then click on a tile to delete it)")
    print("  'Ctrl+C' - Exit simulation")
    
    # After stabilization, create debug sliders for interactive controls
    reset_param = p.addUserDebugParameter("Reset simulation", 0, 1, 0)
    save_param = p.addUserDebugParameter("Save vertices", 0, 1, 0)
    delete_param = p.addUserDebugParameter("Delete tile index", 0, len(bricks) - 1, -1)
    last_delete_idx = -1
    
    try:
        while p.isConnected():
            # Poll debug parameters instead of keyboard events
            if p.readUserDebugParameter(reset_param) > 0.5:
                print("Reset slider triggered")
                # Reset the slider
                p.resetUserDebugParameter(reset_param, 0)
                # Reinitialize
                for b in bricks:
                    p.removeBody(b)
                sim_data, force_tiles, force_function = initialize_simulation()
                event_handler = EventHandler(sim_data)
                continue
            if p.readUserDebugParameter(save_param) > 0.5:
                print("Save slider triggered")
                p.resetUserDebugParameter(save_param, 0)
                event_handler._save_vertex_locations()
            delete_idx = int(p.readUserDebugParameter(delete_param))
            if delete_idx != last_delete_idx and 0 <= delete_idx < len(bricks):
                print(f"Deleting via slider index: {delete_idx}")
                event_handler._remove_brick(bricks[delete_idx])
            last_delete_idx = delete_idx
            
            # Calculate the center of the structure
            current_positions = [p.getBasePositionAndOrientation(brick_id)[0] 
                                for brick_id in bricks]
            if current_positions:  # Check if there are any bricks left
                whole_center = np.mean(current_positions, axis=0)
            
                # Apply forces to specific tiles
                force_brick_ids = [brick_id for idx, brick_id in enumerate(bricks) 
                                if idx in force_tiles and idx < len(bricks)]
                
                force_normals = [norm for idx, norm in enumerate(normals) 
                              if idx in force_tiles and idx < len(normals)]
                
                # Pass whole_center for outward force calculation
                if args.force_type == 'outward':
                    for i, body_id in enumerate(force_brick_ids):
                        center_pos, orientation = p.getBasePositionAndOrientation(body_id)
                        force_dir = force_function(center_pos, whole_center)
                        force = [args.force_magnitude * d for d in force_dir]
                        p.applyExternalForce(body_id, -1, force, center_pos, flags=p.WORLD_FRAME)
                else:
                    apply_force_to_bodies(force_brick_ids, force_function, args.force_magnitude, force_normals)
            
            # Step simulation
            p.stepSimulation()
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
    
    # Global variables
    bricks = []
    normals = []
    event_handler = None
    
    # Run the simulation
    run_simulation(args)