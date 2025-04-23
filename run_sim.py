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
from simulation.geometry import (create_3d_brick, create_brick_body, create_point_constraint,
                              create_constraints_between_bricks)
from simulation.forces import get_force_direction_function, apply_force_to_bodies


def run_simulation(args):
    """Run the kirigami simulation with the specified parameters"""
    global bricks, normals
    
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
    
    
    # Load data
    vertices = load_vertices_from_file(args.vertices_file)
    constraints = load_constraints_from_file(args.constraints_file)
    
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
    bricks = []
    brick_centers = []
    top_vertices = []
    normals = []
    
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
    
    # Simulation loop
    start_time = time.time()
    print("Starting simulation...")
    
    for step in range(args.sim_steps):
        
        current_positions = [p.getBasePositionAndOrientation(brick_id)[0] 
                               for brick_id in bricks]
        whole_center = np.mean(current_positions, axis=0)
    
        # Apply forces to specific tiles
        force_brick_ids = [bricks[idx] for idx in force_tiles if idx < len(bricks)]
        force_normals = [normals[idx] for idx in force_tiles if idx < len(bricks)]
        
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
    
    # Keep window open if requested
    if args.keep_open:
        print("Simulation completed. Press Ctrl+C to exit...")
        try:
            while p.isConnected():
                
                current_positions = [p.getBasePositionAndOrientation(brick_id)[0] 
                                      for brick_id in bricks]
                whole_center = np.mean(current_positions, axis=0)
                
                # Continue applying forces if needed
                if args.force_type == 'outward':
                    for i, body_id in enumerate(force_brick_ids):
                        center_pos, orientation = p.getBasePositionAndOrientation(body_id)
                        force_dir = force_function(center_pos, whole_center)
                        force = [args.force_magnitude * d for d in force_dir]
                        p.applyExternalForce(body_id, -1, force, center_pos, flags=p.WORLD_FRAME)
                else:
                    apply_force_to_bodies(force_brick_ids, force_function, args.force_magnitude, force_normals)
                
                p.stepSimulation()
                time.sleep(args.timestep)
        except KeyboardInterrupt:
            print("Exiting...")
    
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
    
    # Run the simulation
    run_simulation(args)