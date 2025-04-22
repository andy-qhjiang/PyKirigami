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
import pybullet_data

# Ensure modules can be found
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import from existing modules
from utils.load_data import (load_vertices_from_file, load_constraints_from_file, load_hull_tiles)
from utils.setup import parse_arguments
from simulation.physics import setup_physics_engine, create_ground_plane, stabilize_bodies
from simulation.geometry import (create_3d_brick, create_brick_body, create_point_constraint,
                              calculate_outward_direction, calculate_normal_direction)


# Force direction functions
def get_normal_direction(body_id, pos, orientation, normal):
    """Return force direction along face normal"""
    return calculate_normal_direction(body_id, pos, orientation, normal)

def get_outward_direction(body_id, pos, orientation, normal):
    """Return force direction outward from structure center"""
    current_positions = [p.getBasePositionAndOrientation(b)[0] for b in bricks]
    whole_center = np.mean(current_positions, axis=0)
    return calculate_outward_direction(pos, whole_center)

def run_simulation(args):
    """Run the kirigami simulation with the specified parameters"""
    global bricks, normals
    
    # Initialize physics
    client_id = setup_physics_engine(
        gravity=(0, 0, args.gravity),
        timestep=args.timestep,
        substeps=args.substeps
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
    
    # Set force function based on type
    force_functions = {
        'normal': get_normal_direction,
        'outward': get_outward_direction
    }
    get_force_direction = force_functions.get(args.force_type, get_normal_direction)
    
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
    create_constraints(constraints, top_vertices, brick_centers)
    
    # Simulation loop
    start_time = time.time()
    print("Starting simulation...")
    
    for step in range(args.sim_steps):
        # Update camera if rotating
        if args.rotating_camera and step % 10 == 0:
            current_positions = [p.getBasePositionAndOrientation(brick_id)[0] 
                               for brick_id in bricks]
            whole_center = np.mean(current_positions, axis=0)
    
        # Apply forces
        apply_forces(force_tiles, args.force_magnitude, get_force_direction)
        
        # Step simulation
        p.stepSimulation()
        time.sleep(args.timestep)
    
    # Keep window open if requested
    if args.keep_open:
        print("Simulation completed. Press Ctrl+C to exit...")
        try:
            while p.isConnected():
                if args.rotating_camera:
                    current_positions = [p.getBasePositionAndOrientation(brick_id)[0] 
                                      for brick_id in bricks]
                    whole_center = np.mean(current_positions, axis=0)
                
                p.stepSimulation()
                time.sleep(args.timestep)
        except KeyboardInterrupt:
            print("Exiting...")
    
    p.disconnect()

def create_constraints(constraints, top_vertices, brick_centers):
    """Create constraints between bricks"""
    for constraint in constraints:
        f_i, v_j, f_p, v_q = constraint
        
        # Skip invalid constraints
        if f_i >= len(top_vertices) or f_p >= len(top_vertices) or v_j >= 4 or v_q >= 4:
            print(f"Warning: Skipping invalid constraint: {constraint}")
            continue
        
        # Get vertices and centers
        vertex_i_global = top_vertices[f_i][v_j]
        vertex_p_global = top_vertices[f_p][v_q]
        center_i = brick_centers[f_i]
        center_p = brick_centers[f_p]
        
        # Calculate pivot points in each body's local coordinates
        pivot_in_i = [vertex_i_global[k] - center_i[k] for k in range(3)]
        pivot_in_p = [vertex_p_global[k] - center_p[k] for k in range(3)]
        
        # Create constraint
        create_point_constraint(bricks[f_i], bricks[f_p], pivot_in_i, pivot_in_p)

def apply_forces(force_tiles, magnitude, get_force_direction):
    """Apply forces to specified tiles"""
    for tile_idx in force_tiles:
        if tile_idx < len(bricks):
            brick_id = bricks[tile_idx]
            pos, orientation = p.getBasePositionAndOrientation(brick_id)
            
            # Get normal direction for this tile
            normal = normals[tile_idx]
            
            # Calculate force direction using the provided function
            direction = get_force_direction(brick_id, pos, orientation, normal)
            
            # Apply the force
            force = [magnitude * d for d in direction]
            p.applyExternalForce(
                brick_id, -1, force, pos, flags=p.WORLD_FRAME
            )

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