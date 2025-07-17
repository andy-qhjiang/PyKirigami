"""
Simplified Kirigami Simulation Script with Auto-Expansion

This script provides a simplified interface for kirigami simulation with spring-like forces
for automatic expansion of contracted structures.

Note: This script expects 3D vertex data with:
      - 3*n values per line (x,y,z for 3 vertices)
      
      For 2D data, users must preprocess files by adding z=0 to each point.

Usage:
    python run_sim.py --vertices_file cube2sphere_contracted_vertices.txt --constraints_file cube2sphere_constraints.txt --auto_expand --spring_radius 10.0 --spring_stiffness 250.0 --spring_damping 2.0 --gravity 0

    python run_sim.py --vertices_file rigid_3by3_vertices.txt --constraints_file rigid_3by3_constraints.txt --auto_expand --spring_radius 10.0 --spring_stiffness 100.0 --spring_damping 2.0 --angular_damping 2.5 --linear_damping 2.5 --ground_plane --gravity -100


    python run_sim.py --vertices_file tangram_vertices.txt --constraints_file tangram_constraints.txt --angular_damping 2.5 --linear_damping 2.5 --ground_plane --gravity -100 --brick_thickness 0.2

    python run_sim.py --vertices_file fan_R10_r1_w3_h3_vertices.txt --constraints_file fan_R10_r1_w3_h3_constraints.txt --angular_damping 10 --linear_damping 10 --ground_plane --gravity -100 --brick_thickness 0.1
"""
import os
import sys
import time
import numpy as np
import pybullet as p

# Ensure modules can be found
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import from existing modules
from utils.load_data import *
from utils.setup import (parse_arguments, setup_physics_engine, create_ground_plane, stabilize_bodies)
from utils.geometry import (create_3d_brick, create_brick_body, create_constraints_between_bricks)
from simulation.spring_forces import apply_spring_forces
from simulation.event_handler import EventHandler
from simulation.interactive_controls import InteractiveControls


def run_simulation(args):
    """Run the kirigami simulation with the specified parameters"""

    # Store original simulation parameters
    original_sim_data = {}
    
    # Initialize physics engine
    setup_physics_engine(
        gravity=(0, 0, args.gravity),
        timestep=args.timestep,
        substeps=args.substeps
    )
    
    # Configure debug visualizer - hide GUI panels for cleaner view
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Hide GUI panels
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)  # Enable shadows for better visualization
    
    # Set up camera
    p.resetDebugVisualizerCamera(
        cameraDistance=8.0,
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
        force_bricks = load_force_bricks(args.force_bricks_file) if args.force_bricks_file else list(range(len(vertices)))

        # Store original data for reset capability
        original_sim_data['vertices'] = vertices.copy()
        original_sim_data['constraints'] = constraints.copy()
        original_sim_data['force_bricks'] = force_bricks.copy()        
        
        
        print(f"Loaded {len(vertices)} bricks and {len(constraints)} constraints")
        print(f"Forces will be applied to {len(force_bricks)} bricks")
        
        # Create bricks
        brick_centers = []
        top_vertices = []
        bottom_vertices = []
        
        # Create each brick
        for brick_vertices in vertices:
            # Create brick geometry - now returns visual indices
            (verts, visual_indices, center, 
             brick_bottom_vertices, brick_top_vertices, normal) = create_3d_brick(
                brick_vertices, args.brick_thickness
            )
            
            # Create brick body in physics engine - pass visual indices
            brick_id = create_brick_body(verts, visual_indices, center)
            
            local_bricks.append(brick_id)
            brick_centers.append(center)
            top_vertices.append(brick_top_vertices)
            bottom_vertices.append(brick_bottom_vertices)
            local_normals.append(normal)
        
        # Stabilize bricks
        stabilize_bodies(local_bricks, 
                        linear_damping=args.linear_damping, 
                        angular_damping=args.angular_damping)
        
        # Create constraints between bricks
        constraint_ids= create_constraints_between_bricks(
            local_bricks, constraints, bottom_vertices, top_vertices, brick_centers
        )
        
        # Prepare simulation data for event handler
        sim_data = {
            'vertices_file': args.vertices_file,
            'constraints_file': args.constraints_file,
            'args': args,
            'vertices': vertices,
            'constraints': constraints,
            'force_bricks': force_bricks,
            'bricks': local_bricks,
            'normals': local_normals,
            'brick_centers': brick_centers,
            'top_vertices': top_vertices,
            'bottom_vertices': bottom_vertices,
            'local_top_vertices': [[
                [v[k] - center[k] for k in range(3)]
                for v in brick_top_vertices
            ] for brick_top_vertices, center in zip(top_vertices, brick_centers)],
            'constraint_ids': constraint_ids,
            'original_data': original_sim_data
        }
        
        return sim_data
    
    # Define the force application function
    def apply_forces(whole_center):
        # If auto_expand is not enabled, no forces will be applied
        if not args.auto_expand:
            return
        
        force_bricks = event_handler.simulation_data['force_bricks']
        
        # Use the current brick list directly from the event handler
        current_bricks = event_handler.simulation_data['bricks']
        
        # Apply forces to specific bricks
        force_brick_ids = [brick_id for idx, brick_id in enumerate(current_bricks) 
                        if idx in force_bricks and idx < len(current_bricks)]
        
        # Setup spring parameters
        spring_params = {
            'radius': args.spring_radius,
            'stiffness': args.spring_stiffness,
            'damping': args.spring_damping,
        }
        
        # Apply spring forces to all force bricks
        apply_spring_forces(force_brick_ids, whole_center, spring_params)
    
    # Initialize simulation for the first time
    sim_data = initialize_simulation()
    
    # Create simulation functions dict
    simulation_functions = {
        'initialize_simulation': initialize_simulation,
        'apply_forces': apply_forces
    }
    
    # Create event handler
    event_handler = EventHandler(sim_data, simulation_functions)
    
    # Create interactive controls
    interactive_controls = InteractiveControls(sim_data)
    
    # Set up interactive simulation with keyboard controls
    print("Starting interactive simulation...")
    print("Keyboard Controls:")
    print("  R - Reset simulation")
    print("  S - Save vertex locations")
    print("  P - Toggle pause/resume")
    print("  Q - Quit simulation")
    print("Mouse Controls:")
    print("  Right-click on a brick - Toggle fix/unfix (red sphere indicates fixed)")
    
    # Main simulation loop
    try:
        while p.isConnected():
            # Handle keyboard events
            keys = p.getKeyboardEvents()
            
            # Process keyboard inputs
            if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                print("Resetting simulation...")
                # Reset the state of interactive controls (e.g., clear fixed objects)
                interactive_controls.reset()
                # Reset simulation via event handler (which calls controller)
                sim_data = event_handler.reset_simulation()                
                # Update interactive controls with the new simulation data
                interactive_controls.update_simulation_data(sim_data)
                
                print("Simulation reset and interactive controls updated.")

            if ord('s') in keys and keys[ord('s')] & p.KEY_WAS_TRIGGERED:
                event_handler.save_vertex_locations()

            if ord('p') in keys and keys[ord('p')] & p.KEY_WAS_TRIGGERED:
                event_handler.toggle_pause()

            if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                print("Quitting simulation...")
                break
            
            # Process mouse events for interactive controls (e.g., toggling fixed state)
            interactive_controls.process_mouse_events() 
            
            # Step simulation (applies forces, calls p.stepSimulation())
            event_handler.step_simulation() 
            
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
    
    if args.force_bricks_file and not os.path.isabs(args.force_bricks_file):
        potential_path = os.path.join(data_dir, args.force_bricks_file)
        if os.path.exists(potential_path):
            args.force_bricks_file = potential_path
    
    # Run the simulation
    run_simulation(args)
