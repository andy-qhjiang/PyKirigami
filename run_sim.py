"""
Kirigami Simulation Script with Target-Based Deployment

This script provides a simplified interface for kirigami simulation with target-driven forces
for controlled deployment of kirigami structures.

Note: This script expects 3D vertex data with:
      - 3*n values per line (x,y,z for n vertices)
      
      For 2D data, users must preprocess files by adding z=0 to each point.

Usage:
    # --- Recommended: model folder usage ---
    # Each model placed in data/<model>/ with files:
    #   vertices.txt   (required)
    #   constraints.txt (can be empty if no constraints)
    #   target.txt     (optional)

    # Basic simulation with physics only (no deployment forces)
    python run_sim.py --model fan --ground_plane --brick_thickness 0.5 --gravity -200

    # cm_expansion deployment (no target file needed)
    python run_sim.py --model stampfli24 --ground_plane --brick_thickness 0.5 --cm_expansion --camera_distance 12

    # Target-based deployment examples (target.txt present in model folder)
    python run_sim.py --model cylinder --brick_thickness 0.1 --camera_distance 15
    python run_sim.py --model cube2sphere_w3_h3 --brick_thickness 0.02
    python run_sim.py --model partialSphere --brick_thickness 0.02 --ground_plane
    python run_sim.py --model heart --ground_plane --brick_thickness 0.5
    python run_sim.py --model square2disk --ground_plane --brick_thickness 0.5

"""
import os
import sys
import time
import numpy as np
import pybullet as p

# Ensure modules can be found
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import from existing modules
from utils.config import *
from utils.physics_utils import setup_physics_engine
from utils.export_info import export_obj_brick, export_obj_bottom
from core.simulation import Simulation
from core.simulation_controller import SimulationController
from core.interaction_controller import InteractionController

def run_simulation(args):
    """Run the kirigami simulation with the specified parameters"""
    
    # Initialize physics engine
    setup_physics_engine(
        gravity=(0, 0, args.gravity),
        timestep=args.timestep,
        substeps=args.substeps
    )
    
    # Configure debug visualizer - hide GUI panels for cleaner view
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Hide GUI panels
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # Enable shadows for better visualization
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0) # avoid conflict between default shortcuts and user-defined shortcuts

    # Set up camera
    p.resetDebugVisualizerCamera(
        cameraDistance=args.camera_distance if hasattr(args, 'camera_distance') else 8.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0]
    )
    
    # Create simulation instance
    simulation = Simulation(args)
    
    # Define wrapper functions for the event handler
    def initialize_simulation():
        return simulation.initialize()
    
    def apply_forces():
        simulation.apply_forces()
    
    # Initialize simulation for the first time
    sim_data = initialize_simulation()
    
    
    # Create simulation functions dict
    simulation_functions = {
        'initialize_simulation': initialize_simulation,
        'apply_forces': apply_forces
    }
    
    # Create simulation controller (replaces event handler)
    simulation_controller = SimulationController(sim_data, simulation_functions)

    # Create interaction controller
    interaction_controller = InteractionController(sim_data)
    
    # Set up interactive simulation with keyboard controls
    print("Starting interactive simulation...")
    print("Keyboard Controls:")
    print("  R - Reset simulation")
    print("  S - Save vertex locations")
    print("  Space - Toggle pause/resume")
    print("  Q - Quit simulation")
    print("  O - Export current state as OBJ")
    print("Mouse Controls:")
    print("  Right-click on a brick - Toggle fix/unfix")
    
    # Main simulation loop
    try:
        while p.isConnected():
            # Handle keyboard events
            keys = p.getKeyboardEvents()
            
            # Process keyboard inputs
            if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                print("Resetting simulation...")
                # Reset the state of interaction controller (clear fixed objects)
                interaction_controller.reset()
                # Reset simulation via simulation controller
                simulation_controller.reset_simulation()                
                
                print("Simulation reset completed.")

            if ord('s') in keys and keys[ord('s')] & p.KEY_WAS_TRIGGERED:
                simulation_controller.save_vertex_locations()

            if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
                simulation_controller.toggle_pause()

            if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                print("Quitting simulation...")
                break
            if ord('o') in keys and keys[ord('o')] & p.KEY_WAS_TRIGGERED:
                os.makedirs("output", exist_ok=True)
                base = os.path.join("output", f"scene_{int(time.time())}_obj")
                # we export two obj files here
                export_obj_brick(
                    file_path=base,
                    bricks=simulation_controller.simulation_data['bricks'],
                    visual_mesh=simulation_controller.simulation_data['visual_mesh'],
                )
                base = os.path.join("output", f"scene_{int(time.time())}_bottom.obj")
                export_obj_bottom(
                    file_path=base,
                    bricks=simulation_controller.simulation_data['bricks'],
                    local_bottom_vertices=simulation_controller.simulation_data['local_coords'],
                )

            # Process mouse events for interaction controller (e.g., toggling fixed state)
            interaction_controller.process_mouse_events() 
            
            # Step simulation (calculate all forces and detect collision in Bullet engine)
            simulation_controller.step_simulation()

            time.sleep(args.timestep)
            
    except KeyboardInterrupt:
        print("Exiting simulation...")
    finally:
        p.disconnect()


if __name__ == "__main__":
    args = parse_arguments()
    
    # Handle relative paths for data files (with optional --model)
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')

    
    model_dir = os.path.join(data_dir, args.model)
    if not os.path.isdir(model_dir):
        print(f"ERROR: Model folder '{args.model}' not found under data/")
        sys.exit(1)
    # Prepend model directory if paths are relative
    if not os.path.isabs(args.vertices_file):
        args.vertices_file = os.path.join(model_dir, args.vertices_file)
    if not os.path.isabs(args.constraints_file):
        args.constraints_file = os.path.join(model_dir, args.constraints_file)
    if args.target_vertices_file and not os.path.isabs(args.target_vertices_file):
        candidate = os.path.join(model_dir, args.target_vertices_file)
        if os.path.exists(candidate):
            args.target_vertices_file = candidate
        else:
            # Allow absence silently
            args.target_vertices_file = None
    
    # Run the simulation
    run_simulation(args)
