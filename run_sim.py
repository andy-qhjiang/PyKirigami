"""
Kirigami Simulation Script with Target-Based Deployment

This script provides a simplified interface for kirigami simulation with target-driven forces
for controlled deployment of kirigami structures.

Note: This script expects 3D vertex data with:
      - 3*n values per line (x,y,z for n vertices)
      
      For 2D data, users must preprocess files by adding z=0 to each point.

Usage:
    # Basic simulation with physics only (no deployment forces)
    python run_sim.py --vertices_file fan_R10_r1_w3_h3_vertices.txt --constraints_file fan_R10_r1_w3_h3_constraints.txt  --ground_plane --gravity -200 --brick_thickness 0.1

    python run_sim.py --vertices_file tangram_vertices.txt --constraints_file tangram_constraints.txt --ground_plane --gravity -100 --brick_thickness 0.1

    # Target-based deployment examples:
    python run_sim.py --vertices_file cylinder_vertices.txt --constraints_file cylinder_constraints.txt --target_vertices_file cylinder_target.txt  --brick_thickness 0.1 --camera_distance 15

    python run_sim.py --vertices_file cube2sphere_w4_h4_vertices.txt --constraints_file cube2sphere_w4_h4_constraints.txt --target_vertices_file cube2sphere_w4_h4_target.txt --brick_thickness 0.02

    python run_sim.py --vertices_file partialSphere_vertices.txt --constraints_file partialSphere_constraints.txt --target_vertices_file partialSphere_target.txt  --brick_thickness 0.02
"""
import os
import sys
import time
import numpy as np
import pybullet as p

# Ensure modules can be found
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import from existing modules
from utils.setup import *
from utils.physics_utils import setup_physics_engine, create_ground_plane
from core.simulation import Simulation
from core.event_handler import EventHandler
from core.interactive_controls import InteractiveControls


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
    
    # Set up camera
    p.resetDebugVisualizerCamera(
        cameraDistance=args.camera_distance if hasattr(args, 'camera_distance') else 8.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0]
    )
    
    if args.ground_plane:
        create_ground_plane()
    
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
    
    if args.target_vertices_file and not os.path.isabs(args.target_vertices_file):
        potential_path = os.path.join(data_dir, args.target_vertices_file)
        if os.path.exists(potential_path):
            args.target_vertices_file = potential_path
    
    # Run the simulation
    run_simulation(args)
