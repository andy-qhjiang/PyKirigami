"""
Simplified Kirigami Simulation Script with Target-Based Deployment

This script provides a simplified interface for kirigami simulation with target-driven forces
for controlled deployment of kirigami structures.

Note: This script expects 3D vertex data with:
      - 3*n values per line (x,y,z for n vertices)
      
      For 2D data, users must preprocess files by adding z=0 to each point.

Usage:
    # Basic simulation with physics only (no deployment forces)
    python run_sim.py --vertices_file tangram_vertices.txt --constraints_file tangram_constraints.txt --angular_damping 2.5 --linear_damping 2.5 --ground_plane --gravity -100 --brick_thickness 0.2

    # Target-based deployment examples:
    python run_sim.py --vertices_file cylinder_vertices.txt --constraints_file cylinder_constraints.txt --target_vertices_file cylinder_target.txt  --brick_thickness 0.1 --camera_distance 15

    python run_sim.py --vertices_file cube2sphere_w4_h4_vertices.txt --constraints_file cube2sphere_w4_h4_constraints.txt --target_vertices_file cube2sphere_w4_h4_target.txt  --target_stiffness 500.0 --target_damping 10.0 --angular_damping 2.5 --linear_damping 2.5 --brick_thickness 0.02
    
    python run_sim.py --vertices_file fan_R10_r1_w3_h3_vertices.txt --constraints_file fan_R10_r1_w3_h3_constraints.txt  --ground_plane --gravity -100 --brick_thickness 0.1

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
from utils.load_data import *
from utils.setup import (parse_arguments, setup_physics_engine, create_ground_plane, stabilize_bodies)
from utils.geometry import (create_extruded_geometry, create_brick_body, create_constraints_between_bricks)
from utils.physics_utils import validate_constraints
from simulation.target_based_forces import (apply_target_based_forces, update_current_vertices_from_simulation)
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
        
    # Define the simulation initialization function
    def initialize_simulation():
        nonlocal original_sim_data
        # Create new local lists for bricks
        local_bricks = []
        
        # Load data
        vertices = load_vertices_from_file(args.vertices_file)
        constraints = load_constraints_from_file(args.constraints_file)
        
        # Load target vertices if using target-based deployment
        target_vertices = None
        if  args.target_vertices_file:
            target_vertices = load_vertices_from_file(args.target_vertices_file)
            if len(target_vertices) != len(vertices):
                print(f"Warning: Target vertices count ({len(target_vertices)}) doesn't match initial vertices count ({len(vertices)})")
                print("Disabling target-based deployment")
                target_vertices = None
            else:
                # Validate constraints in target vertices
                validate_constraints(target_vertices, constraints)   
        
        # Store original data for reset capability
        original_sim_data['vertices'] = vertices.copy()
        original_sim_data['constraints'] = constraints.copy()
        if target_vertices:
            original_sim_data['target_vertices'] = target_vertices.copy()        
        
        print(f"Loaded {len(vertices)} bricks and {len(constraints)} constraints")
        
        # Create bricks
        
        local_verts_list = []  # Store local vertices for each brick
        num_vertices_list = []  # Store number of bottom vertices for each brick
        
        # Create each brick
        for i, brick_vertices in enumerate(vertices):
            # Calculate number of vertices for this brick
            num_bottom_vertices = len(brick_vertices) // 3
            
            # Create extruded 3D geometry from polygon vertices
            (local_verts, visual_indices, center) = create_extruded_geometry(
                brick_vertices, args.brick_thickness
            )
            
            # Create brick body in physics engine
            brick_id = create_brick_body(local_verts, visual_indices, center)
            
            local_bricks.append(brick_id)
            # brick_centers.append(center)
            local_verts_list.append(local_verts)
            num_vertices_list.append(num_bottom_vertices)
            
            
            # Process target vertices if available (convert to same format as current_bottom)
            if target_vertices and i < len(target_vertices):
                target_brick_vertices = target_vertices[i]
                # Convert flat target vertices to 4×3 format like current_bottom
                target_vertices_reshaped = []
                for j in range(0, len(target_brick_vertices), 3):
                    target_vertices_reshaped.append([
                        target_brick_vertices[j],     # x
                        target_brick_vertices[j+1],   # y
                        target_brick_vertices[j+2]    # z
                    ])
                # Store target vertices for this brick (will be collected later)
                if 'target_bottom_vertices' not in locals():
                    target_bottom_vertices = []
                target_bottom_vertices.append(target_vertices_reshaped)
        
        # Initialize target_bottom_vertices if not created
        if 'target_bottom_vertices' not in locals():
            target_bottom_vertices = None
            

        # Stabilize bricks
        stabilize_bodies(local_bricks, 
                        linear_damping=args.linear_damping, 
                        angular_damping=args.angular_damping)
        
        # Create constraints between bricks
        constraint_ids = create_constraints_between_bricks(
            local_bricks, constraints, local_verts_list
        )
        
        # Prepare simulation data for event handler
        sim_data = {
            'args': args,
            'target_vertices': target_bottom_vertices,  # Use the properly formatted target vertices
            'bricks': local_bricks,
            'local_verts_list': local_verts_list,      # Local vertex coordinates
            'constraint_ids': constraint_ids,
            'original_data': original_sim_data
        }
        
        return sim_data
    
    # Define the force application function
    def apply_forces():
        current_bricks = event_handler.simulation_data['bricks']
        
        # Check if we should use target-based deployment
        if event_handler.simulation_data.get('target_vertices'):
            
            # Compute local bottom vertices on-demand from stored local_verts
            local_verts_list = event_handler.simulation_data['local_verts_list']
            
            # Create local bottom vertices for each brick
            local_bottom_vertices = []
            for local_verts in local_verts_list:
                num_bottom = len(local_verts) // 2  # Compute number of bottom vertices
                # Bottom vertices are the first num_bottom vertices in local_verts
                local_bottom = local_verts[:num_bottom]
                local_bottom_vertices.append(local_bottom)
            
            # Use target-based forces on all bricks
            # First update current vertex positions from simulation
            current_bottom = update_current_vertices_from_simulation(
                current_bricks,
                local_bottom_vertices
            )
            
            # Setup target-based force parameters
            target_force_params = {
                'stiffness': args.target_stiffness,
                'damping': args.target_damping
            }
            
            # Apply target-based forces to all bricks
            apply_target_based_forces(
                current_bricks, 
                current_bottom,
                event_handler.simulation_data['target_vertices'],
                target_force_params
            )
        # If no target deployment is specified, no additional forces are applied
        # (only physics constraints and gravity will act)
    
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
