"""
Event handler for interactive kirigami simulation.

This module provides a comprehensive handler for the kirigami simulation interactive controls,
with responsibilities split into focused components:
- EventHandler: Main coordinator and entry point
- SimulationController: Handles high-level simulation operations (reset, save)
"""
import os
import numpy as np
import pybullet as p
from datetime import datetime

class SimulationController:
    """Handles high-level simulation operations like reset and saving."""
    
    def __init__(self, simulation_data, simulation_functions):
        """
        Initialize the simulation controller.
        
        Args:
            simulation_data: Dictionary containing simulation data
            simulation_functions: Dictionary with functions for simulation control
        """
        self.simulation_data = simulation_data
        self.simulation_functions = simulation_functions
        self.is_paused = False  # Track pause state
    
    def reset_simulation(self):
        """
        Reset the simulation to its initial state.
        
        Returns:
            dict: Updated simulation data
        """
        # Clean up existing bodies
        for b in self.simulation_data['bricks']:
            try:
                p.removeBody(b)
            except:
                pass
        
        # # Give time for physics engine to process removals
        # for _ in range(20):
        #     p.stepSimulation()
        #     time.sleep(0.01)
        
        # Reinitialize with the original data
        sim_data = self.simulation_functions['initialize_simulation']()
        self.simulation_data = sim_data
      
        print("Reset completed")
        return sim_data
    
    def save_vertex_locations(self):
        """
        Save current vertex locations to a file.
        
        Returns:
            bool: Success or failure
        """
        try:
            # Create a filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../output')
            
            # Create output directory if it doesn't exist
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            output_file = os.path.join(output_dir, f"vertices_{timestamp}.txt")
            
            # Compute world positions of top face vertices for each brick
            brick_data = []
            for idx, brick_id in enumerate(self.simulation_data['bricks']):
                position, orientation = p.getBasePositionAndOrientation(brick_id)
                # Get local top-vertex offsets stored at initialization
                offsets = self.simulation_data['local_top_vertices'][idx]
                # Rotation matrix from body orientation
                rot = p.getMatrixFromQuaternion(orientation)
                rot_mat = np.array(rot).reshape(3, 3)
                world_vertices = []
                for offset in offsets:
                    world_v = (rot_mat.dot(np.array(offset)) + np.array(position)).tolist()
                    world_vertices.append(world_v)
                brick_data.append((idx, world_vertices))
            
            with open(output_file, 'w') as f:
                # Write 12 values per tile: top-face world coordinates
                for idx, verts in brick_data:
                    # flatten vertices [ [x,y,z], ... ] into a single line
                    line = ' '.join(str(coord) for vert in verts for coord in vert)
                    f.write(line + "\n")
            
            print(f"Saved vertex data to {output_file}")
            return True
        except Exception as e:
            print(f"Error saving vertex data: {e}")
            return False
    
    def toggle_pause(self):
        """
        Toggle pause state of the simulation.
        When paused, all objects are frozen in place.
        
        Returns:
            bool: Current pause state (True if paused, False if running)
        """
        try:
            if not self.is_paused:
                # Pause the simulation
                print("Pausing simulation...")
                
                # Save current velocities and set them to zero
                for brick_id in self.simulation_data['bricks']:
                    try:
                        
                        # Stop the brick by setting velocity to zero
                        p.resetBaseVelocity(brick_id, [0, 0, 0], [0, 0, 0])
                        
                        # Temporarily increase damping to maximum to prevent movement
                        p.changeDynamics(brick_id, -1, 
                                       linearDamping=100.0, 
                                       angularDamping=100.0)
                    except Exception as e:
                        print(f"Warning: Could not pause brick {brick_id}: {e}")
                
                self.is_paused = True
                print("Simulation paused. Press 'P' again to resume.")
                
            else:
                # Resume the simulation
                print("Resuming simulation...")
                
                # Restore original damping and velocities
                args = self.simulation_data.get('args')
                linear_damping = args.linear_damping if args else 2.5
                angular_damping = args.angular_damping if args else 2.5
                
                for brick_id in self.simulation_data['bricks']:
                    try:
                        # Restore original damping
                        p.changeDynamics(brick_id, -1, 
                                       linearDamping=linear_damping, 
                                       angularDamping=angular_damping)
                        
                    except Exception as e:
                        print(f"Warning: Could not resume brick {brick_id}: {e}")
                
                
                self.is_paused = False
                print("Simulation resumed.")
            
            return self.is_paused
            
        except Exception as e:
            print(f"Error toggling pause: {e}")
            return self.is_paused
        

class EventHandler:
    """Main event handler that coordinates between the specialized components."""
    
    def __init__(self, simulation_data, simulation_functions):
        """
        Initialize the event handler with simulation data.
        
        Args:
            simulation_data: Dictionary containing simulation data
            simulation_functions: Dictionary with functions for simulation control
        """
        self.simulation_data = simulation_data
        self.simulation_functions = simulation_functions
        
        # Create specialized components
        self.simulation_controller = SimulationController(simulation_data, simulation_functions)
        
    
    def reset_simulation(self):
        """Reset the simulation"""
        # Reset interactive controls first - MOVED to run_sim.py
        # self.interactive_controls.reset() # REMOVED
        
        # Reset simulation
        new_sim_data = self.simulation_controller.reset_simulation()
        
        # Update data in all components
        self.simulation_data = new_sim_data
        self.simulation_controller.simulation_data = new_sim_data
        # self.interactive_controls.update_simulation_data(new_sim_data) # REMOVED: Handled in run_sim.py
            
        return new_sim_data
    
    def save_vertex_locations(self):
        """Save current vertex locations to a file"""
        return self.simulation_controller.save_vertex_locations()
    
    def toggle_pause(self):
        """Toggle pause state of the simulation"""
        return self.simulation_controller.toggle_pause()
      # remove_constraint_between_tiles method removed
    
    # handle_ui_events method removed
    
    def step_simulation(self):
        """Run one step of the simulation with forces"""
        # Check if simulation is paused
        if self.simulation_controller.is_paused:
            # Still step physics to maintain GUI responsiveness, but don't apply forces
            p.stepSimulation()
            return
        
        # Process any mouse events for interactive controls - MOVED to run_sim.py
        # self.interactive_controls.process_mouse_events() # REMOVED
        
        # Skip if there are no bricks
        if not self.simulation_data['bricks']:
            p.stepSimulation()
            return
            
        # Calculate the center of the structure
        # Sample only every nth brick for performance with large structures
        num_bricks = len(self.simulation_data['bricks'])
        sampling_rate = max(1, num_bricks // 1000)  # Limit to ~1000 samples max
        
        # Ensure sampled_bricks is not empty if bricks exist
        if num_bricks > 0 and sampling_rate > 0 :
            sampled_bricks = self.simulation_data['bricks'][::sampling_rate]
            if not sampled_bricks: # if sampling rate made it empty, take at least one
                 sampled_bricks = [self.simulation_data['bricks'][0]]
        else: # No bricks or invalid sampling rate
            p.stepSimulation()
            return

        # Get positions efficiently in batch
        sampled_positions = [p.getBasePositionAndOrientation(brick_id)[0] 
                           for brick_id in sampled_bricks]
        
        if not sampled_positions: # If somehow still empty
            p.stepSimulation()
            return

        whole_center = np.mean(sampled_positions, axis=0)
        
        # Apply forces only if auto_expand is enabled
        args = self.simulation_data.get('args')
        if args and args.auto_expand:
            self.simulation_functions['apply_forces'](whole_center)
        
        # Step simulation
        p.stepSimulation()