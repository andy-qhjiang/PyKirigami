"""
Simulation Controller for Interactive Kirigami Simulation

This module provides runtime control and progression for the kirigami simulation, including:
- Processing keyboard events (reset, save, pause/resume, quit)
- Applying target-based forces during simulation
- Managing simulation state transitions
- Coordinating physics stepping with PyBullet
"""
import os
import numpy as np
import pybullet as p
import time
from datetime import datetime
from utils.physics_utils import fix_multiple_objects_to_world, unfix_multiple_objects_from_world
from utils.geometry import transform_local_to_world_coordinates

class SimulationController:
    """
    Handles runtime control and progression of the kirigami simulation.
    
    """
    
    def __init__(self, simulation_data, simulation_functions):
        """
        Initialize the simulation controller.
        
        Args:
            simulation_data: Dictionary containing simulation data.
            simulation_functions: Dictionary with functions for simulation control.
        """
        self.simulation_data = simulation_data
        self.simulation_functions = simulation_functions
        self.is_paused = False
        self.pause_constraints = {}
    
    def reset_simulation(self):
        """
        Reset the simulation to its initial state.
        This method updates the internal simulation_data directly.
        """
        # Clean up existing bodies
        if 'bricks' in self.simulation_data:
            for b in self.simulation_data['bricks']:
                try:
                    p.removeBody(b)
                except p.error:
                    pass
        
        # Give time for physics engine to process removals
        for _ in range(20):
            p.stepSimulation()
            time.sleep(0.01)
        
        # Reinitialize with the original data and update internal state
        self.simulation_data = self.simulation_functions['initialize_simulation']()
      
        # Reset pause state
        self.is_paused = False
        self.pause_constraints = {}

        print("Reset completed")
    
    def save_vertex_locations(self):
        """Save current vertex locations to a file."""
        try:
            # Create a filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../output')
            
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            output_file = os.path.join(output_dir, f"vertices_{timestamp}.txt")
      
            # Get current bottom vertices in world coordinates
            bottom_world_coords = transform_local_to_world_coordinates(
                self.simulation_data['bricks'], self.simulation_data['local_coords']
            )
            
            with open(output_file, 'w') as f:
                for verts in bottom_world_coords:
                    line = ' '.join(map(str, np.array(verts).flatten()))
                    f.write(line + "\n")
            
            print(f"Saved vertex data to {output_file}")
            return True
        except Exception as e:
            print(f"Error saving vertex data: {e}")
            return False
    
    def toggle_pause(self):
        """
        Toggle pause state of the simulation. When paused, all objects are frozen.
        """
        if not self.is_paused:
            print("Pausing simulation...")
            self.pause_constraints = fix_multiple_objects_to_world(self.simulation_data['bricks'])
            self.is_paused = True
            print(f"Simulation paused. Fixed {len(self.pause_constraints)} objects.")
        else:
            print("Resuming simulation...")
            unfix_multiple_objects_from_world(self.pause_constraints)
            self.pause_constraints = {}
            self.is_paused = False
            print("Simulation resumed.")
            
        return self.is_paused

    def step_simulation(self):
        """Run one step of the simulation, applying forces if not paused."""
        if self.is_paused:
            p.stepSimulation()  # Still step to keep GUI responsive
            return
        
        if not self.simulation_data.get('bricks'):
            p.stepSimulation()
            return
            
        # Apply forces (selection handled within Simulation.apply_forces)
        self.simulation_functions['apply_forces']()
        
        p.stepSimulation()