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

        # Auto-enable deployment when a target file exists
        has_target = self.simulation_data.get('target_vertices') is not None
        self.deployment_enabled = has_target
        if has_target:
            print("Automatic deployment: ON (target detected, press F to toggle)")

        # ---- Adaptive stiffness / stall recovery ----
        self._stall_counter = 0          # consecutive steps without improvement
        self._stall_threshold = 60       # steps before triggering recovery
        self._best_error = float('inf')  # best error seen in current phase
        self._stiffness_mult = 1.0       # progressive stiffness multiplier
        self._last_reported_mult = 1.0   # suppress duplicate stall messages
    
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
        has_target = self.simulation_data.get('target_vertices') is not None
        self.deployment_enabled = has_target
        self._stall_counter = 0
        self._best_error = float('inf')
        self._stiffness_mult = 1.0
        self._last_reported_mult = 1.0

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
        Toggle pause state of the simulation. 
        """
        if not self.is_paused:
            self.is_paused = True
            print(f"Simulation paused.")
        else:
            self.is_paused = False
            print("Simulation resumed.")

    def toggle_deployment(self):
        """Toggle target-based deployment forces on/off."""
        self.deployment_enabled = not self.deployment_enabled
        if self.deployment_enabled:
            self._stall_counter = 0
            self._best_error = float('inf')
            self._stiffness_mult = 1.0
            self._last_reported_mult = 1.0
        state = "ON" if self.deployment_enabled else "OFF"
        pull_tiles = self.simulation_data.get('pull_tiles')
        if self.deployment_enabled and pull_tiles:
            print(f"Automatic deployment: {state} (pulling {len(pull_tiles)} tiles)")
        else:
            print(f"Automatic deployment: {state}")

    def step_simulation(self):
        """Run one step of the simulation, applying forces if not paused."""
        if self.is_paused:
            return
        
        if not self.simulation_data.get('bricks'):
            p.stepSimulation()
            return
            
        # Apply forces only when deployment is enabled (toggle with 'F' key)
        if self.deployment_enabled:
            pull_tiles = self.simulation_data.get('pull_tiles')
            has_target = self.simulation_data.get('target_vertices') is not None

            # ---- Stall detection & adaptive stiffness (target-based only) ----
            if has_target:
                current_err = self.simulation_functions['compute_error'](pull_tiles)
                if current_err < self._best_error * 0.995:
                    self._best_error = min(self._best_error, current_err)
                    self._stall_counter = max(0, self._stall_counter - 1)
                else:
                    self._stall_counter += 1
                    if self._stall_counter >= self._stall_threshold:
                        prev_mult = self._stiffness_mult
                        self._stiffness_mult = min(self._stiffness_mult * 2.0, 16.0)
                        self._stall_counter = 0
                        if self._stiffness_mult != prev_mult:
                            self._last_reported_mult = self._stiffness_mult
                            print(f"Stall detected (err={current_err:.4f}) → "
                                  f"stiffness ×{self._stiffness_mult:.0f}")

            self.simulation_functions['apply_forces'](
                pull_tiles, self._stiffness_mult
            )
        
        p.stepSimulation()