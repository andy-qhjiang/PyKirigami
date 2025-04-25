"""
Event handler for interactive kirigami simulation.

This module provides a comprehensive handler for the kirigami simulation interactive controls
through the PyBullet GUI, including:
- Reset simulation
- Save vertex data
- Delete tiles
- Display and update tile labels
"""
import os
import time
import numpy as np
import pybullet as p
from datetime import datetime

class EventHandler:
    def __init__(self, simulation_data, simulation_functions, show_labels=True):
        """
        Initialize the event handler with simulation data.
        
        Args:
            simulation_data: Dictionary containing simulation data
            simulation_functions: Dictionary containing functions for simulation control:
                - initialize_simulation: Function to reinitialize the simulation
                - apply_forces: Function to apply forces to tiles
            show_labels: Whether to show tile index labels (default: True)
        """
        self.simulation_data = simulation_data
        self.simulation_functions = simulation_functions
        self.constraints_map = {}  # Maps body_id to list of constraint IDs
        self.tile_labels = {}      # Maps brick_id to its text label ID
        self.ui_controls = {}      # Stores UI control IDs
        self.labels_visible = show_labels
        self.update_frequency = 5  # Only update labels every N frames for performance
        self.frame_counter = 0
        
        # State tracking for edge detection
        self.last_reset_val = 0
        self.last_save_val = 0
        self.last_delete_button_val = 0
        self.last_label_toggle_val = 1 if show_labels else 0
        
        # Map constraints
        self._map_constraints_to_bodies()
    
    def setup_ui_controls(self):
        """Set up UI controls for interactive simulation"""
        # Create debug controls
        self.ui_controls['reset_param'] = p.addUserDebugParameter("Reset simulation", 0, 1, 0)
        self.ui_controls['save_param'] = p.addUserDebugParameter("Save vertices", 0, 1, 0)
        
        # Simple deletion interface with visual feedback
        p.addUserDebugText("Delete tile by index:", [0, -0.8, 0], textColorRGB=[1, 0.3, 0.3], textSize=1.5)
        self.ui_controls['delete_param'] = p.addUserDebugParameter("Index to delete", 0, len(self.simulation_data['bricks'])-1, 0)
        self.ui_controls['delete_button'] = p.addUserDebugParameter("Execute deletion", 0, 1, 0)
        
        # Add toggle for tile labels visibility
        self.ui_controls['label_toggle'] = p.addUserDebugParameter("Show/Hide Labels", 0, 1, 1 if self.labels_visible else 0)
        
        # Add performance control slider - update frequency for labels
        self.ui_controls['update_freq'] = p.addUserDebugParameter("Label update frequency", 1, 30, self.update_frequency)
        
        # Add labels to all tiles at startup if enabled
        if self.labels_visible:
            self.add_labels_to_tiles()
        
        return self.ui_controls
        
    def _map_constraints_to_bodies(self):
        """Map constraints to the bodies they connect"""
        # Get all constraints in the simulation
        num_constraints = p.getNumConstraints()
        
        if num_constraints == 0:
            # No constraints to map
            return
            
        # Store constraint IDs with validation
        valid_constraints = []
        for i in range(num_constraints):
            try:
                constraint_info = p.getConstraintInfo(i)
                valid_constraints.append((i, constraint_info))
            except Exception as e:
                # Skip invalid constraints
                continue
                
        # Map valid constraints to bodies
        for constraint_id, constraint_info in valid_constraints:
            parent_id = constraint_info[2]
            child_id = constraint_info[3]
            
            if parent_id not in self.constraints_map:
                self.constraints_map[parent_id] = []
            if child_id not in self.constraints_map:
                self.constraints_map[child_id] = []
                
            self.constraints_map[parent_id].append(constraint_id)
            self.constraints_map[child_id].append(constraint_id)
    
    def handle_ui_events(self):
        """
        Handle UI control events and take appropriate actions.
        
        Returns:
            dict: Updated simulation data if reset was triggered, None otherwise
        """
        # Handle reset control
        current_reset = p.readUserDebugParameter(self.ui_controls['reset_param'])
        if current_reset > 0.5 and self.last_reset_val <= 0.5:
            print("Reset simulation requested")
            # Clean up existing labels
            self._remove_all_labels()
            
            # Reset the simulation
            new_sim_data = self.reset_simulation()
            self.last_reset_val = current_reset
            return new_sim_data
        self.last_reset_val = current_reset
        
        # Handle save control
        current_save = p.readUserDebugParameter(self.ui_controls['save_param'])
        if current_save > 0.5 and self.last_save_val <= 0.5:
            print("Save vertices requested")
            self.save_vertex_locations()
        self.last_save_val = current_save
        
        # Handle deletion control
        current_delete_button_val = p.readUserDebugParameter(self.ui_controls['delete_button'])
        if current_delete_button_val > 0.5 and self.last_delete_button_val <= 0.5:
            delete_idx = int(p.readUserDebugParameter(self.ui_controls['delete_param']))
            self.remove_brick_by_index(delete_idx)
            
            # Reset the delete button
            p.removeUserDebugItem(self.ui_controls['delete_button'])
            self.ui_controls['delete_button'] = p.addUserDebugParameter("Execute deletion", 0, 1, 0)
            self.last_delete_button_val = 0
        else:
            self.last_delete_button_val = current_delete_button_val
            
        # Handle label visibility toggle
        current_label_toggle_val = p.readUserDebugParameter(self.ui_controls['label_toggle'])
        if (current_label_toggle_val > 0.5 and self.last_label_toggle_val <= 0.5) or \
           (current_label_toggle_val <= 0.5 and self.last_label_toggle_val > 0.5):
            # Toggle changed state
            self.labels_visible = not self.labels_visible
            if self.labels_visible:
                self.add_labels_to_tiles()
            else:
                self._remove_all_labels()
        self.last_label_toggle_val = current_label_toggle_val
        
        # Handle update frequency adjustment
        new_freq = int(p.readUserDebugParameter(self.ui_controls['update_freq']))
        if new_freq != self.update_frequency:
            self.update_frequency = new_freq
        
        return None
    
    def reset_simulation(self):
        """
        Reset the simulation to its initial state.
        
        Returns:
            dict: Updated simulation data
        """
        # Make sure all labels are removed
        self._remove_all_labels()
        
        # Clean up existing bodies
        for b in self.simulation_data['bricks']:
            try:
                p.removeBody(b)
            except:
                pass
        
        # Give time for physics engine to process removals
        for _ in range(20):
            p.stepSimulation()
            time.sleep(0.01)
        
        # Reinitialize with the original data
        sim_data, force_tiles, force_function = self.simulation_functions['initialize_simulation']()
        self.simulation_data = sim_data
        
        # Stabilize the new scene
        print("Stabilizing reset simulation...")
        for _ in range(50):
            p.stepSimulation()
            time.sleep(0.001)
            
        # Add labels to all tiles after reset if they were visible
        if self.labels_visible:
            self.add_labels_to_tiles()
            
        print("Reset completed")
        return sim_data
        
    def add_labels_to_tiles(self):
        """Add visible index labels to all tiles in the simulation"""
        # Remove any existing labels first
        self._remove_all_labels()
        
        # Create new labels for each brick
        for idx, brick_id in enumerate(self.simulation_data['bricks']):
            # Add text label (position will be updated in update_labels)
            text_id = p.addUserDebugText(
                str(idx),
                [0, 0, 0],  # Initial position will be updated
                textColorRGB=[1, 1, 0],  # Yellow text
                textSize=1.5,
                lifeTime=0  # Persistent until removed
            )
            self.tile_labels[brick_id] = text_id
        
        # Update positions immediately
        self.update_labels()
    
    def update_labels(self):
        """Update label positions to match current tile positions"""
        if not self.tile_labels or not self.labels_visible:
            return
            
        # Check if we should update based on the frequency
        self.frame_counter += 1
        if self.frame_counter % self.update_frequency != 0:
            return
            
        # Get all positions and orientations at once for better performance
        positions_and_orientations = {brick_id: p.getBasePositionAndOrientation(brick_id) 
                                     for brick_id in self.tile_labels.keys() 
                                     if brick_id in self.simulation_data['bricks']}
        
        # Update in batches for better performance
        for brick_id, label_id in list(self.tile_labels.items()):
            if brick_id in self.simulation_data['bricks']:
                idx = self.simulation_data['bricks'].index(brick_id)
                if idx < len(self.simulation_data['normals']):
                    # Get cached position and orientation
                    if brick_id in positions_and_orientations:
                        pos, orn = positions_and_orientations[brick_id]
                    else:
                        continue
                        
                    normal = self.simulation_data['normals'][idx]
                    
                    # Apply current orientation to normal
                    rot = p.getMatrixFromQuaternion(orn)
                    rot_mat = np.array(rot).reshape(3, 3)
                    rotated_normal = rot_mat.dot(np.array(normal))
                    
                    # Position the label slightly above the brick in the normal direction
                    label_pos = [
                        pos[0] + rotated_normal[0] * 0.05,
                        pos[1] + rotated_normal[1] * 0.05,
                        pos[2] + rotated_normal[2] * 0.05
                    ]
                    
                    # Update label position
                    p.addUserDebugText(
                        str(idx),
                        label_pos,
                        textColorRGB=[1, 1, 0],
                        textSize=1.5,
                        lifeTime=0,
                        replaceItemUniqueId=label_id
                    )
            else:
                # If brick no longer exists, remove the label
                p.removeUserDebugItem(label_id)
                del self.tile_labels[brick_id]
    
    def _remove_all_labels(self):
        """Remove all tile index labels"""
        for label_id in self.tile_labels.values():
            try:
                p.removeUserDebugItem(label_id)
            except:
                pass
        self.tile_labels = {}
        
    def remove_brick_by_index(self, index):
        """
        Remove a brick by its index in the bricks list.
        
        Args:
            index: Index of brick to remove (0-based)
            
        Returns:
            bool: True if brick was removed, False otherwise
        """
        # Validate index range
        if index < 0 or index >= len(self.simulation_data['bricks']):
            print(f"Invalid brick index: {index}. Valid range: 0-{len(self.simulation_data['bricks'])-1}")
            return False
            
        # Get the correct brick ID at the specified index
        brick_id = self.simulation_data['bricks'][index]
        print(f"Deleting brick at index {index}")
        
        # First, find and remove all constraints involving this brick
        constraints_to_remove = []
        for c in range(p.getNumConstraints()):
            try:
                constraint_info = p.getConstraintInfo(c)
                if constraint_info[2] == brick_id or constraint_info[3] == brick_id:
                    constraints_to_remove.append(c)
            except Exception as e:
                continue
                
        # Remove constraints (in reverse order to avoid index issues)
        for c in reversed(constraints_to_remove):
            try:
                p.removeConstraint(c)
            except:
                pass
        
        # Remove the brick from the simulation
        try:
            p.removeBody(brick_id)
        except:
            pass
        
        # Update simulation data structures
        self.simulation_data['bricks'].pop(index)
        
        if index < len(self.simulation_data['normals']):
            self.simulation_data['normals'].pop(index)
        
        if 'local_top_vertices' in self.simulation_data and index < len(self.simulation_data['local_top_vertices']):
            self.simulation_data['local_top_vertices'].pop(index)
        
        # Update constraints map
        if brick_id in self.constraints_map:
            del self.constraints_map[brick_id]
        
        # Rebuild constraints map
        self.constraints_map = {}
        self._map_constraints_to_bodies()
        
        # Step a few frames so the GUI refreshes without waiting for the main loop
        for _ in range(10):
            p.stepSimulation()
            time.sleep(0.001)
        
        # Update tile labels to reflect the new indices
        if self.labels_visible:
            self._remove_all_labels()
            self.add_labels_to_tiles()
            
        return True
    
    def save_vertex_locations(self):
        """Save current vertex locations to a file for MATLAB processing"""
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
            
    def step_simulation(self):
        """Run one step of the simulation with forces"""
        # Skip if there are no bricks
        if not self.simulation_data['bricks']:
            p.stepSimulation()
            return
            
        # Calculate the center of the structure
        # Sample only every nth brick for performance with large structures
        num_bricks = len(self.simulation_data['bricks'])
        sampling_rate = max(1, num_bricks // 100)  # Limit to ~100 samples max
        sampled_bricks = self.simulation_data['bricks'][::sampling_rate]
        
        # Get positions efficiently in batch
        sampled_positions = [p.getBasePositionAndOrientation(brick_id)[0] 
                           for brick_id in sampled_bricks]
        whole_center = np.mean(sampled_positions, axis=0)
        
        # Apply forces using the provided function
        # Only apply forces every few steps for large simulations
        if num_bricks < 100 or self.frame_counter % 2 == 0:
            self.simulation_functions['apply_forces'](whole_center)
        
        # Step simulation
        p.stepSimulation()
        
        # Update label positions if visible
        if self.labels_visible:
            self.update_labels()