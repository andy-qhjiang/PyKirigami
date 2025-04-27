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
        self.tile_labels = {}      # Maps brick_id to its text label ID
        self.ui_controls = {}      # Stores UI control IDs
        self.labels_visible = show_labels
        self.update_frequency = 5  # Only update labels every N frames for performance
        self.frame_counter = 0
        
        # Initialize tile index values
        self.tile1_index = 0
        self.tile2_index = 1
        
        # State tracking for edge detection
        self.last_reset_val = 0
        self.last_save_val = 0
        self.last_constraint_removal_val = 0
        self.last_label_toggle_val = 1 if show_labels else 0
        
        # Initialize constraint mapping if it exists in simulation_data
        if 'constraint_mapping' in simulation_data:
            self.constraint_mapping = simulation_data['constraint_mapping']
        else:
            # For backward compatibility, create an empty mapping
            self.constraint_mapping = []
            # Map constraints
            self._map_constraints_to_bodies()
    
    def setup_ui_controls(self):
        """Set up UI controls for interactive simulation"""
        # Create debug controls
        self.ui_controls['reset_param'] = p.addUserDebugParameter("Reset simulation", 0, 1, 0)
        self.ui_controls['save_param'] = p.addUserDebugParameter("Save vertices", 0, 1, 0)
        
        # Constraint removal interface
        self.ui_controls['tile1_param'] = p.addUserDebugParameter("Tile 1 Index", 0, len(self.simulation_data['bricks'])-1, 0)
        self.ui_controls['tile2_param'] = p.addUserDebugParameter("Tile 2 Index", 0, len(self.simulation_data['bricks'])-1, 1)
        self.ui_controls['remove_constraint_button'] = p.addUserDebugParameter("Remove Constraint", 0, 1, 0)
        
        # Add toggle for tile labels visibility
        self.ui_controls['label_toggle'] = p.addUserDebugParameter("Show/Hide Labels", 0, 1, 1 if self.labels_visible else 0)
        
        # Add performance control slider - update frequency for labels
        self.ui_controls['update_freq'] = p.addUserDebugParameter("Label update frequency", 1, 30, self.update_frequency)
        
        # Add labels to all tiles at startup if enabled
        if self.labels_visible:
            self.add_labels_to_tiles()
        
        return self.ui_controls
        
    def _map_constraints_to_bodies(self):
        """Map constraints to the bodies they connect for simulations not using the constraint mapping"""
        # Get all constraints in the simulation
        num_constraints = p.getNumConstraints()
        
        if num_constraints == 0:
            # No constraints to map
            return
            
        # Store constraint IDs with validation
        for i in range(num_constraints):
            try:
                constraint_info = p.getConstraintInfo(i)
                parent_id = constraint_info[2]
                child_id = constraint_info[3]
                
                # Try to find the tile indices for these IDs
                if parent_id in self.simulation_data['bricks'] and child_id in self.simulation_data['bricks']:
                    parent_idx = self.simulation_data['bricks'].index(parent_id)
                    child_idx = self.simulation_data['bricks'].index(child_id)
                    self.constraint_mapping.append((i, parent_idx, child_idx))
            except Exception as e:
                # Skip invalid constraints
                continue
    
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
        
        # Handle constraint removal between two tiles
        current_remove_val = p.readUserDebugParameter(self.ui_controls['remove_constraint_button'])
        if current_remove_val > 0.5 and self.last_constraint_removal_val <= 0.5:
            tile1_idx = int(p.readUserDebugParameter(self.ui_controls['tile1_param']))
            tile2_idx = int(p.readUserDebugParameter(self.ui_controls['tile2_param']))
            self.remove_constraint_between_tiles(tile1_idx, tile2_idx)
            
            # Reset the button
            p.removeUserDebugItem(self.ui_controls['remove_constraint_button'])
            self.ui_controls['remove_constraint_button'] = p.addUserDebugParameter("Remove Constraint", 0, 1, 0)
        self.last_constraint_removal_val = current_remove_val
        
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
        """Update label positions to match current tile positions without removing any labels"""
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
        
        # Update positions of all existing labels
        for brick_id, label_id in self.tile_labels.items():
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
    
    def _remove_all_labels(self):
        """Remove all tile index labels"""
        for label_id in self.tile_labels.values():
            try:
                p.removeUserDebugItem(label_id)
            except:
                pass
        self.tile_labels = {}
        
    def remove_constraint_between_tiles(self, tile1_idx, tile2_idx):
        """
        Remove any constraints between two tiles specified by their indices
        
        Args:
            tile1_idx: Index of first tile
            tile2_idx: Index of second tile
        
        Returns:
            int: Number of constraints removed
        """
        # Validate indices
        bricks = self.simulation_data['bricks']
        if tile1_idx < 0 or tile1_idx >= len(bricks) or tile2_idx < 0 or tile2_idx >= len(bricks):
            print(f"Invalid tile indices: {tile1_idx}, {tile2_idx}. Valid range: 0-{len(bricks)-1}")
            return 0
        
        # Get the actual brick IDs for the specified indices
        brick1_id = bricks[tile1_idx]
        brick2_id = bricks[tile2_idx]
        
        # Find constraints between these tiles using our mapping
        constraints_removed = 0
        constraints_to_remove = []
        
        # Loop through constraint mapping and identify constraints between the specified tiles
        for i, constraint_data in enumerate(self.constraint_mapping):
            # Safety check for constraint data format
            if not isinstance(constraint_data, (list, tuple)) or len(constraint_data) < 3:
                print(f"Warning: Invalid constraint data format at index {i}: {constraint_data}")
                continue
                
            try:
                constraint_id, t1_idx, t2_idx = constraint_data
                
                # Match by actual indices, not by brick IDs
                if (t1_idx == tile1_idx and t2_idx == tile2_idx) or (t1_idx == tile2_idx and t2_idx == tile1_idx):
                    constraints_to_remove.append((i, constraint_id))
            except (ValueError, TypeError) as e:
                print(f"Error processing constraint data at index {i}: {e}")
                continue
        
        # Remove constraints in reverse order to keep indices valid
        for idx_to_remove, constraint_id in sorted(constraints_to_remove, reverse=True):
            try:
                # Remove the constraint in PyBullet
                p.removeConstraint(constraint_id)
                # Remove from our mapping
                del self.constraint_mapping[idx_to_remove]
                constraints_removed += 1
                print(f"Successfully removed constraint {constraint_id} between tiles {tile1_idx} and {tile2_idx}")
            except Exception as e:
                print(f"Error removing constraint {constraint_id}: {e}")
        
        if constraints_removed > 0:
            print(f"Removed {constraints_removed} constraint(s) between tiles {tile1_idx} and {tile2_idx}")
        else:
            print(f"No constraints found between tiles {tile1_idx} and {tile2_idx}")
            
        return constraints_removed
    
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