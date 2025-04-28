"""
Event handler for interactive kirigami simulation.

This module provides a comprehensive handler for the kirigami simulation interactive controls,
with responsibilities split into focused components:
- EventHandler: Main coordinator and entry point
- LabelManager: Handles the tile labels display and updates
- ConstraintManager: Manages the physics constraints between tiles
- SimulationController: Handles high-level simulation operations (reset, save)
"""
import os
import time
import numpy as np
import pybullet as p
from datetime import datetime

class LabelManager:
    """Handles the creation, updating, and removal of tile labels in the simulation."""
    
    def __init__(self, simulation_data, visible=True, update_frequency=5):
        """
        Initialize the label manager.
        
        Args:
            simulation_data: Dictionary containing simulation data
            visible: Whether labels should be visible initially
            update_frequency: How often to update label positions (every N frames)
        """
        self.simulation_data = simulation_data
        self.labels_visible = visible
        self.tile_labels = {}  # Maps brick_id to its text label ID
        self.update_frequency = update_frequency
        self.frame_counter = 0
        
        # Create labels initially if they should be visible
        if visible:
            self.add_labels_to_tiles()
    
    def add_labels_to_tiles(self):
        """Add visible index labels to all tiles in the simulation"""
        # Remove any existing labels first
        self.remove_all_labels()
        
        # Create new labels for each brick
        for idx, brick_id in enumerate(self.simulation_data['bricks']):
            # Add text label (position will be updated later)
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
    
    def remove_all_labels(self):
        """Remove all tile index labels"""
        for label_id in self.tile_labels.values():
            try:
                p.removeUserDebugItem(label_id)
            except:
                pass
        self.tile_labels = {}
        
    def toggle_visibility(self):
        """Toggle whether labels are visible"""
        self.labels_visible = not self.labels_visible
        if self.labels_visible:
            self.add_labels_to_tiles()
        else:
            self.remove_all_labels()
    
    def set_update_frequency(self, frequency):
        """Set how often labels are updated"""
        self.update_frequency = max(1, int(frequency))


class ConstraintManager:
    """Handles operations related to constraints between tiles."""
    
    def __init__(self, simulation_data):
        """
        Initialize the constraint manager.
        
        Args:
            simulation_data: Dictionary containing simulation data
        """
        self.simulation_data = simulation_data
        self.constraint_mapping = simulation_data.get('constraint_mapping', [])
        
        # If no constraint mapping was provided, create one
        if not self.constraint_mapping:
            self._map_constraints_to_bodies()
    
    def _map_constraints_to_bodies(self):
        """Map constraints to the bodies they connect"""
        # Get all constraints in the simulation
        num_constraints = p.getNumConstraints()
        
        if num_constraints == 0:
            return
            
        # Store constraint IDs with validation
        for i in range(num_constraints):
            try:
                constraint_info = p.getConstraintInfo(i)
                parent_id = constraint_info[2]
                child_id = constraint_info[3]
                
                # Find the tile indices for these IDs
                if parent_id in self.simulation_data['bricks'] and child_id in self.simulation_data['bricks']:
                    parent_idx = self.simulation_data['bricks'].index(parent_id)
                    child_idx = self.simulation_data['bricks'].index(child_id)
                    self.constraint_mapping.append((i, parent_idx, child_idx))
            except Exception as e:
                continue
    
    def remove_constraint_between_tiles(self, tile1_idx, tile2_idx):
        """
        Remove constraints between two tiles specified by their indices.
        
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
        
        # Find constraints between these tiles
        constraints_removed = 0
        constraints_to_remove = []
        
        # Identify constraints between the specified tiles
        for i, constraint_data in enumerate(self.constraint_mapping):
            if not isinstance(constraint_data, (list, tuple)) or len(constraint_data) < 3:
                print(f"Warning: Invalid constraint data format at index {i}: {constraint_data}")
                continue
                
            try:
                constraint_id, t1_idx, t2_idx = constraint_data
                
                # Check if this constraint connects the specified tiles
                if (t1_idx == tile1_idx and t2_idx == tile2_idx) or (t1_idx == tile2_idx and t2_idx == tile1_idx):
                    constraints_to_remove.append((i, constraint_id))
            except (ValueError, TypeError) as e:
                print(f"Error processing constraint data at index {i}: {e}")
                continue
        
        # Remove constraints in reverse order to keep indices valid
        for idx_to_remove, constraint_id in sorted(constraints_to_remove, reverse=True):
            try:
                p.removeConstraint(constraint_id)
                del self.constraint_mapping[idx_to_remove]
                constraints_removed += 1
                print(f"Removed constraint {constraint_id} between tiles {tile1_idx} and {tile2_idx}")
            except Exception as e:
                print(f"Error removing constraint {constraint_id}: {e}")
        
        if constraints_removed > 0:
            print(f"Removed {constraints_removed} constraint(s) between tiles {tile1_idx} and {tile2_idx}")
        else:
            print(f"No constraints found between tiles {tile1_idx} and {tile2_idx}")
            
        return constraints_removed


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


class EventHandler:
    """Main event handler that coordinates between the specialized components."""
    
    def __init__(self, simulation_data, simulation_functions, show_labels=True):
        """
        Initialize the event handler with simulation data.
        
        Args:
            simulation_data: Dictionary containing simulation data
            simulation_functions: Dictionary with functions for simulation control
            show_labels: Whether to show tile index labels (default: True)
        """
        self.simulation_data = simulation_data
        self.simulation_functions = simulation_functions
        
        # Create specialized components
        self.label_manager = LabelManager(simulation_data, visible=show_labels)
        self.constraint_manager = ConstraintManager(simulation_data)
        self.simulation_controller = SimulationController(simulation_data, simulation_functions)
        
        # UI control state tracking
        self.ui_controls = {}
        self.tile1_index = 0
        self.tile2_index = 1
        
        # State tracking for edge detection on UI controls
        self.last_reset_val = 0
        self.last_save_val = 0
        self.last_constraint_removal_val = 0
        self.last_label_toggle_val = 1 if show_labels else 0
        
    def setup_ui_controls(self):
        """Set up minimal PyBullet UI controls needed for Qt integration"""
        # We only need the controls that Qt will interact with
        # Reset and save controls aren't needed in PyBullet since Qt will call our methods directly
        
        # Add control for label update frequency
        self.ui_controls['update_freq'] = p.addUserDebugParameter("Label update frequency", 1, 30, self.label_manager.update_frequency)
        
        return self.ui_controls
    
    @property
    def labels_visible(self):
        """Get the current label visibility state"""
        return self.label_manager.labels_visible
    
    @labels_visible.setter
    def labels_visible(self, value):
        """Set label visibility and update as needed"""
        if value != self.label_manager.labels_visible:
            self.label_manager.toggle_visibility()
    
    def add_labels_to_tiles(self):
        """Add labels to tiles (proxy to label manager)"""
        self.label_manager.add_labels_to_tiles()
    
    def _remove_all_labels(self):
        """Remove all labels (proxy to label manager)"""
        self.label_manager.remove_all_labels()
    
    def reset_simulation(self):
        """Reset the simulation"""
        # Remove labels before reset
        self.label_manager.remove_all_labels()
        
        # Reset simulation
        new_sim_data = self.simulation_controller.reset_simulation()
        
        # Update data in all components
        self.simulation_data = new_sim_data
        self.label_manager.simulation_data = new_sim_data
        self.constraint_manager.simulation_data = new_sim_data
        self.simulation_controller.simulation_data = new_sim_data
        
        # Add labels again if they were visible
        if self.label_manager.labels_visible:
            self.label_manager.add_labels_to_tiles()
            
        return new_sim_data
    
    def save_vertex_locations(self):
        """Save current vertex locations to a file"""
        return self.simulation_controller.save_vertex_locations()
    
    def remove_constraint_between_tiles(self, tile1_idx, tile2_idx):
        """Remove constraints between specified tiles"""
        return self.constraint_manager.remove_constraint_between_tiles(tile1_idx, tile2_idx)
    
    def handle_ui_events(self):
        """
        Handle PyBullet UI control events.
        
        Returns:
            dict: Updated simulation data if reset was triggered, None otherwise
        """
        # We only need to handle essential parameters that aren't directly controlled by Qt
        
        # Check if the update frequency has changed
        if 'update_freq' in self.ui_controls:
            new_freq = int(p.readUserDebugParameter(self.ui_controls['update_freq']))
            if new_freq != self.label_manager.update_frequency:
                self.label_manager.set_update_frequency(new_freq)
        
        # No data to return since reset will be handled through Qt
        return None
    
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
        if num_bricks < 100 or self.label_manager.frame_counter % 2 == 0:
            self.simulation_functions['apply_forces'](whole_center)
        
        # Step simulation
        p.stepSimulation()
        
        # Update label positions if visible
        if self.label_manager.labels_visible:
            self.label_manager.update_labels()