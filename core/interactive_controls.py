"""
Interactive Controls for Kirigami Simulation

This module provides mouse-based interaction functionality for the Kirigami simulation,
allowing users to fix or unfix bricks by clicking on them.
"""
import pybullet as p
import numpy as np
import math
from utils.physics_utils import (
    fix_object_to_world, unfix_object_from_world,
    create_visual_indicator, remove_visual_indicator
)



class InteractiveControls:
    """Manages interactive mouse-based controls for the simulation."""
    
    def __init__(self, simulation_data):
        """
        Initialize the interactive controls.
        
        Args:
            simulation_data: Dictionary containing simulation data
        """
        self.simulation_data = simulation_data
        
        # Dictionary to track fixed objects
        self.fixed_objects = {}  # {body_id: (constraint_id, indicator_id)}

    def update_simulation_data(self, simulation_data):
        """
        Update the simulation data reference after resetting or reloading.
        
        Args:
            simulation_data: Updated simulation data dictionary
        """
        self.simulation_data = simulation_data

    
    def create_static_indicator(self, object_id, hit_position=None):
        """
        Create a visual indicator for fixed/static objects.
        
        Args:
            object_id: ID of the object being fixed
            hit_position: Position where the object was hit (or None for center)
            
        Returns:
            int: ID of the created indicator
        """
        # Get object position
        pos, _ = p.getBasePositionAndOrientation(object_id)
        
        # If no hit position provided, use object center
        if hit_position is None:
            hit_position = pos
        
        # Use the imported function to create indicator
        return create_visual_indicator(hit_position)
    
    def toggle_static(self, object_id, hit_position=None):
        """
        Toggle an object between static and dynamic states using a JOINT_FIXED constraint.
        
        Args:
            object_id: ID of the object to toggle
            hit_position: Position where the object was hit (or None)
            
        Returns:
            bool: True if object is now static, False if dynamic
        """
        if object_id in self.fixed_objects:
            # Object is static - make it dynamic again
            unfix_object_from_world(self.fixed_objects[object_id]['constraint_id'])
            remove_visual_indicator(self.fixed_objects[object_id]['indicator_id'])
                
            # Remove from fixed objects dictionary
            del self.fixed_objects[object_id]
            print(f"Brick {object_id} is free.")
   
        else: # Object is dynamic - make it static
            # Use utility function to fix object to world
            constraint_id = fix_object_to_world(object_id)
            
            # Create visual indicator
            indicator_id = self.create_static_indicator(object_id, hit_position)
            
            # Store in dictionary
            self.fixed_objects[object_id] = {
                'constraint_id': constraint_id,
                'indicator_id': indicator_id
            }
            
            print(f"Brick {object_id} is fixed.")
            
        return object_id in self.fixed_objects
            
    def process_mouse_events(self):
        """
        Process mouse events for interactive control.
        
        Returns:
            bool: True if any relevant mouse event was processed
        """
        # Get all mouse events
        mouse_events = p.getMouseEvents()
        
        if not mouse_events:
            return False
        
        for event in mouse_events:
            # Check for right-click events
            if (event[0] == 2  # MOUSE_BUTTON_EVENT
                    and event[3] == 2  # Right button
                    and (event[4] & p.KEY_WAS_TRIGGERED)):  # Button was triggered
                
                # Get mouse coordinates
                mouse_x, mouse_y = event[1], event[2]
                
                # Get camera information
                cam_info = p.getDebugVisualizerCamera()
                width, height = cam_info[0], cam_info[1]
                view_matrix = cam_info[2]
                proj_matrix = cam_info[3]
                cam_forward = cam_info[5]
                cam_right = cam_info[6]
                cam_up = cam_info[7]
                
                
                # Extract camera world position from view matrix
                # The camera position is at (0,0,0) in camera space, but we need world space
                rayFrom = [
                    -view_matrix[0] * view_matrix[12] - view_matrix[1] * view_matrix[13] - view_matrix[2] * view_matrix[14],
                    -view_matrix[4] * view_matrix[12] - view_matrix[5] * view_matrix[13] - view_matrix[6] * view_matrix[14],
                    -view_matrix[8] * view_matrix[12] - view_matrix[9] * view_matrix[13] - view_matrix[10] * view_matrix[14]
                ]
                
                # Convert mouse coordinates to normalized device coordinates (-1 to 1)
                ndc_x = (2.0 * mouse_x / width - 1.0)
                ndc_y = -(2.0 * mouse_y / height - 1.0)  # Y is flipped in screen space
                
                # Get field of view from projection matrix
                fov_y = 2.0 * math.atan(1.0 / proj_matrix[5])
                aspect = width / height
                
                # Normalize vectors for proper ray direction calculation
                cam_forward_normalized = np.array(cam_forward) / np.linalg.norm(np.array(cam_forward))
                cam_right_normalized = np.array(cam_right) / np.linalg.norm(np.array(cam_right))
                cam_up_normalized = np.array(cam_up) / np.linalg.norm(np.array(cam_up))
                
                # Scale factor based on field of view - ensures ray passes through clicked pixel
                scale_factor = math.tan(fov_y / 2.0)
                
                # Calculate ray direction using the camera basis vectors
                ray_dir = [
                    cam_forward_normalized[i] + 
                    (cam_right_normalized[i] * ndc_x * scale_factor * aspect) + 
                    (cam_up_normalized[i] * ndc_y * scale_factor)
                    for i in range(3)
                ]
                
                # Normalize the final direction
                ray_length = math.sqrt(sum([c*c for c in ray_dir]))
                ray_dir = [c / ray_length for c in ray_dir]
                
                # Set maximum distance for ray
                max_distance = 1000.0  # Use a longer ray for better reach
                ray_end = [rayFrom[i] + ray_dir[i] * max_distance for i in range(3)]
                
                # Perform ray test
                results = p.rayTest(rayFrom, ray_end)
                
                if results and results[0][0] >= 0:  # If we hit something
                    hit_object_id = results[0][0]
                    hit_position = results[0][3]
                  
                    # Check if the hit object is a brick
                    if hit_object_id in self.simulation_data['bricks']:
                        # Toggle static state
                        self.toggle_static(hit_object_id, hit_position)
                        return True
                    else:
                        print(f"Object {hit_object_id} is not a brick in our simulation")
                else:
                    print("No brick is fixed at the clicked position")
        
        return False
    
    def reset(self):
        """
        Reset all fixed objects to dynamic state and remove indicators.
        """
        fixed_ids = list(self.fixed_objects.keys())
        
        for object_id in fixed_ids:
            # Remove constraint and indicator using utility functions
            unfix_object_from_world(self.fixed_objects[object_id]['constraint_id'])
            remove_visual_indicator(self.fixed_objects[object_id]['indicator_id'])
            
            # Remove from dictionary
            del self.fixed_objects[object_id]
        
