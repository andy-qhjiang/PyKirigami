"""
Interaction Controller for Kirigami Simulation

This module provides mouse-based interaction functionality for the Kirigami simulation, including:
- Processing mouse events (right-click to fix/unfix bricks)
"""
import pybullet as p
import numpy as np
from utils.physics_utils import fix_object_to_world, unfix_object_from_world, getRayFromTo


BRICK_COLOR = [0.8, 0.35, 0.13, 1.0]      # Default brick color
FIXED_COLOR = [0.53, 0.81, 0.92, 1.0]       # Fixed brick color


class InteractionController:
    """
    Handles mouse-based interactive controls for the simulation, just pin/unpin object for now
   
    """
    
    def __init__(self, simulation_data):
        """Initialize the interactive controls.

        Args:
            simulation_data: Dictionary containing simulation data
        """
        self.simulation_data = simulation_data
        # Track fixed objects
        self.fixed_objects = {}        # {body_id: constraint_id}
        
    
    def toggle_static(self, object_id):
        """Toggle an object between free and fixed states.

    Free brick  -> apply FIXED_COLOR and add fixed constraint.
    Fixed brick -> revert to BRICK_COLOR and remove constraint.
        """
        if object_id in self.fixed_objects:  # currently fixed -> free it
            unfix_object_from_world(self.fixed_objects[object_id])
            del self.fixed_objects[object_id]
            p.changeVisualShape(object_id, -1, rgbaColor=BRICK_COLOR)
            print(f"Brick {object_id} is free.")
        else:  # currently free -> fix it
            constraint_id = fix_object_to_world(object_id)
            self.fixed_objects[object_id] = constraint_id
            p.changeVisualShape(object_id, -1, rgbaColor=FIXED_COLOR)
            print(f"Brick {object_id} is fixed.")
            
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
                
                # Perform ray test
                rayFrom, rayTo = getRayFromTo(mouse_x, mouse_y, cam_info)
                results = p.rayTest(rayFrom, rayTo)

                if results and results[0][0] >= 0:  # If we hit something
                    hit_object_id = results[0][0]
                  
                    # Check if the hit object is a brick
                    if hit_object_id in self.simulation_data['bricks']:
                        # Toggle static state
                        self.toggle_static(hit_object_id)
                        return True
                    else:
                        print(f"Object {hit_object_id} is not a brick in our simulation")
                else:
                    print("No brick is fixed at the clicked position")
        
        return False
    
    def reset(self):
        """Release all fixed bricks and restore base color."""
        fixed_ids = list(self.fixed_objects.keys())
        for object_id in fixed_ids:
            unfix_object_from_world(self.fixed_objects[object_id])
            p.changeVisualShape(object_id, -1, rgbaColor=BRICK_COLOR)
            del self.fixed_objects[object_id]
