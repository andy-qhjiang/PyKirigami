"""
Interactive Controls for Kirigami Simulation

This module provides mouse-based interaction functionality for the Kirigami simulation,
allowing users to fix or unfix tiles by clicking on them.
"""
import pybullet as p
import numpy as np
import math # Ensure math is imported if used for normalization, though it was commented out

# PyBullet constants for easier reference
COV_ENABLE_GUI = p.COV_ENABLE_GUI
COV_ENABLE_SHADOWS = p.COV_ENABLE_SHADOWS
COV_ENABLE_WIREFRAME = p.COV_ENABLE_WIREFRAME
COV_ENABLE_VR_PICKING = p.COV_ENABLE_VR_PICKING
COV_ENABLE_RGB_BUFFER_PREVIEW = p.COV_ENABLE_RGB_BUFFER_PREVIEW
COV_ENABLE_DEPTH_BUFFER_PREVIEW = p.COV_ENABLE_DEPTH_BUFFER_PREVIEW
COV_ENABLE_SEGMENTATION_MARK_PREVIEW = p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW
COV_ENABLE_RENDERING = p.COV_ENABLE_RENDERING


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
        self.fixed_objects = {}  # {body_id: (original_mass, indicator_id)}
        
        # Debug line ID for ray visualization
        self.ray_debug_line = -1
    
    def update_simulation_data(self, simulation_data):
        """
        Update the simulation data reference.
        
        Args:
            simulation_data: Updated simulation data dictionary
        """
        self.simulation_data = simulation_data

    def reset(self):
        """Resets the state of interactive controls."""
        # Clear fixed objects, removing indicators and constraints
        for object_id in list(self.fixed_objects.keys()): # Iterate over a copy for safe deletion
            details = self.fixed_objects.pop(object_id, {}) # Remove and get details
            if 'constraint_id' in details:
                try:
                    p.removeConstraint(details['constraint_id'])
                except p.error:
                    pass # Already removed or error during removal
            if 'indicator_id' in details:
                try:
                    p.removeBody(details['indicator_id'])
                except p.error: # pybullet.error can be used if specifically imported
                    pass # Already removed or error during removal

        if self.ray_debug_line != -1:
            try:
                p.removeUserDebugItem(self.ray_debug_line)
            except p.error:
                pass # Already removed or error
            self.ray_debug_line = -1
        # print("Interactive controls reset.") # Optional: for debugging
    
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
        
        # Create indicator visual (red sphere)
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.08,
            rgbaColor=[1, 0, 0, 0.8]
        )
        
        # Create indicator body (visual only) - attach it directly to the object
        # Instead of placing at hit position, place at local offset from object center
        # local_offset = [
        #     hit_position[0] - pos[0],
        #     hit_position[1] - pos[1],
        #     hit_position[2] - pos[2]
        # ]
        
        # # Normalize the offset distance to be consistent
        # offset_length = math.sqrt(sum([x*x for x in local_offset]))
        # if offset_length > 0.2:  # If hit point is far enough from center
        #     # Make offset consistent distance
        #     normalized_offset = [x/offset_length * 0.2 for x in local_offset]
        #     indicator_pos = [pos[i] + normalized_offset[i] for i in range(3)]
        # else:
        #     # If hit close to center, place indicator right above the object
        #     indicator_pos = [pos[0], pos[1], pos[2] + 0.2]


        indicator_pos = hit_position
        # Create indicator body
        indicator_id = p.createMultiBody(
            baseMass=0,  # Zero mass so physics don't affect it
            baseVisualShapeIndex=visual_shape,
            basePosition=indicator_pos,
            baseOrientation=[0, 0, 0, 1],
            useMaximalCoordinates=True
        )
        
        # # Create a constraint to attach the indicator to the object
        # # This ensures the indicator moves with the object
        # constraint_id = p.createConstraint(
        #     parentBodyUniqueId=object_id,
        #     parentLinkIndex=-1,
        #     childBodyUniqueId=indicator_id,
        #     childLinkIndex=-1,
        #     jointType=p.JOINT_FIXED,
        #     jointAxis=[0, 0, 0],
        #     parentFramePosition=[
        #         indicator_pos[0] - pos[0],
        #         indicator_pos[1] - pos[1],
        #         indicator_pos[2] - pos[2]
        #     ],
        #     childFramePosition=[0, 0, 0]
        # )
        
        # # Return both the indicator ID and constraint ID
        return indicator_id
    
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
            details = self.fixed_objects[object_id]
            # Remove constraint
            if 'constraint_id' in details:
                try:
                    p.removeConstraint(details['constraint_id'])
                except Exception as e:
                    print(f"Error removing constraint: {e}")
            
            # Remove indicator
            if 'indicator_id' in details:
                try:
                    p.removeBody(details['indicator_id'])
                except Exception as e:
                    print(f"Error removing indicator: {e}")
            
            # Remove from fixed objects dictionary
            del self.fixed_objects[object_id]
            print(f"Object {object_id}: Unfixed and now dynamic.")
            return False
   
        else: # Object is dynamic - make it static
            # Get the current position and orientation before fixing
            pos, orn = p.getBasePositionAndOrientation(object_id)
            
            # Create fixed constraint to world frame
            try:
                constraint_id = p.createConstraint(
                    parentBodyUniqueId=object_id,
                    parentLinkIndex=-1,
                    childBodyUniqueId=-1,  # World frame
                    childLinkIndex=-1,
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=[0, 0, 0],  # Relative to object's CoM
                    childFramePosition=pos,         # Target position in world
                    parentFrameOrientation=[0,0,0,1], # Relative orientation
                    childFrameOrientation=orn         # Target orientation in world
                )
                
                # Ensure very high max force for stability
                p.changeConstraint(constraint_id, maxForce=1e10) # Significantly increased maxForce
                
                # Create visual indicator
                indicator_id = self.create_static_indicator(object_id, hit_position)
                
                # Store in dictionary
                self.fixed_objects[object_id] = {
                    'constraint_id': constraint_id,
                    'indicator_id': indicator_id
                }
                
                print(f"Object {object_id}: Fixed in place with JOINT_FIXED constraint (maxForce=1e10).")
                return True
                
            except Exception as e:
                print(f"Error creating constraint: {e}")
                return False
            
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
                cam_pos = cam_info[4]
                cam_forward = cam_info[5]
                cam_right = cam_info[6]
                cam_up = cam_info[7]
                
                # Print debug info for camera
                print(f"Mouse position: ({mouse_x}, {mouse_y})")
                print(f"Camera position: {cam_pos}")
                
                # Extract camera world position from view matrix
                # The camera position is at (0,0,0) in camera space, but we need world space
                rayFrom = [
                    -view_matrix[0] * view_matrix[12] - view_matrix[1] * view_matrix[13] - view_matrix[2] * view_matrix[14],
                    -view_matrix[4] * view_matrix[12] - view_matrix[5] * view_matrix[13] - view_matrix[6] * view_matrix[14],
                    -view_matrix[8] * view_matrix[12] - view_matrix[9] * view_matrix[13] - view_matrix[10] * view_matrix[14]
                ]
                
                print(f"Ray origin (from view matrix): {rayFrom}")

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
                
                # Visualize ray (optional)
                if self.ray_debug_line >= 0:
                    p.removeUserDebugItem(self.ray_debug_line)
                self.ray_debug_line = p.addUserDebugLine(rayFrom, ray_end, [1, 0, 0], 2, 0.1)
                
                # Add coordinate system visualization at ray origin
                axis_length = 0.3
                p.addUserDebugLine(rayFrom, 
                                 [rayFrom[i] + cam_right_normalized[i] * axis_length for i in range(3)],
                                 lineColorRGB=[1, 0, 0], lineWidth=2, lifeTime=30.5)  # X-axis (red)
                p.addUserDebugLine(rayFrom, 
                                 [rayFrom[i] + cam_up_normalized[i] * axis_length for i in range(3)],
                                 lineColorRGB=[0, 1, 0], lineWidth=2, lifeTime=30.5)  # Y-axis (green)
                p.addUserDebugLine(rayFrom, 
                                 [rayFrom[i] + cam_forward_normalized[i] * axis_length for i in range(3)],
                                 lineColorRGB=[0, 0, 1], lineWidth=2, lifeTime=30.5)  # Z-axis (blue)
                
                # Perform ray test
                results = p.rayTest(rayFrom, ray_end)
                
                if results and results[0][0] >= 0:  # If we hit something
                    hit_object_id = results[0][0]
                    hit_position = results[0][3]
                    hit_normal = results[0][4]  # Normal of the hit surface
                    
                    print(f"Hit object {hit_object_id} at position {hit_position}, normal: {hit_normal}")
                    
                    # Draw a line from ray origin to hit point
                    p.addUserDebugLine(rayFrom, hit_position, lineColorRGB=[1, 1, 0], lineWidth=2, lifeTime=30.5)
                    
                    # Check if the hit object is a brick
                    if hit_object_id in self.simulation_data['bricks']:
                        # Toggle static state
                        self.toggle_static(hit_object_id, hit_position)
                        return True
                    else:
                        print(f"Object {hit_object_id} is not a brick in our simulation")
                else:
                    print("No object hit by ray")
        
        return False
    
    def reset(self):
        """
        Reset all fixed objects to dynamic state.
        """
        # Copy the dictionary keys to avoid modifying during iteration
        fixed_ids = list(self.fixed_objects.keys())
        
        for object_id in fixed_ids:
            original_mass, indicator_data = self.fixed_objects[object_id]
            indicator_id, constraint_id = indicator_data
            
            # Restore original mass
            try:
                p.changeDynamics(object_id, -1, mass=original_mass)
            except:
                pass  # Object might have been removed
            
            # Remove constraint first
            try:
                p.removeConstraint(constraint_id)
            except:
                pass  # Constraint might have been removed
            
            # Remove indicator
            try:
                p.removeBody(indicator_id)
            except:
                pass
            
            # Remove from dictionary
            del self.fixed_objects[object_id]
        
        # Clear ray visualization
        if self.ray_debug_line >= 0:
            p.removeUserDebugItem(self.ray_debug_line)
            self.ray_debug_line = -1
