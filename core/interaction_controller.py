"""
Interaction Controller for Kirigami Simulation

This module provides mouse-based interaction functionality for the Kirigami simulation, including:
- Processing mouse events (right-click to fix/unfix bricks)
- Managing fixed object constraints and visual indicators
- Converting screen coordinates to 3D world ray casting
- Providing real-time user interaction with simulation objects
"""
import pybullet as p
import numpy as np
import math
import os
from datetime import datetime
from utils.physics_utils import fix_object_to_world, unfix_object_from_world

# Optional Pillow import for snapshot PNG saving (handled gracefully if absent)
try:
    from PIL import Image  # noqa: F401
except ImportError:  # Pillow not installed; snapshot will fall back to .npy
    Image = None


BRICK_COLOR = [0.98, 0.8, 0.43, 1.0]      # Default brick color
FIXED_COLOR = [1, 0.47, 0.47, 1.0]       # Fixed brick color



class InteractionController:
    """
    Handles mouse-based interactive controls for the simulation.
   
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

    def snapshot(self, output_dir="../output", width=None, height=None, flip_vertical=False):
        """Capture a snapshot of the current viewport and save as a PNG.

        Args:
            output_dir (str): Directory to save the snapshot (relative to this file or absolute path).
            width (int): Override capture width. If None, uses current window width.
            height (int): Override capture height. If None, uses current window height.
            flip_vertical (bool): Whether to vertically flip image (PyBullet origin at lower-left).

        Returns:
            str | None: Path to saved PNG (or .npy fallback) or None on failure.
        """
        try:
            cam_info = p.getDebugVisualizerCamera()
            win_w, win_h = cam_info[0], cam_info[1]
            view_matrix = cam_info[2]
            proj_matrix = cam_info[3]

            # Use provided resolution or current window size
            w = int(width) if width else int(win_w)
            h = int(height) if height else int(win_h)

            # Capture image
            _, _, rgb_pixels, _, _ = p.getCameraImage(
                width=w,
                height=h,
                viewMatrix=view_matrix,
                projectionMatrix=proj_matrix,
                renderer=p.ER_BULLET_HARDWARE_OPENGL
            )

            # Convert to numpy array (RGBA)
            img = np.reshape(rgb_pixels, (h, w, 4))[:, :, :3]  # Drop alpha
            if flip_vertical:
                img = img[::-1]

            # Prepare output path
            base_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), output_dir)
            os.makedirs(base_dir, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"snapshot_{timestamp}.png"
            out_path = os.path.join(base_dir, filename)

            # Try to save as PNG via Pillow
            saved_path = None
            if Image is not None:
                Image.fromarray(img).save(out_path)
                saved_path = out_path
            else:
                npy_path = out_path.replace('.png', '.npy')
                np.save(npy_path, img)
                saved_path = npy_path
                print("Pillow not installed. Saved raw array as .npy instead (pip install pillow).")

            print(f"Snapshot saved to: {saved_path}")
            return saved_path
        except Exception as e:
            print(f"Snapshot failed: {e}")
            return None
        
