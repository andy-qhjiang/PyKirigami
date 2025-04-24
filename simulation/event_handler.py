"""
Event handler for interactive kirigami simulation.

This module provides functions to handle keyboard and mouse events in the PyBullet simulation:
- 'r' key: Reset the simulation to its initial state
- 'v' key: Save current vertex positions to a file for MATLAB processing
- Mouse click with 'd' key: Delete a specific tile/brick from the simulation
"""
import os
import time
import numpy as np
import pybullet as p
from datetime import datetime

class EventHandler:
    def __init__(self, simulation_data):
        """
        Initialize the event handler with simulation data.
        
        Args:
            simulation_data: Dictionary containing simulation data:
                - vertices_file: Path to original vertices file
                - constraints_file: Path to constraints file
                - hull_file: Path to hull file (optional)
                - args: Command-line arguments
                - vertices: Original vertex data
                - constraints: Constraint data
                - force_tiles: Tile indices to apply forces to
                - force_function: Function to calculate force direction
                - bricks: List of brick body IDs
                - normals: List of normal vectors for each brick
                - original_data: Any other data needed for reset
        """
        self.simulation_data = simulation_data
        self.selected_brick_id = None
        self.delete_mode = False
        self.constraints_map = {}  # Maps body_id to list of constraint IDs
        self._map_constraints_to_bodies()
        
        # Register event handlers
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
        p.setRealTimeSimulation(1)
        
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
    
    def handle_events(self):
        """
        Handle keyboard and mouse events.
        
        Returns:
            bool: True if simulation should be reset, False otherwise
        """
        keys = p.getKeyboardEvents()
        mouse_events = p.getMouseEvents()
        
        # Handle keyboard events
        for key, state in keys.items():
            # Key pressed (state == 3) or Key held down (state == 1)
            if state == 3:  
                if key == ord('r'):
                    print("Resetting simulation...")
                    return True  # Signal to reset the simulation
                
                elif key == ord('v'):
                    self._save_vertex_locations()
                
                elif key == ord('d'):
                    self.delete_mode = not self.delete_mode
                    mode_text = "enabled" if self.delete_mode else "disabled"
                    print(f"Delete mode {mode_text}")
        
        # Handle mouse events
        for event in mouse_events:
            # Mouse button clicked
            if event[0] == 2:  # Mouse button down event
                # Check if we're in delete mode
                if self.delete_mode:
                    self._handle_brick_deletion(event)
            
        return False
    
    def _handle_brick_deletion(self, mouse_event):
        """Handle deletion of a brick when clicked in delete mode"""
        # Get the ray from mouse click
        x, y = mouse_event[1], mouse_event[2]
        ray_from, ray_to = self._get_ray_from_mouse(x, y)
        
        # Perform ray test
        results = p.rayTest(ray_from, ray_to)
        
        if results[0][0] != -1:  # If the ray hit something
            hit_object_id = results[0][0]
            
            # Check if the hit object is a brick
            if hit_object_id in self.simulation_data['bricks']:
                print(f"Deleting brick ID: {hit_object_id}")
                self._remove_brick(hit_object_id)
    
    def _get_ray_from_mouse(self, x, y):
        """Convert mouse coordinates to a ray in 3D space"""
        # Get camera information - this returns (width, height, viewMatrix, projectionMatrix, cameraUp, cameraForward, ...)
        cam_info = p.getDebugVisualizerCamera()
        width, height = cam_info[0], cam_info[1]
        view_matrix = np.array(cam_info[2]).reshape(4, 4, order='F')
        proj_matrix = np.array(cam_info[3]).reshape(4, 4, order='F')
        
        # Get camera position from view matrix
        camera_position = [view_matrix[0][3], view_matrix[1][3], view_matrix[2][3]]
        
        # Convert screen space coordinates to normalized device coordinates
        ndc_x = (2.0 * x) / width - 1.0
        ndc_y = 1.0 - (2.0 * y) / height
        
        # Calculate ray direction in clip space
        ray_clip = np.array([ndc_x, ndc_y, -1.0, 1.0])
        
        # Calculate ray direction in eye space
        inv_proj = np.linalg.inv(proj_matrix)
        ray_eye = np.dot(inv_proj, ray_clip)
        ray_eye = np.array([ray_eye[0], ray_eye[1], -1.0, 0.0])
        
        # Calculate ray direction in world space
        inv_view = np.linalg.inv(view_matrix)
        ray_world = np.dot(inv_view, ray_eye)
        ray_direction = np.array([ray_world[0], ray_world[1], ray_world[2]])
        ray_direction = ray_direction / np.linalg.norm(ray_direction)
        
        # Ray origin is the camera position
        ray_from = camera_position
        
        # Ray end point is some distance along the ray
        ray_to = [
            ray_from[0] + ray_direction[0] * 1000.0,
            ray_from[1] + ray_direction[1] * 1000.0,
            ray_from[2] + ray_direction[2] * 1000.0
        ]
        
        return ray_from, ray_to
    
    def _remove_brick(self, brick_id):
        """Remove a brick and its constraints from the simulation"""
        # First, remove all constraints involving this brick
        constraints_to_remove = self.constraints_map.get(brick_id, [])
        for constraint_id in constraints_to_remove:
            try:
                p.removeConstraint(constraint_id)
            except:
                pass  # Constraint might already be removed
        
        # Remove the brick from the simulation
        try:
            p.removeBody(brick_id)
        except:
            pass  # Body might already be removed
        
        # Update simulation data
        if brick_id in self.simulation_data['bricks']:
            idx = self.simulation_data['bricks'].index(brick_id)
            self.simulation_data['bricks'].remove(brick_id)
            if idx < len(self.simulation_data['normals']):
                self.simulation_data['normals'].pop(idx)
        
        # Update constraints map
        if brick_id in self.constraints_map:
            del self.constraints_map[brick_id]
        
        # Rebuild constraints map - constraints may have been removed
        # that affect other bodies
        self.constraints_map = {}
        self._map_constraints_to_bodies()
        # Step a few frames so the GUI refreshes without waiting for the main loop
        for _ in range(10):
            p.stepSimulation()
            time.sleep(0.001)
    
    def _save_vertex_locations(self):
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
        except Exception as e:
            print(f"Error saving vertex data: {e}")