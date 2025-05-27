#!/usr/bin/env python3
"""
Improved Kirigami Demo with Direct Object Fixing

This demo shows a much simpler approach to object fixing using PyBullet's JOINT_FIXED constraint:
- Right-click on an object: Toggles between fixed and free states
- No need for multiple anchors or constraints

This approach provides more robust fixing by directly constraining all 6 degrees of freedom
with a single constraint, preventing both translation and rotation.
"""

import pybullet as p
import time
import math
import pybullet_data
import numpy as np

# Connect to PyBullet
cid = p.connect(p.SHARED_MEMORY)
if cid < 0:
    cid = p.connect(p.GUI)  # Store the connection ID
    print(f"Connected to PyBullet with ID: {cid}")
else:
    print(f"Connected to existing PyBullet server with ID: {cid}")

# Set up the environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1. / 120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")

# Load plane
planeId = p.loadURDF("plane100.urdf", useMaximalCoordinates=True)

# Configure visualization
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)

# Create some test objects
# Box 1
box1_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
box1_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5], rgbaColor=[1, 0, 0, 1])
box1_id = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box1_shape, 
                           baseVisualShapeIndex=box1_visual, basePosition=[0, 0, 2])

# Box 2  
box2_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.8])
box2_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.8], rgbaColor=[0, 1, 0, 1])
box2_id = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box2_shape,
                           baseVisualShapeIndex=box2_visual, basePosition=[2, 0, 2])

# Sphere
sphere_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.4)
sphere_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.4, rgbaColor=[0, 0, 1, 1])
sphere_id = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=sphere_shape,
                             baseVisualShapeIndex=sphere_visual, basePosition=[-2, 0, 2])

# Create connected objects for testing
# Create two boxes connected by a joint
box3_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3])
box3_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3], rgbaColor=[0, 0, 1, 1])
box3_id = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box3_shape,
                           baseVisualShapeIndex=box3_visual, basePosition=[0, 3, 1])

box4_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3])
box4_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3], rgbaColor=[1, 0, 1, 1])
box4_id = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box4_shape,
                           baseVisualShapeIndex=box4_visual, basePosition=[0.8, 3, 1])

# Connect them with a fixed joint
joint_id = p.createConstraint(
    parentBodyUniqueId=box3_id, parentLinkIndex=-1,
    childBodyUniqueId=box4_id, childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0.4, 0, 0],
    childFramePosition=[-0.4, 0, 0]
)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.stopStateLogging(logId)
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

# Simplified object fixing system
fixed_objects = {}  # object_id -> {'constraint_id': constraint_id, 'indicator_id': indicator_id}

def safe_normalize(vector):
    """Safely normalize a vector, handling zero-length vectors."""
    vector = np.array(vector)
    norm = np.linalg.norm(vector)
    if norm < 1e-8:
        return np.array([0.0, 0.0, 1.0])
    return vector / norm

def create_static_indicator(position, color=[1, 0, 0, 0.8]):
    """
    Create a static visual indicator at the specified position.
    
    Args:
        position: Where to create the indicator
        color: RGBA color for the indicator
        
    Returns:
        The ID of the created indicator or -1 if creation failed
    """
    try:
        visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.08, rgbaColor=color)
        indicator_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape,
                                       basePosition=position, useMaximalCoordinates=True)
        return indicator_id
    except Exception as e:
        print(f"Error creating indicator: {e}")
        return -1

def safe_get_object_position(object_id):
    """Safely get object position with error handling."""
    try:
        pos, orn = p.getBasePositionAndOrientation(object_id)
        return pos, orn
    except Exception as e:
        print(f"Error getting object position: {e}")
        return [0, 0, 0], [0, 0, 0, 1]

def toggle_object_fixing(object_id, hit_position):
    """
    Toggle object fixing using a single JOINT_FIXED constraint.
    - If object is not fixed: Fix it in its current position and orientation
    - If object is already fixed: Unfix it
    """
    try:
        if object_id in fixed_objects:
            # Object is already fixed - unfix it
            print(f"Object {object_id}: Removing constraint, object is now FREE")
            
            # Remove constraint
            try:
                p.removeConstraint(fixed_objects[object_id]['constraint_id'])
            except Exception as e:
                print(f"Error removing constraint: {e}")
            
            # Remove indicator
            try:
                p.removeBody(fixed_objects[object_id]['indicator_id'])
            except Exception as e:
                print(f"Error removing indicator: {e}")
            
            # Remove from fixed objects dictionary
            del fixed_objects[object_id]
            
        else:
            # Object is not fixed - fix it in place
            pos, orn = safe_get_object_position(object_id)
            
            # Create fixed constraint to world frame
            try:
                constraint_id = p.createConstraint(
                    parentBodyUniqueId=object_id,
                    parentLinkIndex=-1,
                    childBodyUniqueId=-1,  # World frame
                    childLinkIndex=-1,
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=[0, 0, 0],  # Center of object
                    childFramePosition=pos,  # Current position in world
                    parentFrameOrientation=[0, 0, 0, 1],
                    childFrameOrientation=orn  # Current orientation in world
                )
                
                # Ensure high max force for stability
                p.changeConstraint(constraint_id, maxForce=500000)
                
                # Create visual indicator
                indicator_id = create_static_indicator(hit_position)
                
                # Store in dictionary
                fixed_objects[object_id] = {
                    'constraint_id': constraint_id,
                    'indicator_id': indicator_id
                }
                
                print(f"Object {object_id}: Fixed in place with JOINT_FIXED constraint")
                print("✓ Object is now COMPLETELY FIXED (no translation or rotation)")
                
            except Exception as e:
                print(f"Error creating constraint: {e}")
    except Exception as e:
        print(f"Error in toggle_object_fixing: {e}")

def safe_raycast(rayFrom, rayTo):
    """
    Safely perform raycast with error handling.
    Returns (hit_object_id, hit_position) or (None, None) if no hit.
    """
    try:
        results = p.rayTest(rayFrom, rayTo)
        if results and len(results) > 0:
            rayTest = results[0]
            hitObjectUid = rayTest[0]
            
            if hitObjectUid >= 0:  # Valid hit
                hitPosition = rayTest[3]
                return hitObjectUid, hitPosition
    except Exception as e:
        print(f"Raycast error: {e}")
    
    return None, None

# Main simulation loop
print("=== Direct Object Fixing Demo (JOINT_FIXED Approach) ===")
print("Right-click on objects to toggle fixing:")
print("• First click: Object becomes completely fixed (no translation or rotation)")
print("• Second click: Object becomes free again")
print("\nTest Objects:")
print("• Red box: Individual object")
print("• Green box: Individual object")
print("• Blue sphere: Individual object")
print("• Blue & Purple boxes: Connected via joint (tests joint handling)")
print("=" * 60)

try:
    while True:
        try:
            # Check if still connected before processing events
            connection_info = p.getConnectionInfo()
            if connection_info['isConnected'] == 0:
                print("Connection to physics server lost. Attempting to reconnect...")
                try:
                    # Try to reconnect
                    cid = p.connect(p.GUI)
                    print(f"Reconnected to PyBullet with ID: {cid}")
                    # Reinitialize necessary settings
                    p.setGravity(0, 0, -9.81)
                    p.setRealTimeSimulation(1)
                except Exception as reconnect_error:
                    print(f"Failed to reconnect: {reconnect_error}")
                    time.sleep(1)  # Wait before trying again
                    continue
            
            mouse_events = p.getMouseEvents()
            
            for event in mouse_events:
                if event[0] == 2:  # MOUSE_BUTTON_EVENT
                    buttonIndex = event[3]
                    buttonState = event[4]

                    if buttonIndex == 2 and (buttonState & p.KEY_WAS_TRIGGERED):  # Right-click
                        try:
                            mouseX = event[1]
                            mouseY = event[2]

                            # Get camera information
                            cam_info = p.getDebugVisualizerCamera()
                            if not cam_info or len(cam_info) < 8:
                                print("Invalid camera info")
                                continue
                                
                            width, height = cam_info[0], cam_info[1]
                            view_matrix = cam_info[2]
                            proj_matrix = cam_info[3]
                            camPos = cam_info[4]
                            camForward = cam_info[5]
                            camRight = cam_info[6]
                            camUp = cam_info[7]

                            # Extract camera world position from view matrix
                            rayFrom = [
                                -view_matrix[0] * view_matrix[12] - view_matrix[1] * view_matrix[13] - view_matrix[2] * view_matrix[14],
                                -view_matrix[4] * view_matrix[12] - view_matrix[5] * view_matrix[13] - view_matrix[6] * view_matrix[14],
                                -view_matrix[8] * view_matrix[12] - view_matrix[9] * view_matrix[13] - view_matrix[10] * view_matrix[14]
                            ]

                            # Convert mouse coordinates to NDC space
                            ndc_x = (2.0 * mouseX / width - 1.0)
                            ndc_y = -(2.0 * mouseY / height - 1.0)

                            # Calculate ray direction with safe normalization
                            fov_y = 2.0 * math.atan(1.0 / proj_matrix[5])
                            aspect = width / height
                            scale_factor = math.tan(fov_y / 2.0)

                            # Safe normalization for camera vectors
                            camForward_normalized = safe_normalize(camForward)
                            camRight_normalized = safe_normalize(camRight)
                            camUp_normalized = safe_normalize(camUp)

                            rayDir = [
                                camForward_normalized[i] + 
                                (camRight_normalized[i] * ndc_x * scale_factor * aspect) + 
                                (camUp_normalized[i] * ndc_y * scale_factor)
                                for i in range(3)
                            ]

                            # Normalize ray direction safely
                            rayDir = safe_normalize(rayDir).tolist()
                            rayTo = [rayFrom[i] + rayDir[i] * 1000 for i in range(3)]

                            # Perform safe raycast
                            hitObjectUid, hitPosition = safe_raycast(rayFrom, rayTo)
                            
                            if hitObjectUid is not None:
                                print(f"\nHit object {hitObjectUid} at position {hitPosition}")
                                
                                # Skip ground plane
                                if hitObjectUid > 0:
                                    # Toggle object fixing
                                    toggle_object_fixing(hitObjectUid, hitPosition)
                                else:
                                    print("Hit ground plane (not fixable)")
                                
                                # Visual feedback
                                p.addUserDebugLine(rayFrom, hitPosition, lineColorRGB=[1, 1, 0], 
                                                lineWidth=2, lifeTime=1.0)
                            else:
                                print("No object hit")
                        except Exception as click_error:
                            print(f"Error handling click: {click_error}")
            
            # Step simulation with error handling
            try:
                # Check if connected before stepping
                if p.getConnectionInfo()['isConnected'] == 1:
                    p.stepSimulation()
                time.sleep(1./240.)
            except Exception as step_error:
                print(f"Error stepping simulation: {step_error}")
                time.sleep(0.1)  # Prevent rapid error loops
            
        except KeyboardInterrupt:
            print("\nSimulation stopped by user")
            break
        except Exception as loop_error:
            print(f"Error in main loop: {loop_error}")
            # Continue running despite errors
            time.sleep(0.1)
            
except Exception as e:
    print(f"Fatal error: {e}")
finally:
    print("Cleaning up resources...")
    try:
        # Check if we're still connected before trying to disconnect
        if p.getConnectionInfo()['isConnected'] == 1:
            # Clean up any remaining constraints and indicators
            for object_id in list(fixed_objects.keys()):
                try:
                    data = fixed_objects[object_id]
                    
                    try:
                        p.removeConstraint(data['constraint_id'])
                    except:
                        pass
                    
                    try:
                        p.removeBody(data['indicator_id'])
                    except:
                        pass
                except:
                    pass
            
            print("Disconnecting from PyBullet")
            p.disconnect()
    except Exception as cleanup_error:
        print(f"Error during cleanup: {cleanup_error}")
    
    print("Demo terminated")
