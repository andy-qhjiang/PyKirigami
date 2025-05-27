# This is an examplilary script to demonstrate the use of PyBullet for interaction through mouse events and ray casting.
# It allows the user to click on objects in the simulation to toggle their static/dynamic state.
# The script uses a duck mesh and a box object, and it visualizes the ray direction and hit points.
import pybullet as p
import time
import math
import pybullet_data
import numpy as np

# Connect to PyBullet
cid = p.connect(p.SHARED_MEMORY)
if cid < 0:
    p.connect(p.GUI)

# Set up the environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1. / 120.) # Using 120Hz for timestep
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")

# Load plane and store its ID
planeId = p.loadURDF("plane100.urdf", useMaximalCoordinates=True)

# Disable rendering during initial creation for performance
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1) # Keep GUI elements like sliders
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)


# Define shape parameters
shift = [0, -0.02, 0]
meshScale = [0.5, 0.5, 0.5]  # Scale for the duck mesh

# Create visual and collision shapes for the duck
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="duck.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shift,
                                    meshScale=meshScale)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="duck_vhacd.obj",
                                          collisionFramePosition=shift,
                                          meshScale=meshScale)

# Create a box object
box_half_extents = [0.5, 0.5, 0.5]
box_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                        halfExtents=box_half_extents,
                                        rgbaColor=[0, 1, 0, 1]) # Green box
box_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                             halfExtents=box_half_extents)
box_start_position = [2, 2, 1] # Position it away from the ducks
box_id = p.createMultiBody(baseMass=1,
                           baseCollisionShapeIndex=box_collision_shape_id,
                           baseVisualShapeIndex=box_visual_shape_id,
                           basePosition=box_start_position)

# List to store box body IDs, similar to duck_ids
box_ids = [box_id]

# Create ducks and store their body IDs
rangex = 2
rangey = 2
duck_ids = []  # List to store duck body IDs
for i in range(rangex):
    for j in range(rangey):
        body_id = p.createMultiBody(baseMass=1,
                                    baseInertialFramePosition=[0, 0, 0],
                                    baseCollisionShapeIndex=collisionShapeId,
                                    baseVisualShapeIndex=visualShapeId,
                                    basePosition=[((-rangex / 2) + i) * meshScale[0] * 2,
                                                  (-rangey / 2 + j) * meshScale[1] * 2, 1],
                                    useMaximalCoordinates=True)
        duck_ids.append(body_id)

# Enable rendering after creation
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.stopStateLogging(logId) # Stop logging after setup
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1) # Using real-time simulation

# Dictionary to track constraints and indicators

fixed_objects = {}  # To track which objects are fixed (static)

def create_static_indicator(object_id, hit_position=None):

    """Create a visual indicator for fixed/static objects at hit position"""
    pos, _ = p.getBasePositionAndOrientation(object_id)
    
    # If no hit position provided, use object center
    if hit_position is None:
        hit_position = pos

    # Create indicator 
    visual_shape = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.08,
            rgbaColor=[1, 0, 0, 0.8]
        ) 
    indicator_pos = hit_position

    # Create main indicator body
    indicator_id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual_shape,
        basePosition=indicator_pos,
        baseOrientation= [0,0,0,1],
        useMaximalCoordinates=True
    )
    
    return indicator_id


def toggle_static(object_id, hit_position=None):
    """Toggle an object between static and dynamic"""
    if object_id in fixed_objects:
        # Object is static - make it dynamic again
        original_mass, indicator_id = fixed_objects[object_id]
        p.changeDynamics(object_id, -1, mass=original_mass)
        
        # Remove all associated indicators
       
        p.removeBody(indicator_id)
        del fixed_objects[object_id]
        
        print(f"Object {object_id} is now free to move")
        return False  # Object is now dynamic
    else:
        # Object is dynamic - make it static
        
        # Temporarily pause simulation to let object settle
        try: 
            current_mass = p.getDynamicsInfo(object_id, -1)[0]
        except p.error as e:
            print(f"Error getting dynamics info: {e}")
            current_mass = 1.0  # Default mass if not found
        if current_mass == 0:
            current_mass = 1.0  # Default mass if not found

        p.setRealTimeSimulation(0)
        p.resetBaseVelocity(object_id, [0, 0, 0], [0, 0, 0])
        
        # Step simulation a few times
        for _ in range(5):
            p.stepSimulation()
        
        # Now make it static
        p.changeDynamics(object_id, -1, mass=0.0)
        
        # Create indicator at hit position
        indicator_id = create_static_indicator(object_id, hit_position)
        fixed_objects[object_id] = (current_mass, indicator_id)
        
        # Resume simulation
        p.setRealTimeSimulation(1)
        
        print(f"Object {object_id} is now fixed in place")
        return True  # Object is now static


# Main simulation loop
while True:
    # Get mouse events
    mouse_events = p.getMouseEvents()
    
    for event in mouse_events:
        if event[0] == 2:  # MOUSE_BUTTON_EVENT
            buttonIndex = event[3]
            buttonState = event[4]

            if buttonIndex == 2 and (buttonState & p.KEY_WAS_TRIGGERED):  # Right-click WAS TRIGGERED
                mouseX = event[1]
                mouseY = event[2]

                # Get camera information
                cam_info = p.getDebugVisualizerCamera()
                width, height = cam_info[0], cam_info[1]
                view_matrix = cam_info[2]
                proj_matrix = cam_info[3]
                camPos = cam_info[4]
                camForward = cam_info[5]
                camRight = cam_info[6]
                camUp = cam_info[7]
                
                # print (cam_info)  # Debug camera info
                print(f"Mouse position: ({mouseX}, {mouseY})")
                print(f"Camera position: {camPos}")
                print(f"Camera forward vector: {camForward}")   
                print(f"Camera right vector: {camRight}")
                print(f"Camera up vector: {camUp}")
                print(f"View matrix: {view_matrix}")
                print(f"Projection matrix: {proj_matrix}")
                

                # Extract camera world position from view matrix
                # The camera position is at (0,0,1) in camera space, but we need world space
                # Extract from the view matrix's inverse translation component
                # This is a simplified method - in a full implementation, you'd invert the matrix
                rayFrom = [
                    -view_matrix[0] * view_matrix[12] - view_matrix[1] * view_matrix[13] - view_matrix[2] * view_matrix[14],
                    -view_matrix[4] * view_matrix[12] - view_matrix[5] * view_matrix[13] - view_matrix[6] * view_matrix[14],
                    -view_matrix[8] * view_matrix[12] - view_matrix[9] * view_matrix[13] - view_matrix[10] * view_matrix[14]
                ]

                # Convert mouse coordinates to NDC space (-1 to 1)
                ndc_x = (2.0 * mouseX / width - 1.0)
                ndc_y = -(2.0 * mouseY / height - 1.0)  # Y is flipped in screen space

                # Get an approximate field of view from projection matrix
                # This helps scale the offsets correctly
                fov_y = 2.0 * math.atan(1.0 / proj_matrix[5]) # Extract FOV from projection matrix
                aspect = width / height

                # Properly calculate the ray direction
                # 1. Start with camera forward vector (normalized)
                # 2. Add scaled offsets for horizontal and vertical components
                # The scale depends on FOV - wider FOV means larger offsets
                camForward_normalized = camForward/np.linalg.norm(np.array(camForward))
                camRight_normalized = camRight/np.linalg.norm(np.array(camRight))
                camUp_normalized = camUp/np.linalg.norm(np.array(camUp))

                # Scale factor based on field of view
                # This is crucial - it ensures the ray passes through the clicked pixel
                scale_factor = math.tan(fov_y / 2.0)

                rayDir = [
                    camForward_normalized[i] + 
                    (camRight_normalized[i] * ndc_x * scale_factor * aspect) + 
                    (camUp_normalized[i] * ndc_y * scale_factor)
                    for i in range(3)
                ]

                # Normalize the final direction
                length = math.sqrt(sum([c*c for c in rayDir]))
                rayDir = [c / length for c in rayDir]

                # Ray end point in world space
                rayTo = [rayFrom[i] + rayDir[i] * 1000 for i in range(3)]

                # Add visual debug elements
                # Coordinate system at camera position
                axis_length = 0.5
                p.addUserDebugLine(rayFrom, [rayFrom[i] + camRight[i] * axis_length for i in range(3)],
                                 lineColorRGB=[1, 0, 0], lineWidth=2, lifeTime=30.5)  # X-axis (red)
                p.addUserDebugLine(rayFrom, [rayFrom[i] + camUp[i] * axis_length for i in range(3)],
                                 lineColorRGB=[0, 1, 0], lineWidth=2, lifeTime=30.5)  # Y-axis (green)
                p.addUserDebugLine(rayFrom, [rayFrom[i] + camForward[i] * axis_length for i in range(3)],
                                 lineColorRGB=[0, 0, 1], lineWidth=2, lifeTime=30.5)  # Z-axis (blue)

                # Ray direction visualization (100 units long)
                rayTo_viz = [rayFrom[i] + rayDir[i] * 100 for i in range(3)]
                p.addUserDebugLine(rayFrom, rayTo_viz, lineColorRGB=[1, 1, 0], lineWidth=2, lifeTime=30.5)

                # Perform the ray test (with longer distance)
                rayTest = p.rayTest(rayFrom, rayTo)[0]
                hitObjectUid = rayTest[0]

                if hitObjectUid >= 0:
                    hitPosition = rayTest[3]
                    print(f"Hit object {hitObjectUid} at position {hitPosition}")
                    
                    # Draw line from camera to hit point (red)
                    p.addUserDebugLine(rayFrom, hitPosition, lineColorRGB=[1, 0, 0], lineWidth=1, lifeTime=0.5)
                    
                    # # Visualize hit point with a small red sphere
                    # visual_shape = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                    #                                  radius=0.1,  # Small radius
                    #                                  rgbaColor=[1, 0, 0, 0.8])
                    # p.createMultiBody(baseMass=0,
                    #                 baseVisualShapeIndex=visual_shape,
                    #                 basePosition=hitPosition,
                    #                 useMaximalCoordinates=True)
                    
                    # Toggle constraint for the hit object (skip the ground plane)
                    if hitObjectUid > 0:
                        toggle_static(hitObjectUid, hitPosition)
                else:
                    print("No hit")

    # Step simulation and maintain timing
    p.stepSimulation()
    time.sleep(1./240.)

