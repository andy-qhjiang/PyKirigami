# # This is a Python script to demostrate the difference between single-sided and double-sided quads in PyBullet.

# import numpy as np  
# import pybullet as p
# import pybullet_data
# # Ensure PyBullet can find the data files


# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())


# verts = [
#     [1, -1, 1],
#     [1, 1, 1],
#     [-1, 1, 1],
#     [-1, -1, 1],
#     [1, -1, -1],
#     [1, 1, -1],
#     [-1, 1, -1],
#     [-1, -1, -1]
# ]

# single_sided_indices = [
#     [0, 1, 2], [0, 2, 3],
#     # Top face (ensure consistent winding)
#     [4, 6, 5], [4, 7, 6], # Winding: 4->6->5, 4->7->6
#     # Side faces (ensure consistent winding)
#     [0, 4, 5], [0, 5, 1],
#     [1, 5, 6], [1, 6, 2],
#     [2, 6, 7], [2, 7, 3],
#     [3, 7, 4], [3, 4, 0]
# ]


# # Set color with slight randomness
# r = 0.529 + np.random.uniform(-0.1, 0.1)
# g = 0.808 + np.random.uniform(-0.1, 0.1)
# b = 0.922 + np.random.uniform(-0.1, 0.1)
# color = [max(0, min(1, r)), max(0, min(1, g)), max(0, min(1, b)), 1]

# visual_indices = [idx for face in single_sided_indices for idx in face]



# vis_shape = p.createVisualShape(
#         shapeType=p.GEOM_MESH,
#         vertices=verts,
#         indices=visual_indices, # Use double-sided indices for visibility
#         rgbaColor=color,
#         specularColor=[0.4, 0.4, 0.4]
#     )
    
#     # Create collision shape - use ONLY vertices (no indices) for proper physics
#     # This makes PyBullet treat it as a convex hull instead of a triangle mesh
# col_shape = p.createCollisionShape(
#     shapeType=p.GEOM_MESH,
#     vertices=verts
#     # No indices parameter - this creates a convex hull
# )
    
# # Create the multibody
# body_id = p.createMultiBody(
#     baseMass=2.0,
#     baseCollisionShapeIndex=col_shape,
#     baseVisualShapeIndex=vis_shape,
#     basePosition=[2,2,2]
# )

# p.loadURDF("plane.urdf")  # Load ground plane

# while True:
#     p.stepSimulation()
#     p.setGravity(0, 0, -9.81)  # Set gravity

import numpy as np
import pybullet as p
import pybullet_data

# --- Boilerplate ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.81)

# --- The Fix: Define Vertices and Normals for a Flat-Shaded Cube ---

# Define 24 vertices, 4 for each of the 6 faces.
# This prevents the renderer from averaging normals at the corners.
verts = [
    # Top face (Y-up)
    [1.0, 1.0, 1.0], [-1.0, 1.0, 1.0], [-1.0, -1.0, 1.0], [1.0, -1.0, 1.0],
    # Bottom face
    [1.0, -1.0, -1.0], [-1.0, -1.0, -1.0], [-1.0, 1.0, -1.0], [1.0, 1.0, -1.0],
    # Front face
    [1.0, 1.0, -1.0], [-1.0, 1.0, -1.0], [-1.0, 1.0, 1.0], [1.0, 1.0, 1.0],
    # Back face
    [-1.0, -1.0, 1.0], [-1.0, -1.0, -1.0], [1.0, -1.0, -1.0], [1.0, -1.0, 1.0],
    # Right face
    [1.0, -1.0, 1.0], [1.0, -1.0, -1.0], [1.0, 1.0, -1.0], [1.0, 1.0, 1.0],
    # Left face
    [-1.0, 1.0, 1.0], [-1.0, 1.0, -1.0], [-1.0, -1.0, -1.0], [-1.0, -1.0, 1.0]
]

# All vertices on a given face must have the same normal vector.
# This ensures the face is lit uniformly.
normals = [
    # Top face normal (all 4 vertices point straight up)
    [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1],
    # Bottom face normal
    [0, 0, -1], [0, 0, -1], [0, 0, -1], [0, 0, -1],
    # Front face normal
    [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0],
    # Back face normal
    [0, -1, 0], [0, -1, 0], [0, -1, 0], [0, -1, 0],
    # Right face normal
    [1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0],
    # Left face normal
    [-1, 0, 0], [-1, 0, 0], [-1, 0, 0], [-1, 0, 0]
]

# Indices now refer to the 24-vertex list.
# Ensure Counter-Clockwise (CCW) winding order.
indices = [
    0, 1, 2, 0, 2, 3,        # Top
    4, 5, 6, 4, 6, 7,        # Bottom
    8, 9, 10, 8, 10, 11,     # Front
    12, 13, 14, 12, 14, 15,  # Back
    16, 17, 18, 16, 18, 19,  # Right
    20, 21, 22, 20, 22, 23   # Left
]


vis_shape = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    vertices=verts,
    indices=indices,
    # Provide the explicit normals to prevent averaging
    normals=normals,
    rgbaColor=[0.529, 0.808, 0.922, 1],
    specularColor=[0.4, 0.4, 0.4]
)

# For collision, the original convex hull approach is still best.
col_shape = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    vertices=[[1,1,1], [1,-1,1], [-1,-1,1], [-1,1,1], [1,1,-1], [1,-1,-1], [-1,-1,-1], [-1,1,-1]]
)

body_id = p.createMultiBody(
    baseMass=2.0,
    baseCollisionShapeIndex=col_shape,
    baseVisualShapeIndex=vis_shape,
    basePosition=[0, 0, 3] # Adjusted position for clarity
)

# --- Simulation Loop ---
while p.isConnected():
    p.stepSimulation()