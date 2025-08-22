"""
Geometry utilities for the kirigami simulation project.
"""
import numpy as np
import pybullet as p

def create_ground_plane(z, thickness, color=(0.4, 0.4, 0.4, 1.0)):
    """Create a large axis-aligned ground box.

    Args:
        z (float): World Z coordinate of the box center (NOT the top surface).
        thickness (float): Half-thickness of the box (full thickness = 2 * thickness).
        color (tuple[float,float,float,float]): RGBA color.

    Returns:
        int: PyBullet body ID of the created static ground.
    """
    
    ground_collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[50000, 50000, thickness]
    )
    ground_visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[50000, 50000, thickness],
        rgbaColor=list(color),
        visualFrameOrientation=[-0.1, -0.2, -0.2, 1]
    )
    
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=ground_collision_shape,
        baseVisualShapeIndex=ground_visual_shape,
        basePosition=[0, 0, z]
    )
    

def create_extruded_geometry(vertices_flat, brick_thickness):
    """
    Create 3D extruded geometry from 2D polygon vertices.
    
    Takes a flat polygon and extrudes it along its normal vector to create
    a 3D brick with all geometric data needed for PyBullet physics simulation.
    
    Args:
        vertices_flat: List of 3D vertices (n*3 values: n vertices with x,y,z)
                      Supports n-vertex polygons (such as triangles with 9 values, quadrilaterals with 12 values).
        brick_thickness: Thickness of the extruded brick
        
    Returns:
        tuple: (local_verts, visual_indices, center)
        local_verts: List of vertices relative to the center
    """
    # Convert to numpy array and reshape vertices
    vertices_array = np.array(vertices_flat)
    num_vertices = len(vertices_array) // 3
    planar_vertices = vertices_array.reshape(num_vertices, 3)
    
    # Calculate geometric properties
    center = np.mean(planar_vertices, axis=0)
    
    # Calculate normal using first three vertices
    vec1 = planar_vertices[1] - planar_vertices[0]
    vec2 = planar_vertices[2] - planar_vertices[0]
    normal = np.cross(vec1, vec2)
    norm_val = np.linalg.norm(normal)
    
    if norm_val == 0:
        # Handle degenerate case (e.g., collinear vertices)
        print(f"Warning: Degenerate polygon detected. Using default normal [0,0,1]. Vertices: {planar_vertices}")
        normal = np.array([0.0, 0.0, 1.0])
    else:
        normal = normal / norm_val
        
    top_vertices = planar_vertices + brick_thickness * normal
    bottom_vertices = planar_vertices.copy() # Keep original vertices as bottom

    center = center + (brick_thickness / 2) * normal  # Center adjusted for thickness

    # Create centered vertices list for PyBullet (relative to center)
    local_verts = []
    for v in bottom_vertices:
        local_verts.append((v - center).tolist())
    for v in top_vertices:
        local_verts.append((v - center).tolist())
    
    # Create face indices for visual shape
    visual_indices = []
    
    # Bottom face triangulation (counter-clockwise from outside)
    for i in range(1, num_vertices - 1):
        visual_indices.extend([0, i+1, i])
    
    # Top face triangulation (counter-clockwise from outside)  
    for i in range(1, num_vertices - 1):
        visual_indices.extend([num_vertices, num_vertices + i, num_vertices + i + 1])
    
    # Side faces (connect bottom and top vertices)
    for i in range(num_vertices):
        next_i = (i + 1) % num_vertices
        # Two triangles per side face
        # Triangle 1: bottom[i] -> top[i] -> top[next_i]
        visual_indices.extend([i, num_vertices + next_i, num_vertices + i])
        # Triangle 2: bottom[i] -> top[next_i] -> bottom[next_i] 
        visual_indices.extend([i, next_i, num_vertices + next_i])

    
    local_normals = []  # Reset and compute properly
    # Bottom vertices: -normal
    for _ in range(num_vertices):
        local_normals.append((-normal).tolist())
    # Top vertices: +normal
    for _ in range(num_vertices):
        local_normals.append(normal.tolist())
    
    return (local_verts, visual_indices, center.tolist(), local_normals)

def create_brick_body(local_verts, visual_indices, center, local_normals,mass=1.0):
    """
    Create a brick body in PyBullet using visual indices.
    
    Args:
        local_verts: Vertices list (relative to center)
        visual_indices: Indices list for the visual shape 
        center: Center position (world coordinates)
        mass: Mass of the brick
        
    Returns:
        int: PyBullet body ID
    """

    # Set color with slight randomness (terracotta base)
    r = 0.8 + np.random.uniform(-0.1, 0.1)
    g = 0.4 + np.random.uniform(-0.1, 0.1)
    b = 0.2 + np.random.uniform(-0.1, 0.1)
    color = [min(1, r), min(1, g), min(1, b), 1]
    brick_color = color

    # Create visual shape
    vis_shape = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        vertices=local_verts,
        indices=visual_indices, 
        normals=local_normals,
        rgbaColor=brick_color,
        specularColor=[0.4, 0.4, 0.4]
    )
    
    # Create collision shape - use ONLY vertices (no indices) for proper physics
    # This makes PyBullet treat it as a convex hull instead of a triangle mesh
    col_shape = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        vertices=local_verts
        # No indices parameter - this creates a convex hull
    )
    
    # Create the multibody
    # Create the multibody
    body_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col_shape,
        baseVisualShapeIndex=vis_shape,
        basePosition=center
    )
        
    return body_id

def create_constraints_between_bricks(bricks, constraints_with_types, local_verts_list):
    """Create constraints between bricks based on vertex connections and type.
    
    Args:
        bricks: List of brick body IDs
        constraints_with_types: List of constraints (f_i, v_j, f_p, v_q, type)
                                type=1 for bottom point connection 
                                type=2 for top point connection
        local_verts_list: List of local vertex coordinates for each brick
        
    Returns:
       list: created_constraint_ids: List of created constraint IDs
        
    """
    created_constraints = []
    
    for f_i, v_j, f_p, v_q, constraint_type in constraints_with_types:
        # Calculate number of bottom vertices for validation
        num_1 = len(local_verts_list[f_i]) // 2
        num_2 = len(local_verts_list[f_p]) // 2
        
        # Skip invalid constraints
        if (f_i >= len(bricks) or f_p >= len(bricks) or 
            v_j >= num_1 or v_q >= num_2):
            print(f"Warning: Skipping invalid constraint input: {(f_i, v_j, f_p, v_q, constraint_type)}")
            continue

        if constraint_type == 1: # Bottom point connection
            # Bottom vertices are first half of local_verts
            pivot_in_1 = local_verts_list[f_i][v_j]
            pivot_in_2 = local_verts_list[f_p][v_q]
            
        elif constraint_type == 2: # Top point connection
            # Top vertices are second half of local_verts
            pivot_in_1 = local_verts_list[f_i][num_1 + v_j]
            pivot_in_2 = local_verts_list[f_p][num_2 + v_q]
            
        else:
            print(f"Warning: Unknown constraint type {constraint_type} for constraint {(f_i, v_j, f_p, v_q)}. Skipping.")
            continue
            
        # Create constraint directly using local coordinates
        c_id = p.createConstraint(
            bricks[f_i], -1, bricks[f_p], -1, 
            p.JOINT_POINT2POINT, [0, 0, 0], 
            pivot_in_1, pivot_in_2
        )
        created_constraints.append(c_id)
            
    return created_constraints


def transform_local_to_world_coordinates(body_ids, local_vertices):
    """
    Transform local vertex coordinates to world coordinates based on current body poses.
    
    This function takes local vertex coordinates (relative to body centers) and transforms
    them to world coordinates using the current position and orientation of each body
    from the physics simulation.
    
    Args:
        body_ids: List of PyBullet body IDs
        local_vertices: List of local vertex coordinate arrays for each body
                       [[[x1,y1,z1], [x2,y2,z2], ...], ...]

    Returns:
        list: List of transformed vertices in world coordinates for each body
              Each element contains the world coordinates of all vertices for that body
    """
    transformed_vertices = []
    
    for i, body_id in enumerate(body_ids):
        if i >= len(local_vertices):
            continue
            
        # Get current pose of this brick
        current_pos, current_orn = p.getBasePositionAndOrientation(body_id)
        
        # Convert quaternion to rotation matrix
        rotation_matrix = np.array(p.getMatrixFromQuaternion(current_orn)).reshape(3, 3)
        
        # Vectorized transformation of all vertices for this brick
        local_verts = np.array(local_vertices[i])  # Shape: (n_vertices, 3)
        world_verts = (rotation_matrix @ local_verts.T).T + np.array(current_pos)  # Shape: (n_vertices, 3)
        
        # Add this brick's transformed vertices to the result
        transformed_vertices.append(world_verts.tolist())
    
    return transformed_vertices