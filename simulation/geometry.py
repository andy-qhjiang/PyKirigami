"""
Geometry utilities for the kirigami simulation project.
"""
import numpy as np
import pybullet as p

def create_3d_brick(vertices_flat, brick_thickness, normal_correction=1):
    """
    Create a 3D brick from 3D quad vertices.
    
    Args:
        vertices_flat: Flattened array of 3D vertices (12 values: 4 vertices with x,y,z)
        brick_thickness: Thickness of the brick
        normal_correction: Factor to correct normal direction (1 or -1)
        
    Returns:
        tuple: (vertices list for PyBullet, indices, center position, top vertices, normal)
    """
    half_thickness = brick_thickness / 2
    quad_vertices_3d = vertices_flat.reshape(4, 3)
    center = np.mean(quad_vertices_3d, axis=0)
    
    # Calculate normal
    vec1 = quad_vertices_3d[1] - quad_vertices_3d[0]
    vec2 = quad_vertices_3d[3] - quad_vertices_3d[0]
    normal = np.cross(vec1, vec2)
    normal = normal / np.linalg.norm(normal)
    normal = normal * normal_correction
    
    # Create vertices
    bottom_vertices = quad_vertices_3d - half_thickness * normal
    top_vertices_3d = quad_vertices_3d + half_thickness * normal
    
    verts = []
    for v in bottom_vertices:
        verts.append((v - center).tolist())
    for v in top_vertices_3d:
        verts.append((v - center).tolist())
    
    # Create face indices
    indices = [
        [0, 1, 2], [0, 2, 3],  # Bottom face
        [4, 5, 6], [4, 6, 7],  # Top face
        [0, 4, 1], [1, 4, 5],  # Side faces
        [1, 5, 2], [2, 5, 6],
        [2, 6, 3], [3, 6, 7],
        [3, 7, 0], [0, 7, 4]
    ]
    flat_indices = [idx for sublist in indices for idx in sublist]
    
    return verts, flat_indices, center.tolist(), top_vertices_3d.tolist(), normal.tolist()

def create_brick_body(verts, indices, center, mass=1.0):
    """
    Create a brick body in PyBullet.
    
    Args:
        verts: Vertices list
        indices: Indices list
        center: Center position
        mass: Mass of the brick
        
    Returns:
        int: PyBullet body ID
    """
    
    vis_shape = p.createVisualShape(p.GEOM_MESH, vertices=verts, indices=indices, rgbaColor=[.2, .5, .8, 1])
    col_shape = p.createCollisionShape(p.GEOM_MESH, vertices=verts)
    body_id = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=col_shape, 
                              baseVisualShapeIndex=vis_shape, basePosition=center)
    return body_id

def create_point_constraint(body_id1, body_id2, pivot_in_body1, pivot_in_body2):
    """
    Create a point-to-point constraint between two bodies.
    
    Args:
        body_id1: First body ID
        body_id2: Second body ID
        pivot_in_body1: Pivot point in body1's local coordinates
        pivot_in_body2: Pivot point in body2's local coordinates
        
    Returns:
        int: Constraint ID
    """
    return p.createConstraint(
        body_id1, -1, body_id2, -1, 
        p.JOINT_POINT2POINT, [0, 0, 0], 
        pivot_in_body1, pivot_in_body2
    )

def create_constraints_between_bricks(bricks, constraints, top_vertices, brick_centers):
    """Create constraints between bricks based on vertex connections.
    
    Args:
        bricks: List of brick body IDs
        constraints: List of constraints (f_i, v_j, f_p, v_q)
        top_vertices: List of top vertices for each brick
        brick_centers: List of brick center positions
    """
    for f_i, v_j, f_p, v_q in constraints:
        # Skip invalid constraints
        if (f_i >= len(bricks) or f_p >= len(bricks) or 
            v_j >= 4 or v_q >= 4):
            print(f"Warning: Skipping invalid constraint: {(f_i, v_j, f_p, v_q)}")
            continue
        
        # Get vertices and centers
        vertex_i_global = top_vertices[f_i][v_j]
        vertex_p_global = top_vertices[f_p][v_q]
        center_i = brick_centers[f_i]
        center_p = brick_centers[f_p]
        
        # Calculate pivot points in local coordinates
        pivot_in_i = [vertex_i_global[k] - center_i[k] for k in range(3)]
        pivot_in_p = [vertex_p_global[k] - center_p[k] for k in range(3)]
        
        # Create constraint
        create_point_constraint(bricks[f_i], bricks[f_p], pivot_in_i, pivot_in_p)

def calculate_outward_direction(center_pos, whole_center):
    """
    Calculate direction from whole center to a position.
    
    Args:
        center_pos: Position to calculate direction to
        whole_center: Center of the whole structure
        
    Returns:
        list: Normalized direction vector
    """
    direction = [
        center_pos[0] - whole_center[0],
        center_pos[1] - whole_center[1],
        center_pos[2] - whole_center[2]
    ]
    norm = np.linalg.norm(direction)
    if norm > 0:
        return [d / norm for d in direction]
    else:
        return [0, 0, 0]

def calculate_normal_direction(body_id, center_pos, orientation, normal):
    """
    Calculate normal direction in world coordinates.
    
    Args:
        body_id: Body ID
        center_pos: Center position
        orientation: Orientation quaternion
        normal: Normal vector in local coordinates
        
    Returns:
        list: Normalized normal vector in world coordinates
    """
    rotation_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)
    world_normal = np.dot(rotation_matrix, normal)
    world_normal = world_normal / np.linalg.norm(world_normal)
    return world_normal.tolist()