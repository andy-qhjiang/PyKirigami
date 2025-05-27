"""
Geometry utilities for the kirigami simulation project.
"""
import numpy as np
import pybullet as p

def create_3d_brick(vertices_flat, brick_thickness):
    """
    Create a 3D brick from 3D quad vertices.
    
    Args:
        vertices_flat: Flattened array of 3D vertices (12 values: 4 vertices with x,y,z)
        brick_thickness: Thickness of the brick
        
    Returns:
        tuple: (vertices list for PyBullet, single_sided_indices, double_sided_indices, center position, bottom vertices, top vertices, normal)
    """
    # Reshape vertices and calculate geometric properties
    quad_vertices = vertices_flat.reshape(4, 3)
    center = np.mean(quad_vertices, axis=0)
    vec1 = quad_vertices[1] - quad_vertices[0]
    vec2 = quad_vertices[3] - quad_vertices[0]
    normal = np.cross(vec1, vec2)
    norm_val = np.linalg.norm(normal)
    if norm_val == 0:
        # Handle degenerate case (e.g., collinear vertices)
        # Default to a Z-axis normal or raise an error
        print(f"Warning: Degenerate quad detected. Using default normal [0,0,1]. Vertices: {quad_vertices}")
        normal = np.array([0.0, 0.0, 1.0])
    else:
        normal = normal / norm_val
        
    top_vertices = quad_vertices + brick_thickness * normal
    bottom_vertices = quad_vertices.copy() # Keep original vertices as bottom

    # Create centered vertices list for PyBullet
    verts = []
    for v in bottom_vertices:
        verts.append((v - center).tolist())
    for v in top_vertices:
        verts.append((v - center).tolist())
    
    # Create face indices - SINGLE-SIDED for collision
    # Indices refer to the 'verts' list: 0-3 are bottom, 4-7 are top
    single_sided_indices_list = [
        # Bottom face (ensure consistent winding, e.g., counter-clockwise from outside)
        [0, 1, 2], [0, 2, 3],
        # Top face (ensure consistent winding)
        [4, 6, 5], [4, 7, 6], # Winding: 4->6->5, 4->7->6
        # Side faces (ensure consistent winding)
        [0, 4, 5], [0, 5, 1],
        [1, 5, 6], [1, 6, 2],
        [2, 6, 7], [2, 7, 3],
        [3, 7, 4], [3, 4, 0]
    ]
    single_sided_indices = [idx for sublist in single_sided_indices_list for idx in sublist]

    # Create face indices - DOUBLE-SIDED for visual
    double_sided_indices_list = []
    for face in single_sided_indices_list:
        double_sided_indices_list.append(face) # Original winding
        double_sided_indices_list.append(face[::-1]) # Reversed winding
        
    double_sided_indices = [idx for sublist in double_sided_indices_list for idx in sublist]
    
    return (verts, single_sided_indices, double_sided_indices, center.tolist(), 
            bottom_vertices.tolist(), top_vertices.tolist(), normal.tolist())

def create_brick_body(verts, collision_indices, visual_indices, center, mass=1.0):
    """
    Create a brick body in PyBullet using separate collision and visual indices.
    
    Args:
        verts: Vertices list (centered)
        collision_indices: Indices list for the collision shape (unused - we'll use convex hull)
        visual_indices: Indices list for the visual shape (double-sided)
        center: Center position (world coordinates)
        mass: Mass of the brick
        
    Returns:
        int: PyBullet body ID
    """
    # Set color with slight randomness
    r = 0.529 + np.random.uniform(-0.1, 0.1)
    g = 0.808 + np.random.uniform(-0.1, 0.1)
    b = 0.922 + np.random.uniform(-0.1, 0.1)
    color = [max(0, min(1, r)), max(0, min(1, g)), max(0, min(1, b)), 1]
    
    # Create visual shape - use double-sided indices for visibility
    vis_shape = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        vertices=verts,
        indices=visual_indices, # Use double-sided indices for visibility
        rgbaColor=color,
        specularColor=[0.4, 0.4, 0.4]
    )
    
    # Create collision shape - use ONLY vertices (no indices) for proper physics
    # This makes PyBullet treat it as a convex hull instead of a triangle mesh
    col_shape = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        vertices=verts
        # No indices parameter - this creates a convex hull
    )
    
    # Create the multibody
    body_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col_shape,
        baseVisualShapeIndex=vis_shape,
        basePosition=center
    )
        
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

def create_constraints_between_bricks(bricks, constraints_with_types, bottom_vertices, top_vertices, brick_centers):
    """Create constraints between bricks based on vertex connections and type.
    
    Args:
        bricks: List of brick body IDs
        constraints_with_types: List of constraints (f_i, v_j, f_p, v_q, type)
                                type=0 for hinge (connects two points, top and bottom)
                                type=1 for point (connects one point, bottom only)
        bottom_vertices: List of bottom vertices for each brick
        top_vertices: List of top vertices for each brick
        brick_centers: List of brick center positions
        
    Returns:
       list: created_constraint_ids: List of created constraint IDs
        
    """
    created_constraints = []
    
    
    for f_i, v_j, f_p, v_q, constraint_type in constraints_with_types:
        # Skip invalid constraints
        if (f_i >= len(bricks) or f_p >= len(bricks) or 
            v_j >= 4 or v_q >= 4): # Vertices are 0,1,2,3
            print(f"Warning: Skipping invalid constraint input: {(f_i, v_j, f_p, v_q, constraint_type)}")
            continue
            
        center_i = np.array(brick_centers[f_i])
        center_p = np.array(brick_centers[f_p])

        def connect_points(body1_idx, body2_idx, vert1_global_coords, vert2_global_coords):
            pivot_in_1 = (np.array(vert1_global_coords) - center_i).tolist()
            pivot_in_2 = (np.array(vert2_global_coords) - center_p).tolist()
            c_id = create_point_constraint(bricks[body1_idx], bricks[body2_idx], pivot_in_1, pivot_in_2)
            created_constraints.append(c_id)
            

        if constraint_type == 1: # Spherical joint (bottom point connection)
            vertex_i_global_bottom = bottom_vertices[f_i][v_j]
            vertex_p_global_bottom = bottom_vertices[f_p][v_q]
            connect_points(f_i, f_p, vertex_i_global_bottom, vertex_p_global_bottom)
            
        elif constraint_type == 2: # Revolute joint (hinge around a shared vertex axis)
            # Connect the bottom points of the shared vertex
            vertex_i_global_bottom = bottom_vertices[f_i][v_j]
            vertex_p_global_bottom = bottom_vertices[f_p][v_q]
            connect_points(f_i, f_p, vertex_i_global_bottom, vertex_p_global_bottom)

            # Connect the top points of the shared vertex
            vertex_i_global_top = top_vertices[f_i][v_j]
            vertex_p_global_top = top_vertices[f_p][v_q]
            connect_points(f_i, f_p, vertex_i_global_top, vertex_p_global_top)
        else:
            print(f"Warning: Unknown constraint type {constraint_type} for constraint {(f_i, v_j, f_p, v_q)}. Skipping.")
            continue
            
    return created_constraints