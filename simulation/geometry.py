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

def create_constraints_between_bricks(bricks, constraints, bottom_vertices, top_vertices, brick_centers, connection_mode='bottom'):
    """Create constraints between bricks based on vertex connections.
    
    Args:
        bricks: List of brick body IDs
        constraints: List of constraints (f_i, v_j, f_p, v_q)
        bottom_vertices: List of bottom vertices for each brick
        top_vertices: List of top vertices for each brick
        brick_centers: List of brick center positions
        connection_mode: Which vertices to connect ('bottom', 'top', 'both')
                        - 'bottom': Connect original vertices (good for 3D space simulations)
                        - 'top': Connect only top vertices
                        - 'both': Connect both bottom and top (best for planar stability)
        
    Returns:
        tuple: (created_constraint_ids, constraint_mapping)
            - created_constraint_ids: List of created constraint IDs
            - constraint_mapping: List of (constraint_id, tile_idx1, tile_idx2) tuples
    """
    created_constraints = []
    constraint_mapping = []
    
    for f_i, v_j, f_p, v_q in constraints:
        # Skip invalid constraints
        if (f_i >= len(bricks) or f_p >= len(bricks) or 
            v_j >= 4 or v_q >= 4):
            print(f"Warning: Skipping invalid constraint: {(f_i, v_j, f_p, v_q)}")
            continue
            
        # Get centers
        center_i = brick_centers[f_i]
        center_p = brick_centers[f_p]
        
        # Connect based on connection mode
        if connection_mode in ('bottom', 'both'):
            # Connect bottom vertices
            vertex_i_global = bottom_vertices[f_i][v_j]
            vertex_p_global = bottom_vertices[f_p][v_q]
            
            # Calculate pivot points in local coordinates
            pivot_in_i = [vertex_i_global[k] - center_i[k] for k in range(3)]
            pivot_in_p = [vertex_p_global[k] - center_p[k] for k in range(3)]
            
            # Create constraint
            c_id = create_point_constraint(bricks[f_i], bricks[f_p], pivot_in_i, pivot_in_p)
            created_constraints.append(c_id)
            
            # Store constraint mapping for easy lookup
            constraint_mapping.append((c_id, f_i, f_p))
            
        if connection_mode in ('top', 'both'):
            # Connect top vertices
            vertex_i_global_top = top_vertices[f_i][v_j]
            vertex_p_global_top = top_vertices[f_p][v_q]
            
            # Calculate pivot points in local coordinates
            pivot_in_i_top = [vertex_i_global_top[k] - center_i[k] for k in range(3)]
            pivot_in_p_top = [vertex_p_global_top[k] - center_p[k] for k in range(3)]
            
            # Create constraint
            c_id = create_point_constraint(bricks[f_i], bricks[f_p], pivot_in_i_top, pivot_in_p_top)
            created_constraints.append(c_id)
            
            # Store constraint mapping for easy lookup
            constraint_mapping.append((c_id, f_i, f_p))
    
    return created_constraints, constraint_mapping