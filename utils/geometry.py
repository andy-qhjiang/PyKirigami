"""
Geometry utilities for the kirigami simulation project.
"""
import numpy as np
import pybullet as p

def create_3d_brick(vertices_flat, brick_thickness):
    """
    Create a 3D brick from 3D polygon vertices (triangle or quadrilateral).
    
    Args:
        vertices_flat: List of 3D vertices (n*3 values: n vertices with x,y,z)
                      Supports n-vertex polygons (such as triangles with 9 values, quadrilaterals with 12 values).
        brick_thickness: Thickness of the brick
        
    Returns:
        tuple: (vertices list for PyBullet, visual_indices, center position, bottom vertices, top vertices, normal)
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
    # Create centered vertices list for PyBullet
    verts = []
    for v in bottom_vertices:
        verts.append((v - center).tolist())
    for v in top_vertices:
        verts.append((v - center).tolist())
    
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
    
    return (verts, visual_indices, center.tolist(), 
            bottom_vertices.tolist(), top_vertices.tolist(), normal.tolist())

def create_brick_body(verts, visual_indices, center, mass=1.0):
    """
    Create a brick body in PyBullet using visual indices.
    
    Args:
        verts: Vertices list (centered)
        visual_indices: Indices list for the visual shape 
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
                                type=0 for bottom point connection 
                                type=1 for top point connection
        bottom_vertices: List of bottom vertices for each brick
        top_vertices: List of top vertices for each brick
        brick_centers: List of brick center positions
        
    Returns:
       list: created_constraint_ids: List of created constraint IDs
        
    """
    created_constraints = []
    
    
    for f_i, v_j, f_p, v_q, constraint_type in constraints_with_types:
        # Skip invalid constraints
        num_1 = len(bottom_vertices[f_i])
        num_2 = len(bottom_vertices[f_p])
        if (f_i >= len(bricks) or f_p >= len(bricks) or 
            v_j >= num_1 or v_q >= num_2): # Vertices are 0,1,2,3
            print(f"Warning: Skipping invalid constraint input: {(f_i, v_j, f_p, v_q, constraint_type)}")
            continue
            
        center_i = np.array(brick_centers[f_i])
        center_p = np.array(brick_centers[f_p])

        def connect_points(body1_idx, body2_idx, vert1_global_coords, vert2_global_coords):
            pivot_in_1 = (np.array(vert1_global_coords) - center_i).tolist()
            pivot_in_2 = (np.array(vert2_global_coords) - center_p).tolist()
            c_id = create_point_constraint(bricks[body1_idx], bricks[body2_idx], pivot_in_1, pivot_in_2)
            created_constraints.append(c_id)
            

        if constraint_type == 0: # Spherical joint for bottom point connection
            vertex_i_global_bottom = bottom_vertices[f_i][v_j]
            vertex_p_global_bottom = bottom_vertices[f_p][v_q]
            connect_points(f_i, f_p, vertex_i_global_bottom, vertex_p_global_bottom)
            
        elif constraint_type == 1: # Spherical joint for top point connection
            vertex_i_global_top = top_vertices[f_i][v_j]
            vertex_p_global_top = top_vertices[f_p][v_q]
            connect_points(f_i, f_p, vertex_i_global_top, vertex_p_global_top)
        else:
            print(f"Warning: Unknown constraint type {constraint_type} for constraint {(f_i, v_j, f_p, v_q)}. Skipping.")
            continue
            
    return created_constraints