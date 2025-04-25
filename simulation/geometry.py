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
        tuple: (vertices list for PyBullet, indices, center position, bottom vertices, top vertices, normal)
    """
    # Reshape to get the four vertices of the quad
    quad_vertices = vertices_flat.reshape(4, 3)
    
    # Use original vertices as bottom vertices
    bottom_vertices = quad_vertices.copy()
    
    # Calculate the center point for centering in PyBullet
    center = np.mean(quad_vertices, axis=0)
    
    # Calculate normal
    vec1 = quad_vertices[1] - quad_vertices[0]
    vec2 = quad_vertices[3] - quad_vertices[0]
    normal = np.cross(vec1, vec2)
    normal = normal / np.linalg.norm(normal)
    
    # Create top vertices by adding thickness along normal
    top_vertices = quad_vertices + brick_thickness * normal
    
    # For PyBullet, we need to center the vertices around (0,0,0)
    # and then position the rigid body at the original center
    verts = []
    for v in bottom_vertices:
        verts.append((v - center).tolist())
    for v in top_vertices:
        verts.append((v - center).tolist())
    
    # Create face indices (unchanged)
    indices = [
        [0, 1, 2], [0, 2, 3],  # Bottom face
        [4, 5, 6], [4, 6, 7],  # Top face
        [0, 4, 1], [1, 4, 5],  # Side faces
        [1, 5, 2], [2, 5, 6],
        [2, 6, 3], [3, 6, 7],
        [3, 7, 0], [0, 7, 4]
    ]
    flat_indices = [idx for sublist in indices for idx in sublist]
    
    return verts, flat_indices, center.tolist(), bottom_vertices.tolist(), top_vertices.tolist(), normal.tolist()

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
    
    vis_shape = p.createVisualShape(p.GEOM_MESH, vertices=verts, indices=indices, rgbaColor=[.529, .808, .922, 1])
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
        list: List of created constraint IDs
    """
    created_constraints = []
    
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
    
    return created_constraints