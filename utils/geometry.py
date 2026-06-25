"""
Geometry utilities for the kirigami simulation project.
"""
import numpy as np
import pybullet as p

def create_ground_plane(z, thickness=1, friction=0.5,color=(0.35, 0.35, 0.35, 1.0)):
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
        visualFrameOrientation=[0, 0, 0, 1], # one can create a tilted ground for better view in some cases
        specularColor=[0, 0, 0] # the ground would not reflect light
    )

    ground_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=ground_collision_shape,
        baseVisualShapeIndex=ground_visual_shape,
        basePosition=[0, 0, z-thickness]
    )
    p.changeDynamics(ground_id, -1, lateralFriction=friction)
    return ground_id

    

def build_extruded_visual_mesh(planar_vertices, normal, brick_thickness, center):
    n = len(planar_vertices)
    top = planar_vertices + brick_thickness * normal
    bottom = planar_vertices

    vis_vertices = []
    vis_normals = []
    vis_indices = []

    def add(v, nrm):
        vis_vertices.append((v - center).tolist())
        vis_normals.append(nrm.tolist())
        return len(vis_vertices) - 1

    # Bottom cap: CCW when viewed from outside (outside is -normal)
    bot_idx = [add(v, -normal) for v in bottom]
    for i in range(1, n - 1):
        vis_indices += [bot_idx[0], bot_idx[i + 1], bot_idx[i]]  # fan, reversed order

    # Top cap: CCW when viewed from outside (outside is +normal)
    top_idx = [add(v, normal) for v in top]
    for i in range(1, n - 1):
        vis_indices += [top_idx[0], top_idx[i], top_idx[i + 1]]  # fan

    # Sides: duplicate vertices per side, use outward normal per side
    poly_center = np.mean(planar_vertices, axis=0)
    for i in range(n):
        j = (i + 1) % n
        edge = planar_vertices[j] - planar_vertices[i]
        side_n = np.cross(normal, edge)
        side_n_norm = np.linalg.norm(side_n)
        if side_n_norm < 1e-12:
            continue
        side_n = side_n / side_n_norm

        # Ensure outward (point away from polygon center)
        mid = 0.5 * (planar_vertices[i] + planar_vertices[j])
        if np.dot(side_n, mid - poly_center) < 0:
            side_n = -side_n

        v0 = add(bottom[i], side_n)  # bottom i
        v1 = add(bottom[j], side_n)  # bottom j
        v2 = add(top[j],    side_n)  # top j
        v3 = add(top[i],    side_n)  # top i

        # CCW winding when viewed from outside
        vis_indices += [v0, v1, v2, v0, v2, v3]

    return vis_vertices, vis_indices, vis_normals


def create_extruded_geometry(vertices_per_tile, brick_thickness):
    
    planar_vertices = np.array(vertices_per_tile)
    
    center = np.mean(planar_vertices, axis=0)

    vec1 = planar_vertices[1] - planar_vertices[0]
    vec2 = planar_vertices[2] - planar_vertices[0]
    normal = np.cross(vec1, vec2)
    norm_val = np.linalg.norm(normal)
    if norm_val == 0:
        print("Warning: Degenerate polygon. Using default normal [0,0,1].")
        normal = np.array([0.0, 0.0, 1.0])
    else:
        normal = normal / norm_val

    top_vertices = planar_vertices + brick_thickness * normal
    bottom_vertices = planar_vertices.copy()

    center = center + 0.5 * brick_thickness * normal  # center of the solid

    # Physics/constraints vertices (keep your original structure: bottom then top)
    local_verts = [(v - center).tolist() for v in bottom_vertices] + \
                  [(v - center).tolist() for v in top_vertices]

    # Visual mesh with proper normals
    vis_vertices, visual_indices, vis_normals = build_extruded_visual_mesh(
        planar_vertices, normal, brick_thickness, center
    )

    return (local_verts, visual_indices, center.tolist(), vis_normals, vis_vertices)

def create_brick_body(col_verts, visual_indices, center, vis_normals, vis_vertices, mass=1.0):
    r = 0.8 + np.random.uniform(-0.1, 0.1)
    g = 0.4 + np.random.uniform(-0.1, 0.1)
    b = 0.2 + np.random.uniform(-0.1, 0.1)
    brick_color = [min(1, r), min(1, g), min(1, b), 1]

    vis_shape = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        vertices=vis_vertices,       # use duplicated vertices
        indices=visual_indices,
        normals=vis_normals,         # correct per-face normals
        rgbaColor=brick_color,
        specularColor=[0.1, 0.1, 0.1]  # reduce specular if you want less glare
    )

    col_shape = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        vertices=col_verts  # convex hull from physics verts (bottom+top)
    )

    body_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col_shape,
        baseVisualShapeIndex=vis_shape,
        basePosition=center
    )
    return body_id


def create_constraints_between_bricks(bricks, constraints_with_types, local_verts_list,
                                      filter_collision=True):
    """Create constraints between bricks based on vertex connections and type.
    
    Args:
        bricks: List of brick body IDs
        constraints_with_types: List of constraints (f_i, v_j, f_p, v_q, type)
        local_verts_list: List of local vertex coordinates for each brick
        filter_collision: If True, disable collision between connected pairs
        
    Returns:
       list: created_constraint_ids
    """
    created_constraints = []
    
    for f_i, v_j, f_p, v_q, constraint_type in constraints_with_types:
        # Calculate number of bottom vertices for validation
        num_1 = len(local_verts_list[f_i]) // 2
        num_2 = len(local_verts_list[f_p]) // 2
        
        if constraint_type == 1: # Bottom point connection
            # Bottom vertices are first half of local_verts
            pivot_in_1 = local_verts_list[f_i][v_j]
            pivot_in_2 = local_verts_list[f_p][v_q]
            
            c_id = p.createConstraint(
                bricks[f_i], -1, bricks[f_p], -1, 
                p.JOINT_POINT2POINT, [0, 0, 0], 
                pivot_in_1, pivot_in_2
            )
            created_constraints.append(c_id)
            
        elif constraint_type == 2: # Top point connection
            # Top vertices are second half of local_verts
            pivot_in_1 = local_verts_list[f_i][num_1 + v_j]
            pivot_in_2 = local_verts_list[f_p][num_2 + v_q]
            
            c_id = p.createConstraint(
                bricks[f_i], -1, bricks[f_p], -1, 
                p.JOINT_POINT2POINT, [0, 0, 0], 
                pivot_in_1, pivot_in_2
            )
            created_constraints.append(c_id)
            
        elif constraint_type == 3: # Both top AND bottom connection
            # Bottom connection
            pivot_bot_1 = local_verts_list[f_i][v_j]
            pivot_bot_2 = local_verts_list[f_p][v_q]
            c_bot = p.createConstraint(
                bricks[f_i], -1, bricks[f_p], -1, 
                p.JOINT_POINT2POINT, [0, 0, 0], 
                pivot_bot_1, pivot_bot_2
            )
            created_constraints.append(c_bot)
            
            # Top connection
            pivot_top_1 = local_verts_list[f_i][num_1 + v_j]
            pivot_top_2 = local_verts_list[f_p][num_2 + v_q]
            c_top = p.createConstraint(
                bricks[f_i], -1, bricks[f_p], -1, 
                p.JOINT_POINT2POINT, [0, 0, 0], 
                pivot_top_1, pivot_top_2
            )
            created_constraints.append(c_top)
        
        # Disable collision between connected tiles if requested
        if filter_collision:
            p.setCollisionFilterPair(bricks[f_i], bricks[f_p], -1, -1, 0)
            
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

def compute_min_z(vertices):
        """
        Return the minimal z from a List[List[List[float]]]
        """
        if not vertices:
            return None
        z = []
        for tile in vertices:
            for v in tile:
                z.append(v[2])
        return min(z)


# ---------------------------------------------------------------------------
#  Auto-detection of connection type (top vs bottom face)
# ---------------------------------------------------------------------------

def compute_face_normal(face_vertices):
    """Compute the unit normal of a planar polygon from its vertices.

    Uses Newell's method for robustness with non-planar or nearly-planar
    polygons, falling back to the cross product of the first two edges.

    Args:
        face_vertices: list of [[x,y,z], ...] in CCW or CW order.

    Returns:
        np.array([nx, ny, nz]): unit normal vector.
    """
    verts = np.array(face_vertices, dtype=float)
    n = len(verts)
    if n < 3:
        return np.array([0.0, 0.0, 1.0])

    # Newell's method
    normal = np.zeros(3)
    for i in range(n):
        j = (i + 1) % n
        normal[0] += (verts[i][1] - verts[j][1]) * (verts[i][2] + verts[j][2])
        normal[1] += (verts[i][2] - verts[j][2]) * (verts[i][0] + verts[j][0])
        normal[2] += (verts[i][0] - verts[j][0]) * (verts[i][1] + verts[j][1])

    norm = np.linalg.norm(normal)
    if norm < 1e-12:
        # fallback: cross product of first two edges
        e1 = verts[1] - verts[0]
        e2 = verts[2] - verts[0]
        normal = np.cross(e1, e2)
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            return np.array([0.0, 0.0, 1.0])
    return normal / norm


def is_planar_configuration(vertices, tol=1e-9):
    """Check whether ALL vertex z-coordinates are (nearly) zero.

    Args:
        vertices: List[List[List[float]]] – one entry per tile.
        tol: tolerance for comparing to 0.

    Returns:
        bool: True if every vertex has |z| < tol.
    """
    for tile in vertices:
        for v in tile:
            if abs(v[2]) > tol:
                return False
    return True


def find_vertices_at_same_position(target_vertices, f_a, f_b, tol=1e-6):
    """Find pairs of vertex indices (i in f_a, j in f_b) that coincide in target.

    Returns:
        list of (idx_a, idx_b) tuples where the vertices are at the same position.
    """
    verts_a = np.array(target_vertices[f_a], dtype=float)
    verts_b = np.array(target_vertices[f_b], dtype=float)
    pairs = []
    for ia in range(len(verts_a)):
        for ib in range(len(verts_b)):
            if np.linalg.norm(verts_a[ia] - verts_b[ib]) < tol:
                pairs.append((ia, ib))
    return pairs


def do_faces_share_edge(target_vertices, f_a, f_b, tol=1e-6):
    """Check if two faces share an edge in the target configuration.

    Two faces share an edge if there exist two pairs of coincident vertices
    that are adjacent in BOTH faces.

    Returns:
        tuple: (shared_edge_a, shared_edge_b) or (None, None)
               shared_edge_a = (idx1_a, idx2_a) – the edge in face A
               shared_edge_b = (idx1_b, idx2_b) – the edge in face B
    """
    coincident = find_vertices_at_same_position(target_vertices, f_a, f_b, tol)
    if len(coincident) < 2:
        return None, None

    n_a = len(target_vertices[f_a])
    n_b = len(target_vertices[f_b])

    # Check all pairs of coincident vertices for adjacency in both faces
    for i in range(len(coincident)):
        for j in range(i + 1, len(coincident)):
            ia1, ib1 = coincident[i]
            ia2, ib2 = coincident[j]
            # Check adjacency in face A (modulo n_a)
            if (ia2 - ia1) % n_a == 1 or (ia1 - ia2) % n_a == 1:
                # Check adjacency in face B
                if (ib2 - ib1) % n_b == 1 or (ib1 - ib2) % n_b == 1:
                    return (ia1, ia2), (ib1, ib2)
    return None, None


def compute_dihedral_angle(target_vertices, f_a, f_b, edge_a, edge_b):
    """Compute the signed dihedral angle between two faces sharing an edge.

    The dihedral angle is measured such that:
      - δ ≈ π means the faces are coplanar (flat)
      - δ < π means the faces fold toward each other (valley from the
        perspective of the bottom face normals)
      - δ > π means the faces fold away from each other (mountain)

    Args:
        target_vertices: List[List[List[float]]].
        f_a, f_b: face indices.
        edge_a: (idx_a1, idx_a2) – edge indices in face A.
        edge_b: (idx_b1, idx_b2) – edge indices in face B.

    Returns:
        float: dihedral angle in radians, in (0, 2π).
    """
    n_a = compute_face_normal(target_vertices[f_a])
    n_b = compute_face_normal(target_vertices[f_b])

    # Edge direction (use face A's edge, but face B's edge is parallel)
    v_a1 = np.array(target_vertices[f_a][edge_a[0]], dtype=float)
    v_a2 = np.array(target_vertices[f_a][edge_a[1]], dtype=float)
    edge_dir = v_a2 - v_a1
    edge_norm = np.linalg.norm(edge_dir)
    if edge_norm < 1e-12:
        return np.pi  # degenerate, treat as flat
    edge_dir = edge_dir / edge_norm

    # Signed dihedral: δ = atan2( (n_a × n_b)·edge_dir,  n_a·n_b )
    cross_n = np.cross(n_a, n_b)
    dot_n = np.dot(n_a, n_b)
    delta = np.arctan2(np.dot(cross_n, edge_dir), dot_n)

    # Normalize to (0, 2π)
    if delta < 0:
        delta += 2 * np.pi

    return delta


def auto_determine_connection_types(initial_vertices, target_vertices, constraints,
                                   tol=1e-6):
    """Automatically determine whether each constraint should connect bottom,
    top, or both faces.

    Logic (per constraint [f_a, v_a, f_b, v_b]):
      1. If both initial & target are planar (all z≈0): type 3 (connect BOTH).
      2. If faces share an edge in the target: compute dihedral angle δ.
         δ < π → valley → TOP (type 2);  δ ≥ π → mountain → BOTTOM (type 1).
      3. Otherwise (negative-space, single-vertex connection):
         (a) If face normals point in opposite hemispheres (dot < 0):
             bottom faces face away → BOTTOM (type 1).
         (b) Else: check whether the face normals **converge** toward each
             other (i.e. each normal points roughly toward the other tile's
             centroid).  Converging → valley → TOP; diverging → mountain →
             BOTTOM.
         (c) If neutral (side-by-side tiles): default to BOTTOM (type 1).

    Args:
        initial_vertices: List[List[List[float]]] – initial vertex positions
                          (before deployment).
        target_vertices:  List[List[List[float]]] – target vertex positions
                          (after Kabsch alignment, if applicable).
        constraints: list of [f_a, v_a, f_b, v_b] (4-column, no type yet).
        tol: tolerance for position equality.

    Returns:
        list of [f_a, v_a, f_b, v_b, type]: same constraints with
        determined type (1=bottom, 2=top, 3=both).
    """
    # Check planar target
    planar_initial = is_planar_configuration(initial_vertices, tol=1e-9)
    planar_target = is_planar_configuration(target_vertices, tol=1e-9)

    if planar_initial and planar_target:
        # Planar-to-planar: connect both faces for all constraints
        return [[c[0], c[1], c[2], c[3], 3] for c in constraints]

    # Pre-compute centroids and face normals for all tiles (used in case 3)
    num_tiles = len(target_vertices)
    face_normals = [compute_face_normal(target_vertices[i]) for i in range(num_tiles)]
    centroids = [np.mean(target_vertices[i], axis=0) for i in range(num_tiles)]

    # Non-planar: decide per constraint
    result = []
    for c in constraints:
        f_a, v_a, f_b, v_b = c[0], c[1], c[2], c[3]

        # Check if faces share an edge
        edge_a, edge_b = do_faces_share_edge(target_vertices, f_a, f_b, tol)

        if edge_a is not None and edge_b is not None:
            # ---- Reconfigurable case: shared edge → use dihedral angle ----
            delta = compute_dihedral_angle(target_vertices, f_a, f_b, edge_a, edge_b)

            # If δ < π, the faces "close toward each other" (valley fold
            # from the bottom perspective) → TOP connection.
            # If δ >= π (mountain fold) → BOTTOM connection.
            if delta < np.pi - 1e-9:
                conn_type = 2  # valley → top
            else:
                conn_type = 1  # mountain → bottom
        else:
            # ---- Negative-space case: single vertex ----
            n_a = face_normals[f_a]
            n_b = face_normals[f_b]
            dot_n = np.dot(n_a, n_b)

            if dot_n < 0:
                # Normals point in opposite hemispheres → bottom faces
                # face away from each other → BOTTOM connection is safe.
                conn_type = 1  # bottom
            else:
                # Normals are in the same hemisphere.  Check convergence
                # by projecting each normal onto the inter-centroid vector.
                c_a = centroids[f_a]
                c_b = centroids[f_b]
                d_ab = c_b - c_a
                d_norm = np.linalg.norm(d_ab)
                if d_norm > 1e-12:
                    d_ab = d_ab / d_norm
                    proj_a = np.dot(n_a, d_ab)       # > 0 → n_a points toward B
                    proj_b = np.dot(n_b, -d_ab)       # > 0 → n_b points toward A
                else:
                    proj_a = proj_b = 0.0

                # Threshold: normals must have a meaningful component
                # toward the other tile to count as "converging".
                thresh = 1e-3
                if proj_a > thresh and proj_b > thresh:
                    # Normals converge → valley → TOP
                    conn_type = 2  # top
                elif proj_a < -thresh and proj_b < -thresh:
                    # Normals diverge → mountain → BOTTOM
                    conn_type = 1  # bottom
                else:
                    # Neutral / side-by-side tiles → default to BOTTOM.
                    # print(f"Warning: Neutral configuration for constraint {(f_a, v_a, f_b, v_b)}. Defaulting to BOTTOM connection.")
                    conn_type = 1  # bottom

        result.append([f_a, v_a, f_b, v_b, conn_type])

    return result


# ---------------------------------------------------------------------------
#  Kabsch alignment: align target vertices to initial vertices
# ---------------------------------------------------------------------------

def align_target_to_initial(initial_vertices, target_vertices):
    """Align target vertices to initial vertices using the Kabsch algorithm.

    Finds the optimal rigid transformation (rotation + translation) that
    minimizes the RMSD between the two point sets.  This prevents tiles
    from clashing when the target is a mirrored / rotated version of the
    initial configuration.

    Args:
        initial_vertices: List[List[List[float]]] – one entry per tile,
                          each tile is a list of [x,y,z] triples.
        target_vertices:  Same structure, one-to-one correspondence.

    Returns:
        tuple: (aligned_target, R, t, centroid_Q)
            aligned_target: target transformed to match initial.
            R (3×3 np.array): optimal rotation matrix.
            t (np.array): translation vector (aligned = R @ Q + t).
            centroid_Q (np.array): centroid of original target.
    """
    # Flatten both structures into (N, 3) arrays, remembering the
    # per-tile vertex counts so we can reconstruct the shape later.
    init_flat = []
    targ_flat = []
    tile_lengths = []

    for tile_init, tile_targ in zip(initial_vertices, target_vertices):
        n = len(tile_init)
        if n != len(tile_targ):
            raise ValueError("Mismatched vertex count per tile – "
                             "initial and target must be one-to-one.")
        tile_lengths.append(n)
        init_flat.extend(tile_init)
        targ_flat.extend(tile_targ)

    P = np.array(init_flat, dtype=float)  # (N, 3) initial
    Q = np.array(targ_flat, dtype=float)  # (N, 3) target

    if len(P) < 3:
        # Too few points for a meaningful alignment – return as-is
        print("Warning: fewer than 3 total vertices, skipping Kabsch alignment.")
        return target_vertices

    # 1. Center both point clouds
    centroid_P = np.mean(P, axis=0)
    centroid_Q = np.mean(Q, axis=0)
    P_centered = P - centroid_P
    Q_centered = Q - centroid_Q

    # 2. Covariance matrix H = P_centered^T @ Q_centered
    H = P_centered.T @ Q_centered

    # 3. SVD
    U, S, Vt = np.linalg.svd(H)
    V = Vt.T  # Vt is V^T, so V = Vt^T

    # 4. Optimal rotation — ensure U and V are proper rotations (det = +1)
    #    For rank-deficient data (e.g. planar points), the SVD may return
    #    U or V with determinant −1.  Flipping the column corresponding to
    #    the smallest singular value corrects this without changing the
    #    decomposition (since that singular value is ≈ 0 for planar data).
    if np.linalg.det(U) < 0:
        U[:, -1] *= -1
    if np.linalg.det(V) < 0:
        V[:, -1] *= -1

    # The Kabsch objective is to maximise tr(R @ H^T), which yields
    # R = U @ V^T  (NOT V @ U^T, which would maximise tr(R @ H)).
    R = U @ V.T

    # Final safeguard against reflections (should not trigger after the above)
    if np.linalg.det(R) < 0:
        V[:, -1] *= -1
        R = U @ V.T

    # 5. Translation
    t = centroid_P - R @ centroid_Q

    # 6. Apply transformation to each target vertex
    aligned_flat = (R @ Q.T).T + t  # (N, 3)

    # 7. Reconstruct the nested list-of-lists structure
    aligned_target = []
    idx = 0
    for n in tile_lengths:
        tile = aligned_flat[idx:idx + n].tolist()
        aligned_target.append(tile)
        idx += n

    # Compute RMSD for diagnostics
    residuals = P - aligned_flat
    rmsd = np.sqrt(np.mean(np.sum(residuals ** 2, axis=1)))
    print(f"Kabsch alignment: RMSD = {rmsd:.6f}")

    return R, t, centroid_Q