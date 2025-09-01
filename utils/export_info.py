import numpy as np
import pybullet as p

 
def export_obj_brick(file_path, bricks, visual_mesh):
    """
    Export OBJ based on visual mesh constructed.
    
    """

    obj_path = file_path + ".obj"
    v_offset = 1
    vn_offset = 1
    with open(obj_path, "w") as obj:
        for i, b in enumerate(bricks):
            name = f"Tile_{i}"
            obj.write(f"o {name}\n")

            pos, orn = p.getBasePositionAndOrientation(b)
            R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            O = np.asarray(pos, dtype=float)

            V_local = visual_mesh[i]['vertices_local']
            N_local = visual_mesh[i]['normals_local']
            I = visual_mesh[i]['indices']

            for v in V_local:
                vw = R @ np.asarray(v) + O
                obj.write(f"v {vw[0]:.8f} {vw[1]:.8f} {vw[2]:.8f}\n")

            for n in N_local:
                nw = R @ np.asarray(n)
                norm = np.linalg.norm(nw)
                if norm > 1e-12:
                    nw = nw / norm
                obj.write(f"vn {nw[0]:.6f} {nw[1]:.6f} {nw[2]:.6f}\n")

            for t in range(0, len(I), 3):
                i0, i1, i2 = I[t:t+3]
                obj.write(
                    f"f {i0 + v_offset}//{i0 + vn_offset} "
                    f"{i1 + v_offset}//{i1 + vn_offset} "
                    f"{i2 + v_offset}//{i2 + vn_offset}\n"
                )

            v_offset += len(V_local)
            vn_offset += len(N_local)

    print(f"Wrote {obj_path}")


def export_obj_bottom(file_path, bricks, local_bottom_vertices):

    """
    Export OBJ file drawing planar bottom face in the format:
    # n vertices m faces
    v x y z
    v x y z
    ...
    f v1 v2 v3 ...
    ...
    No object/group lines which is different from standard OBJ files.
    """
    vertices = []
    faces = []

    # Build global vertex and face lists
    for i, b in enumerate(bricks):
        pos, orn = p.getBasePositionAndOrientation(b)
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        O = np.asarray(pos, dtype=float)

        bottom_vertices_per_tile = local_bottom_vertices[i]
        if not bottom_vertices_per_tile or len(bottom_vertices_per_tile) < 3:
            continue

        base = len(vertices) + 1  # OBJ is 1-based
        # transform and append vertices
        for v in bottom_vertices_per_tile:
            vw = R @ np.asarray(v) + O
            vertices.append(vw)

        n = len(bottom_vertices_per_tile)
        faces.append(list(range(base, base + n)))

    # Write file: header, then v, then f
    with open(file_path, "w") as obj:
        # include header
        obj.write(f"# {len(vertices)} vertices {len(faces)} faces\n")
        
        for v in vertices:
            obj.write(f"v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}\n")
        obj.write("\n")
        
        for f in faces:
            obj.write("f " + " ".join(map(str, f)) + "\n")

    print(f"Wrote planar OBJ: {file_path}")