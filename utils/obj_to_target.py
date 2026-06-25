"""
Convert a PyKirigami-exported OBJ file back to target.txt format.

Reads the .obj file exported by export_obj_bottom() and reconstructs
the per-tile vertex list in the same format as vertices.txt / target.txt:

    x1 y1 z1 x2 y2 z2 x3 y3 z3 ...    (one line per tile)

The OBJ face order matches tile order (face i = tile i-1), and vertex
coordinates are already in world space — no transformation needed.

Usage:
    python obj_to_target.py <input.obj> [output.txt]

    If output is omitted, writes to stdout.
"""
import sys


def parse_obj(path):
    """Read an OBJ file, return (vertices, faces).

    vertices: list of (x, y, z) tuples, 0-indexed
    faces:    list of (i1, i2, i3, ...) tuples, 0-indexed vertex indices
    """
    vertices = []
    faces = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if parts[0] == "v":
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif parts[0] == "f":
                # OBJ face indices are 1-based; may be "f i j k" or "f i//t j//t"
                idxs = [int(p.split("/")[0]) - 1 for p in parts[1:]]
                faces.append(idxs)
    return vertices, faces


def obj_to_target(obj_path):
    """Convert OBJ to target.txt lines."""
    vertices, faces = parse_obj(obj_path)
    lines = []
    for face in faces:
        coords = []
        for idx in face:
            x, y, z = vertices[idx]
            coords.append(f"{x:.6f} {y:.6f} {z:.6f}")
        lines.append(" ".join(coords))
    return lines


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    obj_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else None

    lines = obj_to_target(obj_path)

    if out_path:
        with open(out_path, "w") as f:
            for line in lines:
                f.write(line + "\n")
        print(f"Converted {len(lines)} tiles → {out_path}")
    else:
        for line in lines:
            print(line)


if __name__ == "__main__":
    main()
