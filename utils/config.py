"""
Handles command-line arguments, file loading, and data validation.
"""
import argparse
import numpy as np
import sys

def load_vertices_from_file(filename):
    """
    Load 3D vertex data from file.
    
    Args:
        filename: Path to the file containing 3D vertex data with n*3 values per line
                 (x,y,z for n vertices). Supports both triangles (9 values) and 
                 quadrilaterals (12 values). For 2D data, the script will add z=0 to each point.

    Returns:
        list: List of lists, where each sublist contains vertex coordinates for one shape.
    """
    vertices = []
    with open(filename, 'r') as file:
        for line in file:
            coords = list(map(float, line.strip().split()))
            if len(coords) % 3 == 0 and len(coords) >= 9:  # At least 3 vertices (triangle)
                vertices.append(coords)
            else:
                print(f"Warning: Skipping line with {len(coords)} values. Expected multiple of 3 with minimum 9 values.")
    return vertices

def load_constraints_from_file(filename):
    """
    Load constraints from file.
    
    Args:
        filename: Path to the file containing constraint data.
        
    Returns:
        list: List of constraints.
    """
    constraints = []
    with open(filename, 'r') as file:
        for line in file:
            parts = list(map(int, line.strip().split()))
            if len(parts) == 4: # Old format: f_i, v_j, f_p, v_q
                constraints.append([parts[0]-1, parts[1]-1, parts[2]-1, parts[3]-1, 1])
            elif len(parts) == 5: # New format: f_i, v_j, f_p, v_q, pa1
                constraints.append([parts[0]-1, parts[1]-1, parts[2]-1, parts[3]-1, parts[4]])
            else:
                print(f"Warning: Skipping constraint line with {len(parts)} values. Expected 4 or 5.")
    return constraints

def validate_constraints(vertices, constraints, max_distance=0.1):
    """
    Validate that constraints in target vertices don't have excessive distances.
    
    This function checks if constraint endpoints in the target configuration
    are close enough together to avoid physics instabilities.
    
    Args:
        vertices: List of vertex arrays for each brick
        constraints: List of constraint definitions [face1, vertex1, face2, vertex2, ...]
        max_distance: Maximum allowed distance between constraint endpoints (default: 0.1)
        
    Raises:
        SystemExit: If any constraint has distance > max_distance
        
    Returns:
        None: Function either passes validation or exits the program
    """
    if vertices is None:
        return
        
    for i, constraint in enumerate(constraints):
        f1, v1, f2, v2 = constraint[:4]
        
        # Get vertex positions for constraint endpoints
        pos1 = np.array(vertices[f1][3*v1:3*v1+3])
        pos2 = np.array(vertices[f2][3*v2:3*v2+3])
        
        # Calculate distance using numpy
        dist = np.linalg.norm(pos1 - pos2)
        
        if dist > max_distance:
            error_msg = f"ERROR: Constraint {i} between faces {f1+1} and {f2+1} with vertices {v1+1} and {v2+1} has a large distance ({dist:.3f}) in target vertices."
            print(error_msg)
            print(f"This exceeds the maximum allowed distance of {max_distance:.3f} and will lead to unexpected behavior.")
            print("Please check your target vertices file.")
            sys.exit(1)

def parse_arguments():
    """Parse command-line arguments for the simulation"""
    parser = argparse.ArgumentParser(description='Run kirigami simulation')
    
    # Input files
    parser.add_argument('--vertices_file', required=True, help='File containing vertex data')
    parser.add_argument('--constraints_file', required=True, help='File with connectivity constraints')
    parser.add_argument('--target_vertices_file', help='File containing target vertex positions for deployment (optional)')
    
    # Physics simulation parameters
    parser.add_argument('--gravity', type=float, default=0, help='Gravity constant')
    parser.add_argument('--timestep', type=float, default=1/240, help='Physics simulation timestep')
    parser.add_argument('--substeps', type=int, default=20, help='Physics substeps per step')
    
    
    # Target-based deployment parameters
    parser.add_argument('--target_stiffness', type=float, default=500.0,
                       help='Stiffness coefficient for target-based forces')
    parser.add_argument('--target_damping', type=float, default=50.0,
                       help='Damping coefficient for target-based forces')
    
    
    
    # Geometry parameters
    parser.add_argument('--brick_thickness', type=float, default=0.02,
                       help='Thickness of the brick (z-height)')
    parser.add_argument('--linear_damping', type=float, default=1, help='Linear damping')
    parser.add_argument('--angular_damping', type=float, default=1, help='Angular damping')
    
    
  
    # Visual options
    parser.add_argument('--ground_plane', action='store_true',
                       help='Add a ground plane to the simulation')
    parser.add_argument('--camera_distance', type=float, default=8.0,
                       help='Distance of the camera from the origin')
    
    return parser.parse_args()
