"""
Data loading utilities for the kirigami simulation project.
"""
import numpy as np

def load_vertices_from_file(filename):
    """
    Load 3D vertex data from file.
    
    Args:
        filename: Path to the file containing 3D vertex data with n*3 values per line
                 (x,y,z for n vertices). Supports both triangles (9 values) and 
                 quadrilaterals (12 values). For 2D data, users should preprocess 
                 files by adding z=0 to each point.
        
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
            if len(parts) == 5: # Old format: f_i, v_j, f_p, v_q
                constraints.append([parts[0]-1, parts[1]-1, parts[2]-1, parts[3]-1, parts[4]])
            else:
                print(f"Warning: Skipping constraint line with {len(parts)} values. Expected 4 or 5.")
    return constraints

def load_force_bricks(filename):
    """
    Load specific bricks to apply forces to from a file.
    
    Args:
        filename: Path to the file containing force brick index.
        
    Returns:
        list: List of brick IDs to apply forces to.
    """
    force_bricks = []
    with open(filename, 'r') as file:
        for line in file:
            parts = list(map(int, line.strip().split()))
            if len(parts) == 1:  # Expecting a single brick ID per line
                force_bricks.append(parts[0] - 1)  # Convert to zero-based index
            else:
                print(f"Warning: Skipping line with {len(parts)} values. Expected 1.")
    return force_bricks
