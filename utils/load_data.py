"""
Data loading utilities for the kirigami simulation project.
"""
import os
import numpy as np

def load_vertices_from_file(filename):
    """
    Load 3D vertex data from file.
    
    Args:
        filename: Path to the file containing 3D vertex data with 12 values per line
                 (x,y,z for 4 vertices). For 2D data, users should preprocess files
                 by adding z=0 to each point.
        
    Returns:
        numpy.ndarray: Array of vertex coordinates.
    """
    vertices = []
    with open(filename, 'r') as file:
        for line in file:
            coords = list(map(float, line.strip().split()))
            if len(coords) == 12:  # x, y, z for 4 vertices
                vertices.append(coords)
            else:
                print(f"Warning: Skipping line with {len(coords)} values instead of 12")
    return np.array(vertices)

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
                # Convert from 1-based to 0-based indexing
                # Default type 0 if not specified (e.g. hinge using 'both' logic)
                constraints.append([parts[0]-1, parts[1]-1, parts[2]-1, parts[3]-1, 0]) 
            elif len(parts) == 5: # New format: f_i, v_j, f_p, v_q, type
                # Convert from 1-based to 0-based indexing for f_i, v_j, f_p, v_q
                constraints.append([parts[0]-1, parts[1]-1, parts[2]-1, parts[3]-1, parts[4]])
            else:
                print(f"Warning: Skipping constraint line with {len(parts)} values. Expected 4 or 5.")
    return constraints

def load_hull_tiles(filename):
    """
    Load hull tiles from file.
    
    Args:
        filename: Path to the file containing hull tile data.
        
    Returns:
        list: List of hull tile indices.
    """
    hull_tiles = set()
    with open(filename, 'r') as file:
        for line in file:
            tile_ind, _ = map(int, line.strip().split())
            hull_tiles.add(tile_ind - 1)  # Convert to 0-based indexing
    return list(hull_tiles)

