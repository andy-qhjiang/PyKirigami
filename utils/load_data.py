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
            constraint = list(map(int, line.strip().split()))
            if len(constraint) == 4:
                # Convert from 1-based to 0-based indexing
                constraints.append([x-1 for x in constraint])
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

