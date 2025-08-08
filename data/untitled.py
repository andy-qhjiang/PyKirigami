# this is a python script to proceed data from some text file and the data is not matrix

def scale_vertices_file(input_file, output_file=None, scale_factor=2.0):
    """
    Scale 2D vertices to 3D with scaling factor.
    
    Args:
        input_file: Path to input vertices file
        output_file: Path to output file (optional, defaults to input_file with '_scaled' suffix)
        scale_factor: Scaling factor for x,y coordinates (default: 2.0)
    """
    if output_file is None:
        # Generate output filename automatically
        if input_file.endswith('.txt'):
            output_file = input_file.replace('.txt', '_scaled.txt')
        else:
            output_file = input_file + '_scaled'
    
    with open(input_file) as f:
        vertices = []
        for line in f:
            # process each line
            vertices.append(list(map(float, line.strip().split())))

        for i in range(len(vertices)):
            coords = []
            for j in range(0, len(vertices[i]), 2):
                a, b = vertices[i][j:j+2]
                a, b = scale_factor * a, scale_factor * b
                coords.append(a)
                coords.append(b)
                coords.append(0)
            vertices[i] = coords

    # write the list into a new file
    with open(output_file, 'w') as f:
        for line in vertices:
            f.write(' '.join(map(str, line)) + '\n')
    
    print(f"Scaled vertices written to: {output_file}")

# Example usage:
if __name__ == "__main__":
    # Basic usage - scales stampfli24_vertices.txt by factor of 2
    scale_vertices_file('stampfli24_vertices.txt')
    
    # Custom output file
    scale_vertices_file('stampfli24_vertices.txt', 'my_custom_output.txt')
    
    # Different scaling factor
    scale_vertices_file('stampfli24_vertices.txt', scale_factor=1.5)
    
    # Process different file
    scale_vertices_file('other_vertices.txt', scale_factor=3.0)