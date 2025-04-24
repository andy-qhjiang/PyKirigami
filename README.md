# Kirigami 3D Simulation Framework

A modular framework for simulating kirigami structures in 3D using PyBullet physics engine. This project enables different types of kirigami simulations including 2D-to-3D transformations, 3D-to-3D transformations, and outward deformations.

## Overview

Kirigami is a variation of origami that includes cutting of the paper, rather than solely folding. This framework simulates the behavior of kirigami structures under various types of forces, providing insights into their mechanical properties and transformation capabilities.

The simulation creates 3D tiles connected by constraints at their vertices, representing the hinges of a kirigami structure. Various forces can be applied to cause the structure to transform from one state to another.

## Project Structure

```
kirigami_sim3D/
├── data/                  # Data files for simulation (vertices, constraints, hull definitions)
├── examples/              # Example simulation scripts
│   ├── demo_2Dto3D_modular.py
│   ├── demo_3Dto3D_modular.py
│   ├── demo_outward_forces_modular.py
│   └── unified_kirigami_sim.py
├── simulation/            # Core simulation modules
│   ├── geometry.py        # Geometry creation and manipulation
│   └── physics.py         # Physics engine setup and simulation
└── utils/                 # Utility modules
    ├── load_data.py       # Data loading utilities
    └── visualization.py   # Visualization utilities
```

## Simulation Types

The framework supports three main simulation types:

1. **2D to 3D Transformation**: Apply vertical forces to selected tiles to transform a flat structure into a 3D shape.
2. **3D to 3D Transformation**: Apply forces along tile normals to transform one 3D shape into another (e.g., cube to sphere).
3. **Outward Forces**: Apply outward forces from the structure's center to cause expansion.

## Getting Started

### Prerequisites

- Python 3.6+
- PyBullet
- NumPy

### Installation

```bash
pip install pybullet numpy
```

### Running Simulations

You can run simulations using the `unified_kirigami_sim.py` script with different parameters:

#### Using data presets:

```bash
python examples/unified_kirigami_sim.py --sim_type 2d_to_3d --data_preset rigid_3by3 --force_magnitude 500 --gravity -20 --ground_plane --keep_open
```

#### Using explicit file paths:

```bash
python examples/unified_kirigami_sim.py --sim_type 3d_to_3d --vertices_file cube2sphere_contracted_vertices.txt --constraints_file cube2sphere_constraints.txt --force_magnitude 1000 --gravity 0 --rotating_camera --keep_open
```

## Command-Line Parameters

### Simulation Type
- `--sim_type`: Type of simulation ('2d_to_3d', '3d_to_3d', 'outward')
- `--data_preset`: Use predefined data sets ('rigid_3by3', 'cube2sphere', 'quad_partial_sphere', 'tessellation')

### File Paths
- `--vertices_file`: Path to the vertices file
- `--constraints_file`: Path to the constraints file
- `--hull_file`: Path to the hull file (optional)

### Physics Parameters
- `--gravity`: Gravity in z-direction (default: -20)
- `--brick_height`: Height of bricks for 2D simulations (default: 0.02)
- `--brick_thickness`: Thickness of bricks for 3D simulations (default: 0.01)
- `--force_magnitude`: Magnitude of applied forces (default: 500)
- `--force_tiles`: Indices of tiles to apply forces to
- `--linear_damping`: Linear damping for bodies (default: 25)
- `--angular_damping`: Angular damping for bodies (default: 25)

### Simulation Control
- `--timestep`: Physics timestep (default: 1/240)
- `--substeps`: Physics substeps (default: 10)
- `--sim_steps`: Number of simulation steps (default: 2400)
- `--ground_plane`: Add ground plane
- `--keep_open`: Keep window open after simulation

### Camera Control
- `--camera_distance`: Camera distance from target (default: 6.0)
- `--camera_pitch`: Camera pitch angle (default: -15)
- `--rotating_camera`: Use rotating camera (for 3D-to-3D)

## Examples

### 2D to 3D Transformation
```bash
python examples/unified_kirigami_sim.py --sim_type 2d_to_3d --data_preset rigid_3by3 --force_magnitude 500 --gravity -20 --ground_plane --keep_open
```

### 3D to 3D Transformation (Cube to Sphere)
```bash
python examples/unified_kirigami_sim.py --sim_type 3d_to_3d --data_preset cube2sphere --force_magnitude 1000 --gravity 0 --rotating_camera --keep_open
```

### Outward Forces
```bash
python examples/unified_kirigami_sim.py --sim_type outward --data_preset rigid_3by3 --force_magnitude 200 --brick_height 0.2 --gravity -9.81 --ground_plane --keep_open
```

## Data Files

The framework uses three types of data files:

1. **Vertices Files**: Defines the geometry of each tile
   - 2D format: 8 values per line (x,y coordinates of 4 vertices)
   - 3D format: 12 values per line (x,y,z coordinates of 4 vertices)

2. **Constraints Files**: Defines connections between tiles
   - Format: 4 values per line (face_i, vertex_j, face_p, vertex_q)
   - Each constraint connects vertex j on tile i to vertex q on tile p

3. **Hull Files** (optional): Defines which tiles are on the hull (exterior)
   - Format: 2 values per line (tile_index, flag)

## Extending the Framework

New simulation types can be added by:

1. Creating a new setup function in `unified_kirigami_sim.py`
2. Adding a new force application function
3. Adding the new simulation type to the command-line options

## License

This project is provided for academic and educational purposes.

## Acknowledgments

This framework is designed for research in computational design and simulation of kirigami structures.

## Interactive Controls (PyBullet GUI Sliders)

After starting the simulation with `run_sim.py`, the PyBullet OpenGL GUI will display an interactive parameter panel on the right. Use the following sliders to control the running simulation in real time:

- **Reset simulation**: Drag above 0.5 to tear down and reinitialize the entire kirigami model from the original input files.
- **Save vertices**: Drag above 0.5 to write the current top‑face vertex coordinates (12 values per face) to a timestamped file in the `output/` directory.
- **Delete tile index**: Range from –1 (no deletion) to N–1 (maximum tile index). Set to a valid 0‑based tile index to immediately remove that tile and all its constraints; after deletion, you can return the slider to –1 or choose another index.

These GUI sliders replace keyboard shortcuts — there is no need to press keys or click in the window. Simply move a slider past its threshold to invoke the corresponding action without restarting the script.