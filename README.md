# PyKirigami: An Interactive Python Simulator for Kirigami Metamaterials

<div align="center">
  <img src="./gallery/partialSphere_demo.gif" alt="Partial Sphere Demo" width="45%" />
  <img src="./gallery/cylinder_demo.gif" alt="Cylinder Demo" width="45%" />
</div>

## Table of Contents

1. [Overview](#overview)
2. [Installation](#installation)
3. [Quick Start](#quick-start)
4. [Input Data Format](#input-data-format)
   - 4.1 [Vertices File Format](#vertices-file-format)
   - 4.2 [Constraints File Format](#constraints-file-format)
   - 4.3 [Force Bricks File Format (Optional)](#force-bricks-file-format-optional)
5. [Command-Line Arguments](#command-line-arguments)
   - 5.1 [Required Arguments](#required-arguments)
   - 5.2 [Physics Parameters](#physics-parameters)
   - 5.3 [Expansion Parameters](#expansion-parameters)
   - 5.4 [Geometry Parameters](#geometry-parameters)
   - 5.5 [Damping Parameters](#damping-parameters)
   - 5.6 [Visual Options](#visual-options)
6. [3D Tile Geometry and Normal Vector Orientation](#3d-tile-geometry-and-normal-vector-orientation)
   - 6.1 [Understanding Normal Vector Calculation](#understanding-normal-vector-calculation)
   - 6.2 [Vertex Ordering Guidelines](#vertex-ordering-guidelines)
   - 6.3 [Top vs Bottom Face Assignment](#top-vs-bottom-face-assignment)
   - 6.4 [Common Issues and Troubleshooting](#common-issues-and-troubleshooting)
7. [Inter-Tile Connections](#inter-tile-connections)
   - 7.1 [Connection Types](#connection-types)
     - 7.1.1 [Spherical Joints (Type 1 & 2)](#spherical-joints-type-1--2)
     - 7.1.2 [Hinge Connections](#hinge-connections)
     - 7.1.3 [Rigid Connections](#rigid-connections)
   - 7.2 [Constraint File Specification](#constraint-file-specification)
   - 7.3 [Best Practices for Defining Connections](#best-practices-for-defining-connections)
8. [Force Models and Deployment Mechanisms](#force-models-and-deployment-mechanisms)
   - 8.1 [Automatic Expansion Forces](#automatic-expansion-forces)
   - 8.2 [Manual Force Application](#manual-force-application)
   - 8.3 [Environmental Forces](#environmental-forces)
   - 8.4 [Parameter Tuning Guidelines](#parameter-tuning-guidelines)
9. [Interactive Controls](#interactive-controls)
   - 9.1 [Mouse Controls](#mouse-controls)
   - 9.2 [Keyboard Controls](#keyboard-controls)
   - 9.3 [Camera Navigation](#camera-navigation)
   - 9.4 [Tile Manipulation](#tile-manipulation)
10. [Example Projects](#example-projects)
    - 10.1 [2D-to-2D Deployments](#2d-to-2d-deployments)
      - 10.1.1 [Square-to-Circle Transformation](#square-to-circle-transformation)
      - 10.1.2 [Tangram Puzzle](#tangram-puzzle)
    - 10.2 [2D-to-3D Deployments](#2d-to-3d-deployments)
      - 10.2.1 [Fan Structure](#fan-structure)
      - 10.2.2 [Cylinder Formation](#cylinder-formation)
    - 10.3 [3D-to-3D Deployments](#3d-to-3d-deployments)
      - 10.3.1 [Cube-to-Sphere Transformation](#cube-to-sphere-transformation)
      - 10.3.2 [Tubular Structures](#tubular-structures)
11. [Advanced Usage](#advanced-usage)
    - 11.1 [Custom Force Application](#custom-force-application)
    - 11.2 [Simulation State Management](#simulation-state-management)
    - 11.3 [Data Export and Analysis](#data-export-and-analysis)
    - 11.4 [Performance Optimization](#performance-optimization)
12. [Troubleshooting](#troubleshooting)
    - 12.1 [Common Simulation Issues](#common-simulation-issues)
    - 12.2 [File Format Errors](#file-format-errors)
    - 12.3 [Physics Stability Problems](#physics-stability-problems)
    - 12.4 [Visualization Issues](#visualization-issues)
13. [Development and Extension](#development-and-extension)
    - 13.1 [Code Structure](#code-structure)
    - 13.2 [Adding New Features](#adding-new-features)
    - 13.3 [Custom Force Models](#custom-force-models)
    - 13.4 [Integration with Other Tools](#integration-with-other-tools)
14. [Technical Background](#technical-background)
    - 14.1 [PyBullet Physics Engine](#pybullet-physics-engine)
    - 14.2 [Rigid Body Dynamics](#rigid-body-dynamics)
    - 14.3 [Constraint Solving](#constraint-solving)
    - 14.4 [Numerical Considerations](#numerical-considerations)
15. [Frequently Asked Questions (FAQ)](#frequently-asked-questions-faq)
16. [Contributing](#contributing)
17. [Citation](#citation)
18. [License](#license)
19. [Contact](#contact)

---

## Overview

PyKirigami is an open-source Python toolbox for simulating the 2D and 3D deployment of general kirigami metamaterials. The simulator utilizes the PyBullet physics engine and is capable of simulating 2D-to-2D, 2D-to-3D, and 3D-to-3D kirigami deployments with interactive controls.

### Key Features
- **Multi-dimensional deployment simulation**: Support for 2D-to-2D, 2D-to-3D, and 3D-to-3D transformations
- **Physics-based simulation**: Powered by PyBullet for realistic physics behavior
- **Interactive controls**: Real-time manipulation and parameter adjustment
- **Flexible input format**: Support for arbitrary polygonal tiles and connection patterns
- **Multiple connection types**: Spherical joints, hinge connections, and rigid connections
- **Automated and manual deployment**: Both programmatic and interactive force application
- **Real-time visualization**: 3D rendering with camera controls and visual feedback

### Applications
- Kirigami metamaterial design and analysis
- Deployable structure simulation
- Mechanical metamaterial exploration
- Educational demonstrations of kirigami principles
- Research in shape-morphing materials

---

## Installation

### Prerequisites
- Python 3.7 or higher
- NumPy
- PyBullet


### Installation Steps
```bash
# Clone the repository
git clone [repository-url]
cd Kirigami_simulation_3D

# Install dependencies
pip install -r requirements.txt

# Run a test simulation
python run_sim.py --vertices_file data/tangram_vertices.txt --constraints_file data/tangram_constraints.txt
```

---

## Quick Start

### Basic Usage Example
```bash
python run_sim.py \
    --vertices_file data/tangram_vertices.txt \
    --constraints_file data/tangram_constraints.txt \
    --brick_thickness 0.1 \
    --auto_expand \
    --spring_radius 5.0 \
    --spring_stiffness 100.0
```

### Running Your First Simulation
1. Prepare your input files (vertices and constraints)
2. Run the simulation with basic parameters
3. Use mouse and keyboard controls to interact
4. Save simulation state when desired

---

## Input Data Format

### Vertices File Format
**Purpose**: Defines the geometric shape of each kirigami tile

**Format**: Each line represents one polygonal tile with `3n` space-separated float values
```
x1 y1 z1 x2 y2 z2 x3 y3 z3 ... xn yn zn
```

**Example** (Triangle):
```
0.0 0.0 0.0 1.0 0.0 0.0 0.5 1.0 0.0
```

**Example** (Quadrilateral):
```
0.0 0.0 0.0 1.0 0.0 0.0 1.0 1.0 0.0 0.0 1.0 0.0
```

**Important Notes**:
- Vertex ordering determines normal vector direction (right-hand rule)
- Consistent ordering across all tiles is crucial
- Each tile can have different numbers of vertices

### Constraints File Format
**Purpose**: Defines connections between kirigami tiles

**Format**: Each line specifies a connection between two tiles
```
tile1_index vertex1_index tile2_index vertex2_index connection_type
```

**Connection Types**:
- `1`: Bottom vertex connection (spherical joint)
- `2`: Top vertex connection (spherical joint)
- If type is omitted, defaults to bottom connection

**Example**:
```
0 0 1 1 1
0 1 1 2 2
```

### Force Bricks File Format (Optional)
**Purpose**: Specifies which tiles should receive targeted forces

**Format**: One tile index per line
```
0
2
5
```

---

## Command-Line Arguments

### Required Arguments
- `--vertices_file`: Path to vertices definition file
- `--constraints_file`: Path to constraints definition file

### Physics Parameters
- `--gravity`: Gravitational acceleration (default: -9.81)
- `--timestep`: Physics simulation timestep
- `--substeps`: Number of physics substeps per frame

### Expansion Parameters
- `--auto_expand`: Enable automatic expansion forces
- `--spring_radius`: Target expansion radius
- `--spring_stiffness`: Spring force stiffness coefficient
- `--spring_damping`: Spring damping coefficient

### Geometry Parameters
- `--brick_thickness`: Thickness of extruded 3D tiles

### Damping Parameters
- `--linear_damping`: Linear velocity damping
- `--angular_damping`: Angular velocity damping

### Visual Options
- `--ground_plane`: Add collision ground plane

---

## 3D Tile Geometry and Normal Vector Orientation

### Understanding Normal Vector Calculation
The normal vector is computed using the cross product of the first two edge vectors:
```
n = (v1 - v0) × (v2 - v0) / ||(v1 - v0) × (v2 - v0)||
```

### Vertex Ordering Guidelines
- **Consistent ordering**: Use either all clockwise or all counter-clockwise
- **Right-hand rule**: Determines which direction is "up" (positive normal)
- **Visual verification**: Check that all tiles extrude in expected directions

### Top vs Bottom Face Assignment
- **Bottom face**: Original vertices from input file
- **Top face**: Vertices translated along positive normal direction
- **Constraint types**: Type 1 connects bottom vertices, Type 2 connects top vertices

### Common Issues and Troubleshooting
- **Inverted tiles**: Check vertex ordering consistency
- **Connection failures**: Verify top/bottom vertex specifications
- **Unexpected orientations**: Validate normal vector directions

---

## Inter-Tile Connections

### Connection Types

#### Spherical Joints (Type 1 & 2)
- **Type 1**: Connect bottom vertices of two tiles
- **Type 2**: Connect top vertices of two tiles
- **Behavior**: Allows full rotational freedom around connection point
- **Use case**: Flexible hinges, ball-and-socket joints

#### Hinge Connections
- **Implementation**: Two spherical joints constraining rotation to one axis
- **Configuration**: Requires two constraint lines with shared tiles
- **Behavior**: Single-axis rotation
- **Use case**: Door hinges, folding mechanisms

#### Rigid Connections
- **Implementation**: Multiple constraints eliminating all relative motion
- **Configuration**: Four or more constraint points
- **Behavior**: No relative motion between tiles
- **Use case**: Welded joints, solid assemblies

### Constraint File Specification
[Detailed explanation of constraint syntax and examples]

### Best Practices for Defining Connections
[Guidelines for creating robust and predictable connections]

---

## Force Models and Deployment Mechanisms

### Automatic Expansion Forces
**Formula**: `F = k(r-d) - μv`
- `k`: Spring stiffness
- `r`: Target radius
- `d`: Current distance from center
- `μ`: Damping coefficient
- `v`: Velocity magnitude

### Manual Force Application
- Left-click dragging for direct manipulation
- Force magnitude depends on drag distance and duration

### Environmental Forces
- **Gravity**: Configurable magnitude and direction
- **Damping**: Linear and angular velocity reduction
- **Ground plane**: Optional collision surface

### Parameter Tuning Guidelines
- Start with low stiffness values and increase gradually
- Balance expansion force with damping for stability
- Adjust timestep for numerical stability

---

## Interactive Controls

### Mouse Controls
- **Left-click + drag**: Apply forces to tiles
- **Right-click on tile**: Toggle fix/unfix (red sphere indicates fixed)
- **Scroll wheel**: Zoom in/out
- **Ctrl + left-click + drag**: Rotate camera
- **Ctrl + right-click + drag**: Zoom
- **Ctrl + middle-click + drag**: Pan camera

### Keyboard Controls
- **R**: Reset simulation to initial state
- **S**: Save current vertex positions
- **P**: Toggle pause/resume simulation (uses fixed constraints for stable pausing)
- **Q**: Quit simulation

> **Note**: The pause functionality (P key) now uses PyBullet's fixed constraints to freeze all objects in place, preventing unwanted movement due to gravity or residual forces. This provides much more stable pausing behavior compared to velocity-based methods.

### Camera Navigation
[Detailed camera control instructions]

### Tile Manipulation
[Instructions for interacting with individual tiles]

---

## Example Projects

### 2D-to-2D Deployments
[Step-by-step examples with input files and expected results]

### 2D-to-3D Deployments
[Examples showing flat-to-3D transformations]

### 3D-to-3D Deployments
[Complex 3D shape transformations]

---

## Advanced Usage
[Advanced features and customization options]

---

## Troubleshooting
[Common problems and solutions]

---

## Development and Extension

### Code Structure

The simulator is organized into several key modules:

- **`utils/physics_utils.py`**: New utility module providing reusable physics manipulation functions
  - `fix_object_to_world()`: Fix objects to world frame using constraints
  - `unfix_object_from_world()`: Remove fixed constraints to restore dynamics
  - `fix_multiple_objects_to_world()`: Batch fixing for pause functionality
  - `create_visual_indicator()`: Create visual markers for fixed objects
  - These utilities are used by both interactive controls and pause functionality
  
- **`simulation/interactive_controls.py`**: Enhanced mouse-based interaction system
  - Now uses `physics_utils` for cleaner, more reliable object fixing
  - Improved error handling and visual feedback
  
- **`simulation/event_handler.py`**: Updated pause mechanism
  - Replaced velocity-based pausing with constraint-based fixing
  - More stable and predictable pause behavior
  
- **`utils/setup.py`**: Enhanced PyBullet configuration
  - Disabled profiling to prevent `timings_*.json` file generation
  - Optimized physics engine parameters for better performance

The new physics utilities provide a clean separation of concerns and enable consistent object manipulation across different parts of the simulator.

---

## Technical Background
[Theoretical foundations and implementation details]

---

## Frequently Asked Questions (FAQ)
[Common questions and answers]

---

## Contributing
[Guidelines for contributing to the project]

---

## Citation
```bibtex
@article{pykirigami2025,
    title={PyKirigami: An interactive Python simulator for kirigami metamaterials},
    author={Jiang, Qinghai and Choi, Gary P. T.},
    journal=
    year=
}
```

---

## License
[License information]

---

## Contact
- **Qinghai Jiang**: qhjiang@math.cuhk.edu.hk
- **Gary P. T. Choi**: ptchoi@cuhk.edu.hk
- **Department of Mathematics, The Chinese University of Hong Kong**
