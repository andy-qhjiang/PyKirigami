# PyKirigami: An Interactive Python Simulator for Kirigami Metamaterials

[![Stars](https://img.shields.io/github/stars/andy-qhjiang/PyKirigami?style=social)](https://github.com/andy-qhjiang/PyKirigami/stargazers)
[![Issues](https://img.shields.io/github/issues/andy-qhjiang/PyKirigami)](https://github.com/andy-qhjiang/PyKirigami/issues)
[![Last Commit](https://img.shields.io/github/last-commit/andy-qhjiang/PyKirigami)](https://github.com/andy-qhjiang/PyKirigami/commits/main)
[![License](https://img.shields.io/github/license/andy-qhjiang/PyKirigami)](https://github.com/andy-qhjiang/PyKirigami/blob/main/LICENSE)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue)](#)

#### Interactive Python 3D/2D kirigami simulation library

[Examples](https://github.com/andy-qhjiang/PyKirigami/tree/main/gallery) —
[Manual](https://github.com/andy-qhjiang/PyKirigami/wiki/Manual) —
[Discussions](https://github.com/andy-qhjiang/PyKirigami/discussions) —
[Citation](#citation)-
[License](#license)-
[Releases](https://github.com/andy-qhjiang/PyKirigami/releases)

<table>
  <tr>
    <td width="45%">
      <img src="gallery/cylinder_demo.gif" alt="Cylinder" width="96%" loading="lazy" />
      <div align="center"><small>(a)</small></div>
    </td>
    <td width="55%">
      <img src="gallery/partialSphere.gif" alt="Partial Sphere" width="100%" loading="lazy" />
      <div align="center"><small>(b)</small></div>
    </td>
  </tr>
  <tr>
    <td width="50%">
      <img src="gallery/heart_demo.gif" alt="Demo 3" width="100%" loading="lazy" />
      <div align="center"><small>(c)</small></div>
    </td>
    <td width="50%">
      <img src="gallery/s2d.gif" alt="Demo 4" width="95%" loading="lazy" />
      <div align="center"><small>(d)</small></div>
    </td>
  </tr>
  
</table>

### Getting Started

Prerequisites:
- Python 3.8+

Install with conda-forge (recommended on Windows/macOS):
```bash
conda create -n kirigami python=3.13
conda activate kirigami
conda install -c conda-forge numpy pybullet
```

### Quick Usage

Run the following command and you will get demo (b):

```bash
python run_sim.py --vertices_file data/partialSphere_vertices.txt --constraints_file data/partialSphere_constraints.txt --target_vertices_file data/partialSphere_target.txt --brick_thickness 0.02 --spring_stiffness 800
```

Run the following command and you will get demo (d):

```bash
python run_sim.py --vertices_file square2disk_vertices.txt --constraints_file square2disk_constraints.txt --target_vertices_file square2disk_target.txt --ground_plane --brick_thickness 0.1 --gravity -200 --force_damping 20
```


## Documentation
Full manual is in the Wiki 
https://github.com/andy-qhjiang/PyKirigami/wiki/Manual

## Citation

If you use PyKirigami in your research, please cite the article:

```bibtex
@article{pykirigami2025,
    title = {PyKirigami: An interactive Python simulator for kirigami metamaterials},
    author = {Jiang, Qinghai and Choi, Gary P. T.},
    journal = {arXiv preprint arXiv:2508.15753},
    url = {https://arxiv.org/abs/2508.15753},
    year = {2025}
}
```

For software citation of the codebase, see the repository's CITATION.cff.

## License
Licensed under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file for details. If you redistribute modified versions, preserve attributions and include the [NOTICE](NOTICE) file per Section 4.

---

## Contact
- **Qinghai Jiang**: qhjiang@math.cuhk.edu.hk
- **Gary P. T. Choi**: ptchoi@cuhk.edu.hk
- **Department of Mathematics, The Chinese University of Hong Kong**

