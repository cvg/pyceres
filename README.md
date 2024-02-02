# pyceres

This repository provides minimal Python bindings for the [Ceres Solver](http://ceres-solver.org/) and the implementation of factor graphs for bundle adjustment and pose graph optimization.

## Installation

Clone the repository and build the package:

```sh
git clone https://github.com/cvg/pyceres.git
cd pyceres
python -m pip install .
```

Alternatively, you can build the Docker image:

```sh
docker build -t pyceres -f Dockerfile .
```

## Factor graph optimization

Factors may be defined in Python (see [`examples/test_python_cost.py`](./examples/test_python_cost.py)) or in C++ with associated Python bindings.
[PyCOLMAP](https://github.com/colmap/colmap/tree/main/pycolmap) provides the following cost functions in `pycolmap.cost_functions`:
- reprojection error for different camera models, with fixed or variable pose and 3D points
- reprojection error for multi-camera rigs, with fixed or variable rig extrinsics
- error of absolute and relative poses
- Sampson error for epipolar geometry

See [`examples/`](./examples/) to use these factors.

## Credits
Pyceres was inspired by the work of Nikolaus Mitchell for [ceres_python_bindings](https://github.com/Edwinem/ceres_python_bindings) and is maintained by [Philipp Lindenberger](https://github.com/Phil26AT) and [Paul-Edouard Sarlin](https://psarlin.com/).
