# pyceres

This repository provides minimal Python bindings for the [Ceres Solver](http://ceres-solver.org/) and the implementation of factor graphs for bundle adjustment and pose graph optimization.

## Installation

Wheels for Python 8/9/10 on Linux, macOS 10+ (both Intel and Apple Silicon), and Windows can be installed using pip:
```sh
pip install pyceres
```

To build from source, follow the following steps:
1. Install the Ceres Solver following [the official instructions](http://ceres-solver.org/installation.html).
2. Clone the repository and build the package:

```sh
git clone https://github.com/cvg/pyceres.git
cd pyceres
python -m pip install .
```

Alternatively, you can build the Docker image:

```sh
docker build --target builder -t pyceres:builder .
docker build --target runtime -t pyceres:runtime .
```
Note: The builder image can be used for development and testing, as it contains
all the necessary dependencies for building the project. On the other hand, the
runtime version is streamlined for running pyceres, containing only the minimal
dependencies required for execution, making it lightweight.

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
