# pyceres

This repository provides minimal Python bindings for the [Ceres Solver](http://ceres-solver.org/) and the implementation of factor graphs for bundle adjustment and pose graph optimization.

## Installation

1. Clone the repository and its submodule by running:

```sh
git clone --recursive git@github.com:cvg/pyceres.git
cd pyceres
```

2. Install [COLMAP 3.9.1](https://colmap.github.io/)

3. Build the package:

```sh
pip install -e .
```

### Docker image

Alternatively, you can build the Docker image:

```sh
export COLMAP_VERSION=3.9.1
export CUDA_ARCHITECTURES=70
docker build -t pyceres \
    --build-arg COLMAP_VERSION=${COLMAP_VERSION}  \
    --build-arg CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES}  \
    -f Dockerfile .
```

## Factor graph optimization

For now we support the following cost functions, defined in `_pyceres/factors/`:
- camera reprojection error (with fixed or variable pose)
- rig reprojection error (with fixed or variable rig extrinsics)
- relative pose prior
- absolute pose prior

All factors support basic observation covariances. Reprojection error costs rely on camera models defined in COLMAP. Absolute poses are represented as quaternions and are expressed in the sensor frame, so are pose residuals, which use the right-hand convention as in the [GTSAM library](https://github.com/borglab/gtsam).

## Examples
See the Jupyter notebooks in `examples/`.

## TODO
- [ ] Define a clean interface for covariances, like in GTSAM
- [ ] Add bindings for Ceres covariance estimation
- [ ] Use proper objects for poses, e.g. from Sophus
- [ ] Proper benchmark against GTSAM

## Credits
The core bindings were written by Nikolaus Mitchell for [ceres_python_bindings](https://github.com/Edwinem/ceres_python_bindings) and later adapted by [Philipp Lindenberger](https://github.com/Phil26AT) for [pixel-perfect-sfm](https://github.com/cvg/pixel-perfect-sfm).
