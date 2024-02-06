ARG UBUNTU_VERSION=22.04
ARG NVIDIA_CUDA_VERSION=12.3.1
FROM nvidia/cuda:${NVIDIA_CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION} as builder

ENV QT_XCB_GL_INTEGRATION=xcb_egl

# Prevent stop building ubuntu at time zone selection.
ENV DEBIAN_FRONTEND=noninteractive

# Prepare and empty machine for building.
RUN apt-get update && \
    apt-get install -y --no-install-recommends --no-install-suggests \
        git \
        cmake \
        ninja-build \
        build-essential \
        libeigen3-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libgtest-dev \
        libatlas-base-dev \
        libsuitesparse-dev \
        python-is-python3 \
        python3-minimal \
        python3-pip \
        python3-dev \
        python3-setuptools

# Install Ceres.
RUN apt-get install -y --no-install-recommends --no-install-suggests wget && \
    wget "http://ceres-solver.org/ceres-solver-2.1.0.tar.gz" && \
    tar zxf ceres-solver-2.1.0.tar.gz && \
    mkdir ceres-build && \
    cd ceres-build && \
    cmake ../ceres-solver-2.1.0 -GNinja \
        -DCMAKE_INSTALL_PREFIX=/ceres_installed && \
    ninja install
RUN cp -r /ceres_installed/* /usr/local/
# Build pyceres.
ADD . /pyceres
WORKDIR /pyceres
RUN pip install . -vv
