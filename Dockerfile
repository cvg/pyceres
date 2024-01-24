ARG UBUNTU_VERSION=22.04
ARG NVIDIA_CUDA_VERSION=12.3.1
FROM nvidia/cuda:${NVIDIA_CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION} as builder

ARG COLMAP_VERSION=3.9.1
ARG CUDA_ARCHITECTURES=70
ENV CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES}
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
        libboost-program-options-dev \
        libboost-filesystem-dev \
        libboost-graph-dev \
        libboost-system-dev \
        libeigen3-dev \
        libflann-dev \
        libfreeimage-dev \
        libmetis-dev \
        libgoogle-glog-dev \
        libgtest-dev \
        libsqlite3-dev \
        libglew-dev \
        qtbase5-dev \
        libqt5opengl5-dev \
        libcgal-dev \
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
    cmake ../ceres-solver-2.1.0 -GNinja && \
    ninja install

# Install Colmap.
RUN wget "https://github.com/colmap/colmap/archive/refs/tags/${COLMAP_VERSION}.tar.gz" -O colmap-${COLMAP_VERSION}.tar.gz && \
    tar zxvf colmap-${COLMAP_VERSION}.tar.gz && \
    mkdir colmap-build && \
    cd colmap-build && \
    cmake ../colmap-${COLMAP_VERSION} -GNinja -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES} && \
    ninja install


# Build pyceres.
COPY . /pyceres
WORKDIR /pyceres
RUN pip install . -vv
