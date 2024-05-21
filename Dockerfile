ARG UBUNTU_VERSION=22.04
FROM ubuntu:${UBUNTU_VERSION} as builder

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
COPY . /pyceres
WORKDIR /pyceres
RUN pip install --upgrade pip
RUN pip wheel . --no-deps -w dist-wheel -vv && \
    whl_path=$(find dist-wheel/ -name "*.whl") && \
    echo $whl_path >dist-wheel/whl_path.txt


#
# Runtime stage.
#
FROM ubuntu:${UBUNTU_VERSION} as runtime

# Install minimal runtime dependencies.
RUN apt-get update && \
    apt-get install -y --no-install-recommends --no-install-suggests \
        libgoogle-glog0v5 \
        libspqr2 \
        libcxsparse3 \
        libatlas3-base \
        python-is-python3 \
        python3-minimal \
        python3-pip

# Copy installed library in the builder stage.
COPY --from=builder /ceres_installed/ /usr/local/

# Install pyceres.
COPY --from=builder /pyceres/dist-wheel /tmp/dist-wheel
RUN pip install --upgrade pip
RUN cd /tmp && whl_path=$(cat dist-wheel/whl_path.txt) && pip install $whl_path
RUN rm -rfv /tmp/*

# # Verify if pyceres library is accessible from python.
RUN python -c "import pyceres"
