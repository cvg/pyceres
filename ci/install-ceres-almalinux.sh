#!/bin/bash
set -e -x
uname -a
CURRDIR=$(pwd)

export PATH="/usr/bin"

# Install toolchain under AlmaLinux 8,
# see https://almalinux.pkgs.org/8/almalinux-appstream-x86_64/
yum install -y \
    gcc \
    gcc-c++ \
    gcc-gfortran \
    git \
    cmake3 \
    ninja-build \
    curl \
    zip \
    unzip \
    tar

DEPENDENCIES=$(cat ${CURRDIR}/ci/vcpkg-dependencies.txt)
git clone https://github.com/microsoft/vcpkg ${VCPKG_INSTALLATION_ROOT}
cd ${VCPKG_INSTALLATION_ROOT}
git checkout ${VCPKG_COMMIT_ID}
./bootstrap-vcpkg.sh
./vcpkg install --recurse --clean-after-build \
    --triplet=${VCPKG_TARGET_TRIPLET} \
    ${DEPENDENCIES}
./vcpkg integrate install
