#!/bin/bash
set -x -e
CURRDIR=$(pwd)

brew update
brew install git cmake ninja llvm gfortran

# When building lapack-reference, vcpkg/cmake looks for gfortran.
ln -sf $(which gfortran-14) "$(dirname $(which gfortran-14))/gfortran"

DEPENDENCIES=$(cat ${CURRDIR}/ci/vcpkg-dependencies.txt)
git clone https://github.com/microsoft/vcpkg ${VCPKG_INSTALLATION_ROOT}
cd ${VCPKG_INSTALLATION_ROOT}
git checkout ${VCPKG_COMMIT_ID}
./bootstrap-vcpkg.sh
./vcpkg install --recurse --clean-after-build \
    --triplet=${VCPKG_TARGET_TRIPLET} \
    ${DEPENDENCIES}
./vcpkg integrate install
