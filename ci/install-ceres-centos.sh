#!/bin/bash
set -e -x
uname -a
CURRDIR=$(pwd)

yum install -y gcc gcc-c++ ninja-build curl zip unzip tar

DEPENDENCIES=$(cat ${CURRDIR}/ci/vcpkg-dependencies.txt)
git clone https://github.com/microsoft/vcpkg ${VCPKG_INSTALLATION_ROOT}
cd ${VCPKG_INSTALLATION_ROOT}
git checkout ${VCPKG_COMMIT_ID}
./bootstrap-vcpkg.sh
./vcpkg install --recurse --clean-after-build \
    --triplet=${VCPKG_TARGET_TRIPLET} \
    --debug \
    ${DEPENDENCIES}
./vcpkg integrate install
