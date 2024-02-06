$CURRDIR = $PWD

$COMPILER_TOOLS_DIR = "${env:COMPILER_CACHE_DIR}/bin"
New-Item -ItemType Directory -Force -Path ${COMPILER_TOOLS_DIR}
$env:Path = "${COMPILER_TOOLS_DIR};" + $env:Path

$NINJA_PATH = "${COMPILER_TOOLS_DIR}/ninja.exe"
If (!(Test-Path -path ${NINJA_PATH} -PathType Leaf)) {
    $zip_path = "${env:TEMP}/ninja.zip"
    $url = "https://github.com/ninja-build/ninja/releases/download/v1.10.2/ninja-win.zip"
    curl.exe -L -o ${zip_path} ${url}
    Expand-Archive -LiteralPath ${zip_path} -DestinationPath ${COMPILER_TOOLS_DIR}
    Remove-Item ${zip_path}
}

cd ${CURRDIR}
git clone https://github.com/microsoft/vcpkg ${env:VCPKG_INSTALLATION_ROOT}
cd ${env:VCPKG_INSTALLATION_ROOT}
git checkout "${env:VCPKG_COMMIT_ID}"
./bootstrap-vcpkg.bat

cd ${CURRDIR}
& "./ci/enter_vs_dev_shell.ps1"

[System.Collections.ArrayList]$DEPS = Get-Content -Path "./ci/vcpkg-dependencies.txt"
& "${env:VCPKG_INSTALLATION_ROOT}/vcpkg.exe" install --recurse --clean-after-build `
    --triplet="${env:VCPKG_TARGET_TRIPLET}" @DEPS
& "${env:VCPKG_INSTALLATION_ROOT}/vcpkg.exe" integrate install
