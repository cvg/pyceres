cmake_minimum_required(VERSION 3.10)
project(${SKBUILD_PROJECT_NAME} VERSION ${SKBUILD_PROJECT_VERSION})

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES "native")
endif()
find_package(COLMAP REQUIRED)
if(${COLMAP_VERSION} VERSION_LESS "3.9.0")
    message( SEND_ERROR "COLMAP version >= 3.9.0 required, found ${COLMAP_VERSION}." )
endif()
if(${CERES_VERSION} VERSION_LESS "2.1.0")
    message( SEND_ERROR "Ceres version >= 2.1 required, found ${CERES_VERSION}." )
endif()


find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

find_package(pybind11 2.11.1 REQUIRED)

pybind11_add_module(pyceres _pyceres/bindings.cc)
target_include_directories(pyceres PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(pyceres PRIVATE colmap::colmap glog::glog Ceres::ceres)
target_compile_definitions(pyceres PRIVATE VERSION_INFO="${PROJECT_VERSION}")
install(TARGETS pyceres LIBRARY DESTINATION .)
