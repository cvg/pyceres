cmake_minimum_required(VERSION 3.10)
project(${SKBUILD_PROJECT_NAME} VERSION ${SKBUILD_PROJECT_VERSION})

find_package(Ceres REQUIRED)
if(NOT TARGET Ceres::ceres)
    # Older Ceres versions don't come with an imported interface target.
    add_library(Ceres::ceres INTERFACE IMPORTED)
    target_include_directories(
        Ceres::ceres INTERFACE ${CERES_INCLUDE_DIRS})
    target_link_libraries(
        Ceres::ceres INTERFACE ${CERES_LIBRARIES})
endif()
if(${CERES_VERSION} VERSION_LESS "2.1.0")
    message( SEND_ERROR "Ceres version >= 2.1 required, found ${CERES_VERSION}." )
endif()

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

find_package(pybind11 2.11.1 REQUIRED)

pybind11_add_module(pyceres _pyceres/bindings.cc)
target_include_directories(pyceres PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(pyceres PRIVATE glog::glog Ceres::ceres)
target_compile_definitions(pyceres PRIVATE VERSION_INFO="${PROJECT_VERSION}")
install(TARGETS pyceres LIBRARY DESTINATION .)
