cmake_minimum_required(VERSION 3.10)
project(${SKBUILD_PROJECT_NAME} VERSION ${SKBUILD_PROJECT_VERSION})

find_package(Ceres 2.1 REQUIRED)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

find_package(pybind11 2.11.1 REQUIRED)

pybind11_add_module(pyceres _pyceres/bindings.cc)
target_include_directories(pyceres PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(pyceres PRIVATE glog::glog Ceres::ceres)
target_compile_definitions(
  pyceres
  PRIVATE VERSION_INFO="${PROJECT_VERSION}"
          GLOG_VERSION_MAJOR=${glog_VERSION_MAJOR}
          GLOG_VERSION_MINOR=${glog_VERSION_MINOR})
install(TARGETS pyceres LIBRARY DESTINATION .)
