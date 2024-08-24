cmake_minimum_required(VERSION 3.10)
project(${SKBUILD_PROJECT_NAME} VERSION ${SKBUILD_PROJECT_VERSION})

find_package(glog REQUIRED)
if(DEFINED glog_VERSION_MAJOR)
  # Older versions of glog don't export version variables.
  add_definitions("-DGLOG_VERSION_MAJOR=${glog_VERSION_MAJOR}")
  add_definitions("-DGLOG_VERSION_MINOR=${glog_VERSION_MINOR}")
endif()

find_package(Ceres 2.1 REQUIRED)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

find_package(pybind11 2.13.0 REQUIRED)

pybind11_add_module(pyceres _pyceres/bindings.cc)
target_include_directories(pyceres PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(pyceres PRIVATE glog::glog Ceres::ceres)
target_compile_definitions(pyceres PRIVATE VERSION_INFO="${PROJECT_VERSION}")
install(TARGETS pyceres LIBRARY DESTINATION .)
