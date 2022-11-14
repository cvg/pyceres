cmake_minimum_required(VERSION 3.6)
project(PyCeres)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 3.3 REQUIRED)
find_package(COLMAP REQUIRED)

if(${CERES_VERSION} VERSION_LESS "2.1.0")
    message( SEND_ERROR "Ceres version >= 2.1 required." )
elseif(${CERES_VERSION} VERSION_LESS "2.2.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

set(PYCERES_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    ${COLMAP_INCLUDE_DIRS}
)
set(PYCERES_LINK_DIRS
    ${COLMAP_LINK_DIRS}
)
set(PYCERES_LIBRARIES
    ${CERES_LIBRARIES}
    ${COLMAP_LIBRARIES}
)
if(MSVC)
# these are added in COLMAP_LIBRARIES for UNIX systems but for some reasons they are omited for windows
# it crashes with LNK2019 because of that
# https://github.com/colmap/colmap/blob/9f3a75ae9c72188244f2403eb085e51ecf4397a8/cmake/CMakeConfig.cmake.in#L165
set(PYCERES_LIBRARIES
    ${PYCERES_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)
endif()

include_directories(
    _pyceres
    ${PYCERES_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
)
link_directories(${PYCERES_LINK_DIRS})

add_subdirectory(pybind11)

pybind11_add_module(pyceres _pyceres/bindings.cc)
target_link_libraries(pyceres PRIVATE ${PYCERES_LIBRARIES})
