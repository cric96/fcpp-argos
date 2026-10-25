cmake_minimum_required(VERSION 3.18)
project(fcpp_argos_example CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ARGoS (installed system-wide, provides ARGoSConfig.cmake)
find_package(ARGoS REQUIRED)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

# FCPP (git submodule)
option(FCPP_INTERNAL_TESTS "Build internal tests for FCPP." OFF)
add_subdirectory(fcpp/src)

# one plugin library with every example controller and the GUI label drawer
file(GLOB SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_library(fcpp_argos MODULE ${SOURCES})
target_include_directories(fcpp_argos PRIVATE ${ARGOS_QTOPENGL_INCLUDE_DIRS})
target_link_libraries(fcpp_argos
    fcpp
    argos3core_simulator
    argos3plugin_simulator_footbot
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_entities
    argos3plugin_simulator_qtopengl
    ${ARGOS_QTOPENGL_LIBRARIES})
