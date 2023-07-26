cmake_minimum_required(VERSION 3.26)
project(myopenvdb)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# OpenVDB
find_package(OpenVDB REQUIRED)
include_directories(${OpenVDB_INCLUDE_DIR} SYSTEM)
find_package(TBB REQUIRED)
include_directories(${TBB_INCLUDE_DIRS} SYSTEM)
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS} SYSTEM)
include_directories(.)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-braces")
endif()

# ligigl
option(LIBIGL_USE_STATIC_LIBRARY       "Use libigl as static library" OFF)
option(LIBIGL_WITH_OPENGL              "Use OpenGL"         ON)
option(LIBIGL_WITH_PNG                 "Use PNG"            ON)
option(LIBIGL_WITH_OPENGL_GLFW         "Use GLFW"           ON)
find_package(LIBIGL REQUIRED)

# loop_subdiv
include_directories("include/")
file(GLOB SRCFILES src/*.cpp)

add_library(core ${SRCFILES})
target_link_libraries(core igl::core igl::opengl igl::opengl_glfw)

add_executable(solveproblem problem_1.cpp)
target_link_libraries(solveproblem OpenVDB::openvdb igl::core igl::opengl igl::opengl_glfw)