cmake_minimum_required(VERSION 3.10)

project(QPIK)

set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/include)  
include_directories(/home/sophia/sg_differentiation/include) 

set(SOURCES
    src/runSolver.cpp
    src/QPIKSolver.cpp
    /home/sophia/sg_differentiation/src/sg_filter.cpp 
)

add_executable(QPIK ${SOURCES})