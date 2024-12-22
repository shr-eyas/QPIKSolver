cmake_minimum_required(VERSION 3.10)

project(QPIK)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Find Armadillo package
find_package(Armadillo REQUIRED)

# Add the executable with source files
add_executable(QPIK
    src/runSolver.cpp
    src/QPIKSolver.cpp
    external/sg_differentiation/src/sg_filter.cpp
    external/svm_grad/src/svm_grad.cpp
)

# Link libraries
target_link_libraries(QPIK PRIVATE
    armadillo
)

# Include directories
target_include_directories(QPIK PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/external/svm_grad/include
    ${PROJECT_SOURCE_DIR}/external/sg_differentiation/include
    ${ARMADILLO_INCLUDE_DIRS}
)

# Set required C++ features
target_compile_features(QPIK PRIVATE cxx_std_11)