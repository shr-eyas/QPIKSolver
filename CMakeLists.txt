cmake_minimum_required(VERSION 3.10)

# Project name
project(QPIK)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)

# Find BLAS and LAPACK
# find_package(BLAS REQUIRED)
# find_package(LAPACK REQUIRED)

# Debug: Print the detected BLAS and LAPACK libraries
# message(STATUS "BLAS libraries: ${BLAS_LIBRARIES}")
# message(STATUS "LAPACK libraries: ${LAPACK_LIBRARIES}")

# Add the executable target
add_executable(QPIK
    src/runSolver.cpp
    src/QPIKSolver.cpp
    external/sg_differentiation/src/sg_filter.cpp
    # external/svm_grad/src/svm_grad.cpp 
)

# Link BLAS and LAPACK libraries to the target
# target_link_libraries(QPIK PRIVATE ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})

# Include directories for the target
target_include_directories(QPIK PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    # ${PROJECT_SOURCE_DIR}/external/svm_grad/include
    ${PROJECT_SOURCE_DIR}/external/sg_differentiation/include
)

# Ensure the C++11 standard is enabled
target_compile_features(QPIK PRIVATE cxx_std_11)