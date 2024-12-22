# QPIKSolver

QPIKSolver is a software package designed to solve the centralized inverse kinematic problem, formulated as a quadratic program (QP) and solved in real-time. This repository includes all necessary packages and instructions to get started.

##  Dependencies
Eigen http://eigen.tuxfamily.org/index.php?title=Main_Page
SVMGrad https://github.com/nbfigueroa/SVMGrad.git
SGDifferentiation https://github.com/epfl-lasa/sg_differentiation.git
Nlopt http://ab-initio.mit.edu/wiki/index.php/NLopt
CVXGEN http://cvxgen.com/docs/index.html 

## Features

- Handling the kinematic constraints of the robots.
- Inequity constraint (We used it for considering the self-collision avoidance (SCA) constraints).
- This package provides three options for solving the inverse kinematic:
  - LVI-based primal-dual Dynamical system solution: http://ieeexplore.ieee.org/abstract/document/1470152/
  - Nlopt
  - CVXgen

## Installation

### Prerequisites

- CMake 3.10 or higher
- GCC 11 or higher
- Armadillo library

### Build Instructions

1. Clone the repository:
    ```sh
    git clone https://github.com/shr-eyas/QPIKSolver.git
    cd QPIKSolver
    ```

2. Create a build directory and navigate into it:
    ```sh
    mkdir build
    cd build
    ```

3. Run CMake to configure the project:
    ```sh
    cmake ..
    ```

4. Build the project:
    ```sh
    make
    ```

## Usage

After building the project, you can run the solver using the following command:
```sh
./QPIK
```

## References

@article{mirrazavi2018unified,
  title={A unified framework for coordinated multi-arm motion planning},
  author={Mirrazavi Salehian, Seyed Sina and Figueroa, Nadia and Billard, Aude},
  journal={The International Journal of Robotics Research},
  pages={0278364918765952},
  publisher={SAGE Publications Sage UK: London, England}
}