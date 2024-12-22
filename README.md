# QPIKSolver

QPIKSolver is a software package designed to solve the centralized inverse kinematic problem, formulated as a quadratic program (QP) and solved in real-time. This repository includes all necessary packages and instructions to get started.

## Features

- Real-time solving of inverse kinematic problems
- Support for multiple numerical optimization solvers
- Modular design for easy extension and customization

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

## Directory Structure

- `src/`: Contains the source code for the solver.
- `include/`: Contains the header files.
- `external/`: Contains external dependencies.
- `build/`: Directory for build files.
- `CMakeLists.txt`: CMake configuration file.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Contact

For any questions or suggestions, please open an issue or contact the repository owner at shreyaskumar.1102@gmail.com.
