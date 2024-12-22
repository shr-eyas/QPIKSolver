# include "QPIKSolver.h"

int main()
{
    // Create an instance of QPIKSolver
    QPIKSolver solver;

    // Initialize the solver with parameters
    int NumOfRobots = 2;      // Example: Two robots
    double dt = 0.01;         // Time step (example: 0.01 seconds)
    SolverType type = Numerical;  // Choose the solver type (Numerical or Dynamical)
    SolverLevel level = VelocityLevel;  // Solver level (Velocity or Acceleration)
    bool SuperConstraint = true;  // Whether or not to use super constraint

    // Initialize the solver with these parameters
    solver.Initialize(NumOfRobots, dt, type, level, SuperConstraint);

}
