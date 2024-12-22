# include "QPIKSolver.h"

int main()
{
    QPIKSolver solver;

    int NumOfRobots = 2;      
    double dt = 0.01;         
    SolverType type = Numerical; 
    SolverLevel level = VelocityLevel; 
    bool SuperConstraint = true;  
    // Initialize the solver with these parameters
    solver.Initialize(NumOfRobots, dt, type, level, SuperConstraint);

}
