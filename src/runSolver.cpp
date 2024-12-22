# include "QPIKSolver.h"

int main() 
{
    QPIKSolver solver;

    int NumOfRobots = 2;      
    double dt = 0.01;         
    SolverType type = Numerical; 
    SolverLevel level = VelocityLevel; 
    bool SuperConstraint = true;  

    solver.Initialize(NumOfRobots, dt, type, level, SuperConstraint);
    
    int numLinks = 2;
    int taskDOF = 2;
    MatrixXd W = MatrixXd::Identity(numLinks, numLinks);

    // Joint position bounds (-pi, pi)
    VectorXd Uq(numLinks);  
    VectorXd Lq(numLinks);  
    Uq << M_PI, M_PI;      
    Lq << -M_PI, -M_PI;    

    // Joint velocity bounds (-2 rad/s, 2 rad/s)
    VectorXd UDq(numLinks);
    VectorXd LDq(numLinks); 
    UDq << 2, 2;            
    LDq << -2, -2;          

    // Joint acceleration bounds (-5 rad/s^2, 5 rad/s^2)
    VectorXd UDDq(numLinks); 
    VectorXd LDDq(numLinks); 
    UDDq << 5, 5;            
    LDDq << -5, -5;          

    // Initialize each robot
    solver.InitializeRobot(0, numLinks, taskDOF, W, Uq, Lq, UDq, LDq);
    solver.InitializeRobot(1, numLinks, taskDOF, W, Uq, Lq, UDq, LDq);

    return 0;
}