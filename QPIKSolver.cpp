#include "QPIKSolver.h"

IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

int order = 3;
int winlen = 20;

/*
 * @brief Objective function used in optimization routines.
 * 
 * This function calculates the error between the product of a matrix (J) and a vector (X),
 * and a desired target vector. It is typically used as an objective function in optimization
 * algorithms to minimize the error.
 * 
 * @param x Input vector (current guess for optimization) passed by reference.
 * @param grad Output gradient vector passed by reference. If not empty, this function
 *              fills it with the gradient values (for gradient-based optimization).
 * @param myFuncData Pointer to user-defined data (myObjectdata*) that contains the
 *                     matrix (J), vector (X), and the desired target vector.
 * 
 * @return The norm (magnitude) of the difference between (J * X) and the desired vector.
 *         This is the error to be minimized by the optimization algorithm.
 * 
 * The gradient is set to 1 in this function, but typically it would be computed based on the
 * objective function's partial derivatives with respect to the input vector x.
 * 
 * This function is part of an optimization algorithm (such as nonlinear optimization or least
 * squares) that aims to minimize the norm of the difference between a computed value (J * X)
 * and the desired target (desired).
 */
double myvfunc(const std::vector<double> &x, std::vector<double> &grad, void *myFuncData)
{
    int size = x.size();  // Get the size of the input vector
    myObjectdata *d = reinterpret_cast<myObjectdata*>(myFuncData);  // Cast user data to the correct type

    VectorXd X = d->X;  // Extract the vector X from the user data
    // Copy the values from the input vector x to the vector X
    for (int i = 0; i < X.rows(); i++) {
        X(i) = x[i];
    }

    // If the gradient vector is provided (not empty)
    if (!grad.empty()) {
        // Fill the gradient with 1's (this is a simple case, usually gradients are calculated)
        for (int i = 0; i < size; i++) {
            grad[i] = 1;
        }
    }

    // Calculate and return the norm of the difference between (J * X) and the desired vector
    return ((d->J) * X - d->desired).norm();
}

void QPIKSolver::Initialize(int NumOfRobots, double dt, SolverType type, SolverLevel level, bool SuperConstraint)
{
    NumOfRobots_ = NumOfRobots;
    dt_ = dt;
    Robots_ = new SRobotIK[NumOfRobots_];
    SolverLevel_ = level;
    SolverType_ = type;

    muP_ = 20;
    etaP_ = 0.9;
    SuperConstraint_ = SuperConstraint;
    SolverNumerical_ = CVXgen1;
    if (saveThePerformace){	myfile.open ("IK_solver_performace_Dynamical.txt");}
	considerCollision = false;
}

void QPIKSolver::InitializeRobot(int index, int numLinks, int numConstraints, MatrixXd W, VectorXd Uq, VectorXd Lq, VectorXd UDq, VectorXd LDq, VectorXd UDDq, VectorXd LDDq)
{
	InitializeRobot(index, numLinks, numConstraints, W, Uq, Lq, UDq, LDq);

    if (SolverLevel_ == AccelerationLevel)
    {
        if((UDDq.rows() != numLinks) || (LDDq.rows() != numLinks))
        {
            cout <<"Initialization of "<< index <<"th robot is wrong."<< endl;
			cout <<"N_links "<< numLinks<<endl;
			cout <<"U_DDp: "<< endl; cout << UDDq << endl;
			cout <<"U_DDp: "<< endl; cout << UDDq <<endl;
			cout <<"Initialization of "<< index <<"th robot is wrong."<< endl;
			ERROR();
        }
        else
        {
            Robots_[index].UDDq = UDDq;
            Robots_[index].LDDq = LDDq;
        }
    }
    PrintRobot(Robots_[index]);
}

inline void QPIKSolver::restartTheRobots()
{
	for(int i=0; i<NumOfRobots_; i++)
	{
		Robots_[i].jacobianIsSet = false;
		Robots_[i].desiredPositionIsSet = false;
		Robots_[i].stateIsSet = false;
	}
}

void QPIKSolver::PrintRobot(SRobotIK Robot)
{
	cout << "Printing of " << Robot.index << "th robot." << endl;
	cout << "Number of Links: " << Robot.numLinks << endl;
	cout << "Up: " << endl; cout << Robot.Uq << endl;
	cout << "Lp: " << endl; cout << Robot.Lq << endl;
	cout << "UDp: " << endl; cout << Robot.UDq << endl;
	cout << "LDp: " << endl; cout << Robot.LDq << endl;
	cout << "q: " << endl; cout << Robot.q << endl;
	cout << "W: " << endl; cout << Robot.W << endl;
	cout << "UDDp: " << endl; cout << Robot.UDDq << endl;
	cout << "UDDp: " << endl; cout << Robot.UDDq << endl;
}

void QPIKSolver::loadDefaultData()
{
    for (int i = 0; i < DimensionConstraint_; i++)
        for (int j = 0; j < DimensionQ_; j++)
            params.J[i+j*DimensionConstraint_] = CEQP_(i, j);
}