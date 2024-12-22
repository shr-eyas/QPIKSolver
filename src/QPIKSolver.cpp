#include "QPIKSolver.h"

IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

int order = 3;
int winlen = 20;
SGF::real sample_time = 0.002;

int MaxNumOfRobots = 3;
bool saveThePerformace = true;

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
    Robots_ = new RobotModel[NumOfRobots_];
    SolverLevel_ = level;
    SolverType_ = type;

    muP_ = 20;
    etaP_ = 0.9;
    SuperConstraint_ = SuperConstraint;
    SolverNumerical_ = CVXgen1;
    if (saveThePerformace){	myfile.open ("external/results/IKSolverPerformaceDynamical.txt");}
	considerCollision = false;
}

// void QPIKSolver::Initialize(int NumOfRobots, double dt, SolverType type, SolverLevel level, bool SuperConstraint, string svmFilename)
// {
// 	NumOfRobots_ = NumOfRobots;
//     dt_ = dt;
//     Robots_ = new RobotModel[NumOfRobots_];
//     SolverLevel_ = level;
//     SolverType_ = type;
// 	muP_ = 20;
//     etaP_ = 0.9;
//     SuperConstraint_ = SuperConstraint;
// 	SolverNumerical_ = CVXgen1;
// 	if (saveThePerformace){	myfile.open ("IK_solver_performace_Dynamical.txt");}

// 	svmBoundary_.loadModel(svmFilename);
// 	svmBoundary_.preComputeKernel(true);
// 	considerCollision = true;
//     lambda = 5;
// }

void QPIKSolver::InitializeRobot(int index, int numLinks, int taskDOF, MatrixXd W, VectorXd Uq, VectorXd Lq, VectorXd UDq, VectorXd LDq, VectorXd UDDq, VectorXd LDDq)
{
	InitializeRobot(index, numLinks, taskDOF, W, Uq, Lq, UDq, LDq);

    if (SolverLevel_ == AccelerationLevel)
    {
        if((UDDq.rows() != numLinks) || (LDDq.rows() != numLinks))
        {
            cout << "Initialization of "<< index << "th robot is wrong." << endl;
			cout << "Number of links: " << numLinks << endl;
			cout << "UDDq: " << UDDq << endl;
			cout << "LDDq: " << LDDq << endl;
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

/*
 * @brief Declare the parameters of Robots
 * 
 * @param index 
 * @param numLinks Number of the links.
 * @param taskDOF Dimensionality of the end-effector's task space.
 * @param W Weight matrix for scaling joint contribution in IK. 
 * @param Uq Upper bound of the joints' positions.
 * @param Lq Lower bound of the joints' positions.
 * @param UDq Upper bound of the joints' velocities.
 * @param LDq Lower bound of the joints' velocities.
 */
void QPIKSolver::InitializeRobot(int index, int numLinks, int taskDOF, MatrixXd W, VectorXd Uq, VectorXd Lq, VectorXd UDq, VectorXd LDq)
{
    if (index > NumOfRobots_-1)
    {
        cout << "Initialization of " << index << "th robot is wrong." << endl;
		cout << "Index: " << index << " Max number of robot: " << NumOfRobots_-1 << endl;
		ERROR();
    }

    Robots_[index].index = index;
    Robots_[index].numLinks = numLinks;
    Robots_[index].taskDOF = taskDOF;

    Robots_[index].W.resize(Robots_[index].numLinks, Robots_[index].numLinks);
	Robots_[index].Jacobian.resize(Robots_[index].taskDOF, Robots_[index].numLinks);

    Robots_[index].Uq.resize(Robots_[index].numLinks);
	Robots_[index].Lq.resize(Robots_[index].numLinks);

	Robots_[index].UDq.resize(Robots_[index].numLinks);
	Robots_[index].LDq.resize(Robots_[index].numLinks);

	Robots_[index].UDDq.resize(Robots_[index].numLinks);
	Robots_[index].LDDq.resize(Robots_[index].numLinks);

	Robots_[index].q.resize(Robots_[index].numLinks);
	Robots_[index].Dq.resize(Robots_[index].numLinks);
    Robots_[index].DesiredEnd.resize(Robots_[index].taskDOF);

    for (int i=0; i<7; i++)
	{
		Robots_[index].JacobianR.Jacobian[i].resize(3, 1+i); 
        Robots_[index].JacobianR.Jacobian[i].setZero();
		Robots_[index].JacobianR.Jacobian7[i].resize(3, 7); 
        Robots_[index].JacobianR.Jacobian7[i].setZero();
	}

    if ((Uq.rows()!=numLinks) || (Lq.rows()!=numLinks) || (LDq.rows()!=numLinks)||(UDq.rows()!=numLinks) || (W.cols()!=numLinks) || (W.rows()!=numLinks))
	{   
        cout << "Initialization of "<< index << "th robot is wrong." << endl;
        cout << "Number of links: " << numLinks << endl;
        cout << "Uq: " << Uq << endl;
        cout << "Lq: " << Lq << endl;
        cout << "UDq: " << UDq << endl;
        cout << "LDq: " << LDq << endl;
        cout << "W: " << W << endl;
        ERROR();
	}
    else
    {
        Robots_[index].W = W;
		Robots_[index].Uq = Uq;
		Robots_[index].Lq = Lq;
		Robots_[index].UDq = UDq;
		Robots_[index].LDq = LDq;
		Robots_[index].LDDq.setZero();
		Robots_[index].UDDq.setZero();
    }
    PrintRobot(Robots_[index]);
}

/*
 * QDimension: Total number of joints across all robots.
 * ConstraintDimension: Total task-space DOFs for all robots' end effectors.
 */
void QPIKSolver::FinalizeInitialization()
{
    int QDimension = 0;
    for (int i=0; i<NumOfRobots_; i++)
    {
        QDimension = QDimension + Robots_[i].numLinks;
    }
    int ConstraintDimension = 0;
    for (int i=0; i<NumOfRobots_; i++)
    {
        ConstraintDimension = ConstraintDimension + Robots_[i].taskDOF;
    }

    QDimension_ = QDimension;
    ConstraintDimension_ = ConstraintDimension;

    W_.resize(QDimension_, QDimension_);
    W_.setZero();

    J_.resize(ConstraintDimension_, QDimension_);
    J_.setZero();

    M_.resize(ConstraintDimension_ + QDimension_ + 1, ConstraintDimension_ + QDimension_ + 1);
    M_.setZero();

    HandleMI_.resize(ConstraintDimension_ + QDimension_ + 1, ConstraintDimension_ + QDimension_ + 1);
    HandleMI_.setZero();

    b_.resize(ConstraintDimension_ + QDimension_ + 1);
    b_.setZero();

    I_.resize(ConstraintDimension_ + QDimension_ + 1, ConstraintDimension_ + QDimension_ + 1); 
    I_.setIdentity();

    UPlus_.resize(ConstraintDimension_ + QDimension_ + 1);
    UPlus_.setZero();

    UMinus_.resize(ConstraintDimension_ + QDimension_ + 1);
    UMinus_.setZero();

    U_.resize(ConstraintDimension_ + QDimension_ + 1);
    U_.setZero();

    DU_.resize(ConstraintDimension_ + QDimension_ + 1);
    DU_.setZero();  

    POmega_.resize(ConstraintDimension_ + QDimension_ + 1);
    POmega_.setZero();

    HandleProjection.resize(ConstraintDimension_ + QDimension_ + 1);
    HandleProjection.setZero();

    CEQP_.resize(ConstraintDimension_, QDimension_);
    CEQP_.setZero();

    ce0QP_.resize(ConstraintDimension_);
    ce0QP_.setZero();

    XQP_.resize(QDimension_);
    XQP_.setZero();

    ThetaPlus_.resize(QDimension_);
    ThetaPlus_.setZero();

    ThetaMinus_.resize(QDimension_);
    ThetaMinus_.setZero();

    DGamma_.resize(QDimension_); 
    DGamma_.setZero();

    Gamma_ = 2;

    ConstUpper_.resize(NumOfRobots_);
    ConstLower_.resize(NumOfRobots_);

    restartTheRobots();

    filter= new SGF::SavitzkyGolayFilter(QDimension_, order, winlen, sample_time);
	inp.resize(QDimension_);
	outp.resize(QDimension_);
	cout << "Constraint Dimension: " << ConstraintDimension_ << endl;
	cout << "Q Dimension " << QDimension_ << endl;
}

inline void QPIKSolver::restartTheRobots()
{
	for (int i=0; i<NumOfRobots_; i++)
	{
		Robots_[i].jacobianIsSet = false;
		Robots_[i].desiredPositionIsSet = false;
		Robots_[i].stateIsSet = false;
	}
}

void QPIKSolver::setJacobian(int index, MatrixXd Jacobian)
{
	if ((Robots_[index].Jacobian.cols()!=Jacobian.cols()) || (Robots_[index].Jacobian.rows()!=Jacobian.rows()))
	{
		cout << "Jacobian of " << index << "th robot is wrong." << endl;
		cout << "The input Jacobian dimension " << Jacobian.rows() << " * " << Jacobian.cols() << endl;
		cout << "The robot Jacobian dimension " << Robots_[index].Jacobian.rows() << " * " << Robots_[index].Jacobian.cols() << endl;
		ERROR();
	}
	if (Robots_[index].jacobianIsSet==true)
	{
		cout << "Jacobian of " << index << "th robot is already set." << endl;
		ERROR();
	}
	Robots_[index].Jacobian=Jacobian;
	Robots_[index].jacobianIsSet=true;
}

VectorXd QPIKSolver::OmegaProjector(VectorXd Input, VectorXd Upper, VectorXd Lower)
{
    if (Input.rows()!=Upper.rows() || (Input.rows()!=Lower.rows()))
    {
        cout << "Problem with Omega Projector." << endl;
        cout << "Input dimension: " << Input.rows() << " Upper dimension: " <<Upper.rows() << " Lower dimension: " << Lower.rows() << endl;
		ERROR();
    }

    VectorXd output(Input.rows());      // Create a vector of the same size as Input 
    for (int i=0; i<Input.rows(); i++)
    {
        output(i) = std::min(std::max(Input(i), Lower(i)), Upper(i));
    }
    return Input;
}

void QPIKSolver::setDesired(int index, VectorXd DesiredEnd)
{
    if (Robots_[index].DesiredEnd.rows()!=DesiredEnd.rows())
    {
        cout << "Desired end of " << index << "th robot is wrong." << endl;
		cout << "The input Desired end dimension " << DesiredEnd.rows() << endl;
		cout << "The robot Desired end dimension " << Robots_[index].DesiredEnd.rows() << endl;
		ERROR();
    }
    if (Robots_[index].desiredPositionIsSet==true)
	{
		cout << "Desired position of " << index << "th robot is already set." << endl;
		ERROR();
	}
    Robots_[index].DesiredEnd = DesiredEnd;
	Robots_[index].desiredPositionIsSet=true;
}

void QPIKSolver::ERROR()
{
	
}

void QPIKSolver::PrintRobot(RobotModel Robot)
{
	cout << "Printing parameters of Robot " << Robot.index + 1 << endl;
	cout << "Number of Links: " << Robot.numLinks << endl;
	cout << "Upper limit of joint positions: " << endl; cout << Robot.Uq << endl;
	cout << "Lower limit of joint positions: " << endl; cout << Robot.Lq << endl;
	cout << "Upper limit of joint velocities: " << endl; cout << Robot.UDq << endl;
	cout << "Lower limit of joint velocities: " << endl; cout << Robot.LDq << endl;
	cout << "q: " << endl; cout << Robot.q << endl;
	cout << "W: " << endl; cout << Robot.W << endl;
	cout << "Upper limit of joint accelerations: " << endl; cout << Robot.UDDq << endl;
	cout << "Lower limit of joint accelerations: " << endl; cout << Robot.UDDq << endl;
}