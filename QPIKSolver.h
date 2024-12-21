/*
 * This code was developed by Shreyas.
 * It is based on concepts and algorithms from the work of Sina Mirrazavi
 * at the Learning Algorithms and Systems Laboratory, EPFL, Switzerland.
 *
 * The original code was licensed under the GNU General Public License (GPL v2),
 * but this code is an independent implementation and does not directly modify
 * or distribute the original code.
 *
 *
 * For more information on the original code, please refer to:
 * Author: Sina Mirrazavi
 * Email: sina.mirrazavi@epfl.ch
 * Website: lasa.epfl.ch
 */
#include <stdio.h>
#include <stdlib.h>
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <time.h>
#include <vector>

using namespace Eigen;
using namespace std;

/*
Format the appearance of matrix outputs for easy readability.

Example Usage:

    IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    MatrixXd mat(r, c);
    std::cout << mat.format(CleanFmt) << std::endl;
*/
IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

struct JacobianS
{
    MatrixXd Jacobian[7];    //Array of 7 Jacobian matrices
    MatrixXd Jacobian7[7];	// The size of jacobian is 7\times 3
    Vector3d LinkPos[7];
};

struct SRobotIK {
    bool jacobianIsSet;                     // Boolean flag to check if the Jacobian matrix has been set
    bool desiredPositionIsSet;              // Boolean flag to check if the desired position has been set
    bool stateIsSet;                        // Boolean flag to check if the robot state is set
    int index;                              // Index of the robot in some list or collection
    int numLinks;                           // Number of links in the robot's kinematic chain
    int numConstraints;                     // Number of constraints in the inverse kinematics problem

    MatrixXd W;                             // Weight matrix used for optimization or Jacobian-related computations
    MatrixXd Jacobian;                      // Jacobian matrix (relates joint velocities to end-effector velocity)

    // Joint position bounds
    VectorXd Uq;                           
    VectorXd Lq;                           

    // Joint velocity bounds
    VectorXd UDq;                          
    VectorXd LDq;                         

    // Joint acceleration bounds
    VectorXd UDDq;                        
    VectorXd LDDq;                         

    VectorXd DesiredEnd;                    // Desired end-effector position or pose (as a vector)
    VectorXd q;                             // Current joint positions (configuration of the robot)
    VectorXd Dq;                            // Current joint velocities

    JacobianS JacobianR;                    // Jacobian for a specific robot (likely defined earlier)
};

/*
This 'typedef' creates an alias, so 'my_object_data' is now a type that can be used for variables.
Define a struct called 'my_object_data'
*/
typedef struct {
    MatrixXd J;
    VectorXd X;
    VectorXd desired;

} myObjectdata;  

enum SolverLevel {
    VelocityLevel = 0,      // When the solver operates at the velocity level.
    AccelerationLevel       // When the solver operates at the acceleration level.
};

enum SolverType {
    Numerical = 0,    // Numerical solvers that solve problems using numerical methods.
    Dynamical         // Dynamical solvers that may account for time-dependent systems or dynamics.
};

enum SolverNumerical {
    Nlopt,       // A specific numerical optimization solver (Nonlinear Optimization).
    CVXgen1,     // CVXgen is a solver for optimization problems, version 1.
    CVXgen2      // CVXgen is a solver for optimization problems, version 2.
};

int MaxNumOfRobots = 3;

bool saveThePerformace = true;

class QPIKSolver
{
public:
    void Initialize(int NumOfRobots, double dt, SolverType type, SolverLevel level, bool SuperConstraint);
    void InitializeRobot(int index, int numLinks, int numConstraints, MatrixXd W, VectorXd Uq, VectorXd Lq, VectorXd UDq, VectorXd LDq, VectorXd UDDq, VectorXd LDDq);
	void InitializeRobot(int index,int numLinks,int numConstraints,MatrixXd W,VectorXd Uq, VectorXd Lq,	VectorXd UDq,VectorXd LDq);
    void FinalizeInitialization();
	void setJacobian(int index, MatrixXd Jacobian);
	void setJacobianLinks(int index, JacobianS Jacobian);
	void setDesired(int index, VectorXd DesiredEnd);
	void setState(int index, VectorXd q, VectorXd Dq);
	void getState(int index, VectorXd &Dq);
    void getGamma(double &gamma);
	void Solve();

private:
    void        ERROR();
    void        PrintRobot(SRobotIK Robot);
    VectorXd	OmegaProjector(VectorXd Input, VectorXd Upper, VectorXd Lower);
    inline void ConstructVel();
    inline void ConstructBoundariesVel();
    inline void restartTheRobots();
    inline void CheckFeasibility(VectorXd U);
    inline void loadDefaultData();

    double dt_;
    SolverType SolverType_;
    SolverLevel SolverLevel_;
    SolverNumerical SolverNumerical_;
    SRobotIK *Robots_;

    int NumOfRobots_;
    int DimensionConstraint_;
    int DimensionQ_;

    MatrixXd W_;
	MatrixXd M_;
	MatrixXd J_;
	VectorXd b_;
	VectorXd POmega_;

	VectorXd UPlus_;
	VectorXd UMinus_;
	VectorXd ThetaPlus_;
	VectorXd ThetaMinus_;

	MatrixXd I_;
	MatrixXd HandleMI_;
	VectorXd HandleProjection;

	VectorXd U_;
	VectorXd DU_;

	VectorXd ConstUpper_;
	VectorXd ConstLower_;

	double muP_;
	double etaP_;

	double		Gamma_;
	VectorXd	DGamma_;
    bool        considerCollision;
    double      lambda;

	bool SuperConstraint_;

	double t1;

	clock_t duration;

	MatrixXd GQP_;
	VectorXd g0QP_;
	MatrixXd CEQP_;
	VectorXd ce0QP_;
	VectorXd XQP_;

	myObjectdata dataObject;

	// nlopt::opt *opt;

	// std::vector<double> lb_;
	// std::vector<double> ub_;

    ofstream myfile;

    // SGF::SavitzkyGolayFilter *filter;
    // SGF::Vec inp;
    // SGF::Vec outp;

    int retCode;
};