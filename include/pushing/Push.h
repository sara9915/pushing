#include "json/json.h"

extern double Flag;

using namespace std;
using Eigen::ArrayXf;
using Eigen::MatrixXd;

#define NUM_STEPS 35
#define NUM_XVARIABLES 4
#define NUM_UVARIABLES 2
#define NUM_INEQ_CONSTRAINTS 550 //(NUM_XVARIABLES * 2 + NUM_UVARIABLES) * (NUM_STEPS - 1) // 20 //550 // Aeq -Aeq Ades (4+4+2)*(NUM_STEPS-1)
#define NUM_EQ_CONSTRAINTS 0
#define NUM_CONSTRAINTS 550 //(NUM_XVARIABLES * 2 + NUM_UVARIABLES) * (NUM_STEPS - 1) // 20 //550
#define NUM_VARIABLES (NUM_XVARIABLES + NUM_UVARIABLES) * NUM_STEPS // 6*NUM_STEPS

#ifndef PUSHING_H
#define PUSHING_H

class Push
{
public:
	// Matrices
	MatrixXd Aeq;
	MatrixXd beq;
	MatrixXd Ain;
	MatrixXd bin;
	MatrixXd lb;
	MatrixXd ub;
	MatrixXd Q;
	MatrixXd delta_u;
	MatrixXd solutionU;
	MatrixXd solutionX;
	// 2d Arrays
	// 1d Arrays
	char sense[NUM_CONSTRAINTS];
	char senseIC[10];
	char vtype[NUM_VARIABLES];
	double solution[NUM_VARIABLES];
	// Doubles
	double objval;
	double a;
	double b;
	double nu_p;
	double nu;
	double c_ls;
	double h_opt;
	double rx;
	double Family;
	// Boolean
	bool success;

	//~ double  Aeq[NUM_EQ_CONSTRAINTS][NUM_VARIABLES];
	//~ double  beq[NUM_EQ_CONSTRAINTS];
	//~ double  Ain[NUM_INEQ_CONSTRAINTS][NUM_VARIABLES];
	//~ double  bin[NUM_INEQ_CONSTRAINTS];
	//~ double  Atotal[NUM_CONSTRAINTS][NUM_VARIABLES];
	//~ double  btotal[NUM_CONSTRAINTS];
	//~ double  variables[NUM_VARIABLES];
	//~
	//~ double  c[NUM_CONSTRAINTS];
	//~ double  Q[NUM_VARIABLES][NUM_VARIABLES];
	//~ double  lb[NUM_VARIABLES];
	//~ double  ub[NUM_VARIABLES];
	//~ char    vtype[NUM_VARIABLES];
	//~ double solution[];
	//~ bool    success;
	//~ double  objval, sol[NUM_VARIABLES];
	//~ char    sense[NUM_CONSTRAINTS];

	GRBEnv env;
	GRBModel model;
	GRBVar *vars;
	GRBLinExpr lhs;
	GRBConstr constr[NUM_CONSTRAINTS];
	GRBConstr constrIC[10];

private:
public:
	Push(int Family);
	void ReadMatrices();
	void SetEquationSense();
	void SetVariableType();
	void BuildModel();
	double OptimizeModel();
	void UpdateICModel(double time, MatrixXd q_slider, MatrixXd q_pusher);
	void RemoveConstraints();
};
#endif
