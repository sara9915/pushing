#include "pushing/headerFiles.h"

using namespace std;
using Eigen::MatrixXd;

Push::Push(int _Family) : // Constructor of Push Class  //peterkty: put the initialization of member variable here
						  env(), model(env), lhs(0)
{
	// Set size of Matrices
	Ain.resize(NUM_CONSTRAINTS, NUM_VARIABLES);
	bin.resize(NUM_CONSTRAINTS, 1);
	lb.resize(NUM_VARIABLES, 1);
	ub.resize(NUM_VARIABLES, 1);
	Q.resize(NUM_VARIABLES, NUM_VARIABLES);
	delta_u.resize(2, 1);
	solutionU.resize(NUM_UVARIABLES * NUM_STEPS, 1);
	solutionX.resize(NUM_XVARIABLES * NUM_STEPS, 1);

	// Initialize scalars
	a = 0.09;				  // length of slider [m]
	b = 0.075;				  // width of slider [m]
	c_ls = 0.036742346141748; // c = mmax/fmax (CREATE FUNCTION TO CALCULATE IT)
	h_opt = 0.025;			  // sample time [s] for optimization
	rx = -a / 2.0;			  // point of contact
	nu_p = 0.3;				  // coefficient of friction (pusher-slider)
	Family = _Family;

	ReadMatrices();
	std::cout << "read matrices" << std::endl;
	SetEquationSense();
	std::cout << "sense equation" << std::endl;
	SetVariableType();
	std::cout << "set variable" << std::endl;
	BuildModel();
	std::cout << "build model" << std::endl;
}
//*******************************************************************************************
void Push::ReadMatrices()
{
	// Load Json file
	ifstream file("/home/workstation/dope_ros_ws/src/pushing/mpc_parameters/Matrices_1.json", std::ifstream::binary);
	Json::Value root;
	file >> root;

	// std::string Ain_string;
	// std::string bin_string;
	if (Family == 1)
	{
		std::cout << "----------- FAMILY 1 ----------------" << std::endl;
		write_matrix_JSON(root["Matrices"]["Ain1"], Ain);
		// write_matrix_JSON(root["Matrices"]["bin1"], bin);
		write_matrix_JSON_index(root["Matrices"]["bin1"], bin, bin.rows(), bin.cols());
	}
	else if (Family == 2)
	{
		std::cout << "----------- FAMILY 2 ----------------" << std::endl;
		write_matrix_JSON(root["Matrices"]["Ain2"], Ain);
		// write_matrix_JSON(root["Matrices"]["bin2"], bin);
		write_matrix_JSON_index(root["Matrices"]["bin2"], bin, bin.rows(), bin.cols());
	}
	else
	{
		std::cout << "----------- FAMILY 3 ----------------" << std::endl;
		write_matrix_JSON(root["Matrices"]["Ain3"], Ain);
		// write_matrix_JSON(root["Matrices"]["bin3"], bin);
		write_matrix_JSON_index(root["Matrices"]["bin3"], bin, bin.rows(), bin.cols());
	}
	write_matrix_JSON(root["Matrices"]["Q"], Q);
}
//*******************************************************************************************
void Push::SetEquationSense()
{
	for (int i = 0; i < NUM_CONSTRAINTS; i++)
	{
		sense[i] = '<';
	}
}
//*******************************************************************************************
void Push::SetVariableType()
{
	// for (int i = 0; i < NUM_VARIABLES; i++)
	// {
	// 	lb(i) = -10;
	// 	ub(i) = 10;
	// 	vtype[i] = 'C';
	// }
	for (int i = 0; i < NUM_UVARIABLES * NUM_STEPS; i++)
	{
		if (i % 2 == 0)
		{
			// normal velocity
			lb(i) = -0.05 + 0.01;
			ub(i) = -0.05 + 0.1;
			vtype[i] = 'C';
		}
		else
		{
			// velocità tangenziale
			lb(i) = -0.1;
			ub(i) = +0.1;
			vtype[i] = 'C';
		}
	}
	for (int i = NUM_UVARIABLES * NUM_STEPS; i < NUM_VARIABLES; i++)
	{
		if ((i + 1 - NUM_UVARIABLES * NUM_STEPS) % 4 == 0)
		{
			lb(i) = -a / 2;
			ub(i) = +a / 2;
		}
		else
		{
			lb(i) = -100;
			ub(i) = 100;
		}
		vtype[i] = 'C';
	}
}
//*******************************************************************************************
void Push::BuildModel()
{
	// Doubles
	double Aind[Ain.rows()][Ain.cols()];
	double bind[bin.rows()];
	double lbd[lb.rows()];
	double Qd[Q.rows()][Q.cols()];
	double ubd[ub.rows()];
	// Convert matrices to arrays
	matrix_to_array(Ain.rows(), Ain.cols(), Aind[0], Ain);
	matrix_to_array(bin.rows(), 1, bind, bin);
	matrix_to_array(lb.rows(), 1, lbd, lb);
	matrix_to_array(ub.rows(), 1, ubd, ub);
	matrix_to_array(Q.rows(), Q.cols(), Qd[0], Q);

	//~ cout<< Ain << endl;
	//~ cout<< " " << endl;
	//~ cout<< bin << endl;

	vars = model.addVars(lbd, ubd, NULL, vtype, NULL, NUM_VARIABLES);
	model.update();
	// Add cost
	GRBQuadExpr obj = 0;
	for (int j = 0; j < NUM_VARIABLES; j++)
		obj += 0 * vars[j]; // fd[j]
	for (int i = 0; i < NUM_VARIABLES; i++)
		for (int j = 0; j < NUM_VARIABLES; j++)
			if (Qd[i][j] != 0)
				obj += Qd[i][j] * vars[i] * vars[j];
	model.setObjective(obj);
	// Add constraints
	for (int i = 0; i < NUM_CONSTRAINTS; i++)
	{
		lhs = 0;
		for (int j = 0; j < NUM_VARIABLES; j++)
			if (Aind[i][j] != 0)
				lhs += Aind[i][j] * vars[j];
		constr[i] = model.addConstr(lhs, sense[i], bind[i]);
	}
	model.update();
	model.getEnv().set(GRB_IntParam_OutputFlag, 0);
}
//*******************************************************************************************
double Push::OptimizeModel()
{
	double counter1 = 0;
	model.optimize();
	if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
	{
		objval = model.get(GRB_DoubleAttr_ObjVal);
		// cout<< "objval"<<endl;
		// cout<< objval<<endl;
		for (int i = 0; i < NUM_VARIABLES; i++)
		{
			solution[i] = vars[i].get(GRB_DoubleAttr_X);
			//~ cout<<vars[i].get(GRB_DoubleAttr_X)<<endl;
			success = true;
		}
	}
	for (int i = 0; i < 2; i++)
	{
		delta_u(i, 0) = solution[i];
	}
	// Retrieve U solutions
	// cout<< NUM_UVARIABLES*NUM_STEPS<<endl;
	for (int i = 0; i < NUM_VARIABLES; i++)
	{
		if (i < NUM_UVARIABLES * NUM_STEPS)
		{
			solutionU(i, 0) = solution[i];
		}
		else
		{
			solutionX(counter1, 0) = solution[i];
			counter1 += 1;
		}
	}

	return objval;
}
//*******************************************************************************************
void Push::UpdateICModel(double time, MatrixXd q_slider, MatrixXd q_pusher)
{
	//----------------Find delta_x: Trajectory Tracking-----------------------------------

	// Uncomment this part for trajectory tracking mode
	// Find desired state
	double FlagStick = 0;
	MatrixXd x_des(4, 1);
	// x_des(0) = (time - 1) * 0.05;
	x_des(0) = time * 0.05 + 0.35;
	x_des(1) = 0.0;
	x_des(2) = 0;
	x_des(3) = 0;
	// Find position d
	MatrixXd ripi(2, 1);
	MatrixXd ribi(2, 1);
	MatrixXd ripb(2, 1);
	MatrixXd rbpb(2, 1);
	MatrixXd Cbi(2, 2);
	double theta = q_slider(2);
	double rx, ry;
	Cbi << cos(theta), sin(theta), -sin(theta), cos(theta);
	ripi << q_pusher(0), q_pusher(1); // pusher world-frame
	ribi << q_slider(0), q_slider(1); // slider world-frame
	ripb = ripi - ribi;
	rbpb = Cbi * ripb;
	// rx = rbpb(0);
	rx = -a / 2;
	ry = rbpb(1) - 0.015;

	// Find delta_x
	MatrixXd delta_x(4, 1);
	MatrixXd x_state(4, 1);
	x_state << q_slider, ry;

	delta_x = x_state - x_des;
	// std::cout << "--------------------" << std::endl;
	// printf("rx, ry: %f %f \n", rx, ry);
	// cout << "delta_x" << endl;
	// cout << delta_x << endl;
	// cout << "q_slider" << endl;
	// cout << q_slider << endl;
	// cout << "rbpb" << endl;
	// cout << rbpb(0) << "\t" << rbpb(1) << endl;
	// std::cout << "--------------------" << std::endl;

	//----------------Find delta_x: Target Tracking-----------------------------------

	// // Find position d
	// MatrixXd ripi(2, 1);
	// MatrixXd ribi(2, 1);
	// MatrixXd ripb(2, 1);
	// MatrixXd rbpb(2, 1);
	// MatrixXd Cbi(2, 2);
	// double theta = q_slider(2);
	// double rx, ry;
	// Cbi << cos(theta), sin(theta), -sin(theta), cos(theta); // rotation from body frame to inertial frame (R_z(theta))
	// ripi << q_pusher(0), q_pusher(1); // relative inertial of pusher inertial
	// ribi << q_slider(0), q_slider(1); // relative inertial of body inertial
	// ripb = ripi - ribi; //relative inertial frame of pusher-body
	// rbpb = Cbi * ripb; // relative body frame of pusher-body
	// // rx = rbpb(0);
	// rx = -0.09 / 2;
	// ry = rbpb(1);
	// printf("rx, ry: %f %f \n", rx, ry);
	// // Compute target frame kinematics
	// MatrixXd riti(2, 1);
	// MatrixXd ritb(2, 1);
	// MatrixXd rbtb(2, 1);
	// MatrixXd vcbi(2, 1);
	// // MatrixXd Cci(2,2);
	// double theta_rel;
	// double theta_g;
	// //
	// if (Flag == 0)
	// {
	// 	riti << .2 + .17, 1 * -.11;
	// 	ritb = riti - ribi;
	// 	cout << "riti" << endl;
	// 	cout << riti << endl;
	// 	cout << "ribi" << endl;
	// 	cout << ribi << endl;
	// 	if (ritb.norm() < 0.02)
	// 	{
	// 		Flag = 1;
	// 	}
	// }
	// else if (Flag == 1)
	// {
	// 	riti << .2 + .17, .11;
	// 	ritb = riti - ribi;
	// 	if (ritb.norm() < 0.02)
	// 	{
	// 		Flag = 2;
	// 	}
	// }
	// else if (Flag == 2)
	// {
	// 	riti << 0.07, -0.0035;
	// 	ritb = riti - ribi;
	// 	if (ritb.norm() < 0.15)
	// 	{
	// 		Flag = 3;
	// 	}
	// }
	// rbtb = Cbi * ritb;
	// complex<double> mycomplex(rbtb(0), rbtb(1));
	// mycomplex = log(mycomplex);
	// theta_rel = -mycomplex.imag();
	// theta_g = theta - theta_rel;
	// // Cci << cos(theta), sin(theta), -sin(theta), cos(theta);
	// // vcbi = Cci*
	// // Compute delta x
	// MatrixXd delta_x(4, 1);
	// delta_x << 0, 0, theta_rel, ry;
	// cout << "delta_x" << endl;
	// cout << delta_x << endl;

	//----------------Add IC constraints-----------------------------------
	// Doubles
	double E, E1, E2;
	double epsilon = 0.005;
	double gamma_top = (nu_p * c_ls * c_ls - rx * ry + nu_p * rx * rx) / (c_ls * c_ls + ry * ry - nu_p * rx * ry);
	double gamma_bottom = (-nu_p * c_ls * c_ls - rx * ry - nu_p * rx * rx) / (c_ls * c_ls + ry * ry + nu_p * rx * ry);
	double Aind[Ain.rows()][Ain.cols()];
	double bind[bin.rows()];
	// Matrices
	MatrixXd B1(4, 2);
	MatrixXd B2(4, 2);
	MatrixXd B3(4, 2);
	MatrixXd B(4, 2);
	MatrixXd u_star(2, 1);
	MatrixXd f_star(4, 1);
	MatrixXd f_star1(4, 1);
	MatrixXd f_star2(4, 1);
	MatrixXd f_star3(4, 1);
	MatrixXd F(4, 1);
	MatrixXd F_tilde(4, 1);
	MatrixXd B_tilde(4, 1);
	MatrixXd D(1, 2);
	MatrixXd D1(1, 2);
	MatrixXd D2(1, 2);
	MatrixXd Ain(10, NUM_VARIABLES);
	MatrixXd bin(10, 1);
	MatrixXd Aeq(4, NUM_VARIABLES);
	MatrixXd beq(4, 1);
	MatrixXd Ain1(1, NUM_VARIABLES);
	MatrixXd bin1(1, 1);
	MatrixXd Ain2(1, NUM_VARIABLES);
	MatrixXd bin2(1, 1);

	// Define u_star
	u_star << 0.05, 0;		  // nominal input
	f_star1 << 0.05, 0, 0, 0; // f1(x_star(t0),u_star(t0)) as in (8)
	f_star2 << 0.05, 0.015, -0.5, -0.0375;
	f_star3 << 0.05, -0.015, 0.5, 0.0375;

	B1(0, 0) = (27.0 * cos(theta) + 360.0 * ry * sin(theta)) / (8000.0 * ry * ry + (27.0));
	B1(0, 1) = -(sin(theta) * (ry * ry + 27.0 / 20000)) / (ry * ry + 27.0 / 8000) - (9 * ry * cos(theta)) / (200.0 * (ry * ry + 27.0 / 8000.0));
	B1(1, 0) = (27.0 * sin(theta) - 360 * ry * cos(theta)) / (8000 * ry * ry + 27.0);
	B1(1, 1) = (cos(theta) * (ry * ry + 27.0 / 20000.0)) / (ry * ry + 27.0 / 8000.0) - (9.0 * ry * sin(theta)) / (200.0 * (ry * ry + 27.0 / 8000.0));
	B1(2, 0) = -ry / (ry * ry + 27.0 / 8000.0);
	B1(2, 1) = -360.0 / (8000.0 * ry * ry + 27.0);
	B1(3, 0) = 0;
	B1(3, 1) = 0;

	B2(0, 0) = (sin(theta) * ((9.0 * ry) / 200.0 - (((9.0 * ry) / 200.0 + 81.0 / 80000.0) * (ry * ry + 27.0 / 20000.0)) / (ry * ry + (27.0 * ry) / 2000.0 + 27.0 / 20000.0))) / (ry * ry + 27.0 / 8000.0) - (cos(theta) * ((9.0 * ry * ((9.0 * ry) / 200.0 + 81.0 / 80000.0)) / (200.0 * (ry * ry + (27.0 * ry) / 2000.0 + 27.0 / 20000.0)) - 27.0 / 8000.0)) / (ry * ry + 27.0 / 8000.0);
	B2(0, 1) = 0;
	B2(1, 0) = -(cos(theta) * ((9.0 * ry) / 200.0 - (((9.0 * ry) / 200.0 + 81.0 / 80000.0) * (ry * ry + 27.0 / 20000.0)) / (ry * ry + (27.0 * ry) / 2000.0 + 27.0 / 20000.0))) / (ry * ry + 27.0 / 8000) - (sin(theta) * ((9.0 * ry * ((9.0 * ry) / 200.0 + 81.0 / 80000.0)) / (200.0 * (ry * ry + (27.0 * ry) / 2000.0 + 27.0 / 20000.0)) - 27.0 / 8000.0)) / (ry * ry + 27.0 / 8000.0);
	B2(1, 1) = 0;
	B2(2, 0) = -(160000000.0 * ry * ry * ry + 2160000.0 * ry * ry + 540000.0 * ry + 7290.0) / (160000000.0 * ry * ry * ry * ry + 2160000.0 * ry * ry * ry + 756000.0 * ry * ry + 7290.0 * ry + 729.0);
	B2(2, 1) = 0;
	B2(3, 0) = -(900.0 * ry + 81.0 / 4.0) / (20000.0 * ry * ry + 270.00 * ry + 27.0);
	B2(3, 1) = 1.0;

	B3(0, 0) = (sin(theta) * ((9.0 * ry) / 200.0 - (((9.0 * ry) / 200.0 - 81.0 / 80000.0) * (ry * ry + 27.0 / 20000.0)) / (ry * ry - (27.0 * ry) / 2000.0 + 27.0 / 20000.0))) / (ry * ry + 27.0 / 8000.0) - (cos(theta) * ((9.0 * ry * ((9.0 * ry) / 200.0 - 81.0 / 80000.0)) / (200.0 * (ry * ry - (27.0 * ry) / 2000.0 + 27.0 / 20000.0)) - 27.0 / 8000.0)) / (ry * ry + 27.0 / 8000.0);
	B3(0, 1) = 0;
	B3(1, 0) = -(cos(theta) * ((9.0 * ry) / 200.0 - (((9.0 * ry) / 200.0 - 81.0 / 80000.0) * (ry * ry + 27.0 / 20000)) / (ry * ry - (27.0 * ry) / 2000.0 + 27.0 / 20000.0))) / (ry * ry + 27.0 / 8000.0) - (sin(theta) * ((9.0 * ry * ((9.0 * ry) / 200.0 - 81.0 / 80000.0)) / (200.0 * (ry * ry - (27.0 * ry) / 2000.0 + 27.0 / 20000.0)) - 27.0 / 8000.0)) / (ry * ry + 27.0 / 8000.0);
	B3(1, 1) = 0;
	B3(2, 0) = -(160000000.0 * ry * ry * ry - 2160000.0 * ry * ry + 540000.0 * ry - 7290.0) / (160000000.0 * ry * ry * ry * ry - 2160000.0 * ry * ry * ry + 756000.0 * ry * ry - 7290.0 * ry + 729.0);
	B3(2, 1) = 0;
	B3(3, 0) = -(900.0 * ry - 81.0 / 4.0) / (20000.0 * ry * ry - 270.0 * ry + 27.0);
	B3(3, 1) = 1;
	if (Family == 1)
	{
		B = B1;
		f_star = f_star1;
		D1 << -gamma_top, 1;
		D2 << gamma_bottom, -1;
		E1 = -u_star(1) + gamma_top * u_star(0);
		E2 = u_star(1) - gamma_bottom * u_star(0);
	}
	else if (Family == 2)
	{
		B = B2;
		f_star = f_star2;
		D1 << gamma_top, -1;
		D2 << 0, 0;
		E1 = u_star(1) - gamma_top * u_star(0) - epsilon;
		E2 = 0;
	}
	else
	{
		B = B3;
		f_star = f_star3;
		D1 << -gamma_bottom, 1;
		D2 << 0, 0;
		E1 = -u_star(1) + gamma_bottom * u_star(0) - epsilon;
		E2 = 0;
	}
	// Define F_tilde
	F = B * u_star - f_star;
	F_tilde = delta_x + h_opt * F;
	B_tilde = h_opt * B;
	// Build Dynamic Constraints Aeq, beq  ----------------------------------
	Aeq.setZero(4, NUM_VARIABLES);
	beq.setZero(4, 1);
	MatrixXd Aeqx(NUM_XVARIABLES, NUM_XVARIABLES);
	Aeqx = MatrixXd::Identity(NUM_XVARIABLES, NUM_XVARIABLES);
	// Add blocks in right location
	Aeq.block(0, NUM_UVARIABLES * NUM_STEPS, 4, 4) = Aeqx;
	Aeq.block(0, 0, 4, 2) = -B_tilde;
	beq = F_tilde;
	// Build Motion Cone Constraints -----------------------------------------
	// 1.0 constraint
	Ain1.setZero(1, NUM_VARIABLES);
	bin1.setZero(1, 1);
	Ain1.block(0, 0, 1, 2) = D1;
	bin1 << E1;
	// 2.0 constraint
	Ain2.setZero(1, NUM_VARIABLES);
	bin2.setZero(1, 1);
	Ain2.block(0, 0, 1, 2) = D2;
	bin2 << E2;
	// Stack Matrices
	Ain << Aeq, -Aeq, Ain1, Ain2;
	bin << beq, -beq, bin1, bin2;
	// Convert matrices to arrays
	matrix_to_array(Ain.rows(), Ain.cols(), Aind[0], Ain);
	matrix_to_array(bin.rows(), 1, bind, bin);
	//~ //Add constraints
	for (int i = 0; i < 10; i++)
	{
		lhs = 0;
		senseIC[i] = '<';
		for (int j = 0; j < NUM_VARIABLES; j++)
			if (Aind[i][j] != 0)
				lhs += Aind[i][j] * vars[j];
		constrIC[i] = model.addConstr(lhs, senseIC[i], bind[i]);
	}
	model.update();
}
//~ model.remove(constrIC[0]);
//~ model.update();
//*******************************************************************************************
void Push::RemoveConstraints()
{
	for (int i = 0; i < 10; i++)
	{
		model.remove(constrIC[i]);
	}
}
