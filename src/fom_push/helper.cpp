#include "pushing/headerFiles.h"

using namespace std;
using Eigen::MatrixXd;

//*************************************************************************
MatrixXd cross_op(MatrixXd w)
{
	MatrixXd Omega(3, 3);
	Omega << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
	return Omega;
}
//********************************************************************
double gettime()
{
	char fmt[64];
	char buf[64];
	struct timeval tv;
	struct tm *tm;

	double test_sec;
	double test_usec;
	double test_usec2;
	double t_ini;

	gettimeofday(&tv, NULL);
	tm = localtime(&tv.tv_sec);
	test_sec = tv.tv_sec;
	test_usec = tv.tv_usec;
	test_usec2 = test_usec / (1000000);
	t_ini = test_sec + test_usec2;

	return t_ini;
}
//******************************
double smooth(double data, float filterVal, double smoothedVal)
{
	if (filterVal > 1)
	{ // check to make sure param's are within range
		filterVal = .99;
	}
	else if (filterVal <= 0)
	{
		filterVal = 0;
	}
	smoothedVal = (data * (1 - filterVal)) + (smoothedVal * filterVal);
	return smoothedVal;
}
//***************************************
void write_file(FILE *myFile, int num_rows, int num_cols, double *A)
{
	int i;
	int j;
	float value;
	const int RowOffset = (0 * num_cols);
	for (j = 0; j < num_cols; j++)
	{
		;
		value = A[RowOffset + j];
		if (j == num_cols - 1)
		{
			fprintf(myFile, "%e \n", value);
		}
		else
		{
			fprintf(myFile, "%e ", value);
		}
	}
}
//***************************************

void write_array_JSON(Json::Value root, int num_rows, int num_cols, double *A)
{
	for (int i = 0; i < num_rows; i++)
	{
		const int RowOffset = (i * num_cols);
		for (int j = 0; j < num_cols; j++)
		{
			A[RowOffset + j] = root[i][j].asDouble();
		}
	}
}
//***************************************
void write_matrix_JSON(Json::Value root, MatrixXd &A)
{
	for (int i = 0; i < A.rows(); i++)
	{
		for (int j = 0; j < A.cols(); j++)
		{
			A(i, j) = root[i][j].asDouble();
		}
	}
}

//***************************************
//This method is used to read row vector from JSON and copy it in a column vector
void write_matrix_JSON_index(Json::Value root, MatrixXd &A, int num_rows, int num_cols)
{
	for (int i = 0; i < num_rows; i++)
	{
		for (int j = 0; j < num_cols; j++)
		{
			A(i, j) = root[j][i].asDouble();
		}
	}
}

//***************************************
void print_array(int num_rows, int num_cols, double *A)
{
	MatrixXd M(num_rows, num_cols);
	array_to_matrix(num_rows, num_cols, A, M);
	cout << M << endl;
}
//***************************************
void array_to_matrix(int num_rows, int num_cols, double *A, MatrixXd &A_matrix)
{
	for (int i = 0; i < num_rows; i++)
	{
		const int RowOffset = (i * num_cols);
		for (int j = 0; j < num_cols; j++)
		{
			A_matrix(i, j) = A[RowOffset + j];
		}
	}
}
//***************************************
void matrix_to_array(int num_rows, int num_cols, double *A, MatrixXd &A_matrix)
{
	for (int i = 0; i < num_rows; i++)
	{
		const int RowOffset = (i * num_cols);
		for (int j = 0; j < num_cols; j++)
		{
			A[RowOffset + j] = A_matrix(i, j);
		}
	}
}
