/*
 */
#include "json/json.h"
#include <Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;
using Eigen::ArrayXf;

#ifndef HELPER_H
#define HELPER_H


MatrixXd cross_op(MatrixXd w);
double gettime();
double smooth(double data, float filterVal, double smoothedVal);
void write_file(FILE* myFile, int num_rows, int num_cols, double *A);
//~ void write_matrix(Json::Value root, double *A);
void write_array_JSON(Json::Value root, int num_rows, int num_cols, double *A);
void write_matrix_JSON(Json::Value root, MatrixXd &A);
void print_array(int num_rows, int num_cols, double *A);
void array_to_matrix(int num_rows, int num_cols, double *A, MatrixXd &A_matrix);
void matrix_to_array(int num_rows, int num_cols, double *A, MatrixXd &A_matrix);
#endif


