/*
 */

//~ using namespace std;
using Eigen::MatrixXd;
//~ using Eigen::ArrayXf;

//~ #include "json/json.h"
 
#ifndef STRUCTURE_H
#define STRUCTURE_H

//**************
struct thread_data{
    MatrixXd *_q_pusher;
    MatrixXd *_q_slider;
    MatrixXd *_u_control;
    MatrixXd *_u_control_sd;
    MatrixXd *_u_control_su;
    MatrixXd *_u_control_st;
    MatrixXd *_delta_uMPC;
    MatrixXd *_delta_xMPC;
    double *_TimeGlobal;
    MatrixXd *_x_sim;
};
#endif


