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
    MatrixXd *_delta_uMPC;
    MatrixXd *_delta_xMPC;
};
#endif


