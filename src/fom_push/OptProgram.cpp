#include "pushing/headerFiles.h"

using namespace tf;
using namespace std;
using Eigen::MatrixXd;

//********************************************************************
// Optimization Thread
//********************************************************************
void *rriMain(void *thread_arg)
{
    std::cout << "START THREAD" << std::endl;
    struct thread_data *my_data;
    my_data = (struct thread_data *)thread_arg;

    //~ //Define variables from argument pointers
    pthread_mutex_lock(&nonBlockMutex);

    MatrixXd *pq_slider = my_data->_q_slider;
    MatrixXd *pq_pusher = my_data->_q_pusher;
    MatrixXd *pu_control = my_data->_u_control;

    MatrixXd *pu_control_st = my_data->_u_control_st;
    MatrixXd *pu_control_su = my_data->_u_control_su;
    MatrixXd *pu_control_sd = my_data->_u_control_sd;
    
    MatrixXd *pdelta_uMPC = my_data->_delta_uMPC;
    MatrixXd *pdelta_xMPC = my_data->_delta_xMPC;
    MatrixXd *px_sim = my_data->_x_sim;
    double *pTimeGlobal = my_data->_TimeGlobal;

    MatrixXd &q_slider = *pq_slider;
    MatrixXd &q_pusher = *pq_pusher;
    MatrixXd &u_control = *pu_control;

    MatrixXd &u_control_st = *pu_control_st;
    MatrixXd &u_control_sd = *pu_control_sd;
    MatrixXd &u_control_su = *pu_control_su;

    MatrixXd &delta_uMPC = *pdelta_uMPC;
    MatrixXd &delta_xMPC = *pdelta_xMPC;
    MatrixXd &x_sim = *px_sim;
    double &TimeGlobal = *pTimeGlobal;

    pthread_mutex_unlock(&nonBlockMutex);

    //~ //Define local variables

    double fval1; // cost function evaluate for sticking
    double fval2; // cost function evaluate for sliding up
    double fval3; // cost function evaluate for sliding down

    double t_ini;
    double time = 0;
    double counter = 0;
    int minIndex, maxCol;
    float min;

    // DELAY MPC
    const int sample_command_delay_buffer = 5; // delayed sample
    Eigen::Matrix<double, 2, sample_command_delay_buffer> command_delay_buffer;
    command_delay_buffer.setZero(2, sample_command_delay_buffer);
    Eigen::Matrix<double, 2, sample_command_delay_buffer> command_delay_buffer_tmp;
    command_delay_buffer_tmp.setZero(2, sample_command_delay_buffer);

    // SIMULATION DELAY MPC
    Eigen::MatrixXd u_sim;
    u_sim.setZero(2, 1);
    MatrixXd twist(3, 1); twist.setZero(3, 1);
    MatrixXd dx(5, 1); dx.setZero(5, 1);
    MatrixXd vibi(2, 1); vibi.setZero(2, 1);
    MatrixXd q_slider_sim(3, 1); q_slider_sim.setZero(3, 1);
    MatrixXd q_pusher_sim(2, 1); q_pusher_sim.setZero(2, 1);
    
    double theta_sim = 0.0;
    MatrixXd Cbi_sim(2, 2); Cbi_sim.setZero(2, 2);

    // Define local matrix variables
    MatrixXd fval(3, 1); fval.setZero(3, 1);       //[fval1 fval2 fval3]
    MatrixXd _q_slider_(3, 1);  _q_slider_.setZero(3, 1); //[x y theta]
    MatrixXd _q_pusher_(2, 1);  _q_pusher_.setZero(2, 1); //[rx ry]
    MatrixXd _x_sim(5, 1); x_sim.setZero(5, 1);
    double _TimeGlobal_ = 0.0;

    // Define object for 3 family of modes
    Push *pStick;
    Push *pUp;
    Push *pDown;
    // std::cout << "defining Stick" << std::endl;
    pStick = new Push(1);
    pUp = new Push(2);
    pDown = new Push(3);
    Push &Stick = *pStick;
    Push &Up = *pUp;
    Push &Down = *pDown;
    int FlagStick;

    int index_old = -1;
    double factor_pref = 0.0; //0.0020; // 0.001;

    // std::cout << "before loop" << std::endl;

    //**********************************************************************
    //************************ Begin Loop **********************************
    //**********************************************************************
    ros::Rate loop(1 / Stick.h_opt);
    while (time < 50000 && ros::ok())
    {
        // std::cout << "loop control time: " << time << std::endl;
        if (time == 0)
        {
            t_ini = gettime();
        }
        time = gettime() - t_ini;
        // Read state of robot and object from shared variables
        pthread_mutex_lock(&nonBlockMutex);
        _q_slider_ = q_slider;
        _q_pusher_ = q_pusher;
        _TimeGlobal_ = TimeGlobal;
        pthread_mutex_unlock(&nonBlockMutex);

        /**************** SIMULATION DELAY *****************/
        q_slider_sim = _q_slider_;
        q_pusher_sim = _q_pusher_;
        x_sim << q_slider_sim, q_pusher_sim;

        for (int i = 0; i < sample_command_delay_buffer; i++)
        {
            theta_sim = q_slider_sim(2);
            Cbi_sim << cos(theta_sim), sin(theta_sim), -sin(theta_sim), cos(theta_sim);

            // std::cout << command_delay_buffer << std::endl;
            u_sim = command_delay_buffer.block(0, sample_command_delay_buffer - 1 - i, 2, 1);
            // std::cout << u_sim << std::endl;

            twist = Stick.dx_funct(q_slider_sim, q_pusher_sim, u_sim);
            vibi = Cbi_sim.transpose() * twist.block(0, 0, 2, 1);
            dx << vibi, twist(2), u_sim + Stick.u_star;

            x_sim = x_sim + Stick.h_opt * dx;
            q_slider_sim = x_sim.block(0, 0, 3, 1);
            q_pusher_sim = x_sim.block(3, 0, 2, 1);
        }
        _q_slider_ = q_slider_sim;
        _q_pusher_ = q_pusher_sim;
        _TimeGlobal_ = _TimeGlobal_ + Stick.h_opt*sample_command_delay_buffer;

        //***************************************************

        // Update Model
        // std::cout << _TimeGlobal_ << std::endl;
        Stick.UpdateICModel(_TimeGlobal_, _q_slider_, _q_pusher_);
        Down.UpdateICModel(_TimeGlobal_, _q_slider_, _q_pusher_);
        Up.UpdateICModel(_TimeGlobal_, _q_slider_, _q_pusher_);
        // Optimize Models
        try
        {
            fval1 = Stick.OptimizeModel();
        }
        catch (...)
        {
            std::cout << "Opt not feasible" << std::endl;
            fval1 = 100000;
        }
        try
        {
            fval2 = Up.OptimizeModel();
        }
        catch (...)
        {
            std::cout << "Opt not feasible" << std::endl;
            fval2 = 100000;
        }
        try
        {
            fval3 = Down.OptimizeModel();
        }
        catch (...)
        {
            std::cout << "Opt not feasible" << std::endl;
            fval3 = 100000;
        }
        // Find best control input
        fval << fval1, fval2, fval3;

        // LAST PRIVILEGED
        if (index_old != -1)
        {
            fval(index_old) = fval(index_old) - factor_pref;
        }

        min = fval.minCoeff(&minIndex, &maxCol);
        index_old = minIndex;

        // FORCED STICKING
        //  if(abs(fval(minIndex) - fval1) < 0.01)
        //  {
        //      minIndex = 0;
        //      min = fval(minIndex);
        //  }

        // Assign new control input to shared variables
        pthread_mutex_lock(&nonBlockMutex);
        // minIndex = 0;
        u_control_st = Stick.delta_u;
        u_control_sd = Down.delta_u;
        u_control_su = Up.delta_u;
        
        if (minIndex == 0)
        {
            u_control = Stick.delta_u;
            delta_uMPC = Stick.solutionU;
            delta_xMPC = Stick.solutionX;
            
            // std::cout << " Sticking " << std::endl;
        }
        else if (minIndex == 1)
        {
            u_control = Up.delta_u;
            delta_uMPC = Up.solutionU;
            delta_xMPC = Up.solutionX;
            // std::cout << " Sliding Up " << std::endl;
        }
        else
        {
            u_control = Down.delta_u;
            delta_uMPC = Down.solutionU;
            delta_xMPC = Down.solutionX;
            // std::cout << " Sliding down " << std::endl;
        }
        // std::cout << "u_control" << endl;
        // std::cout << u_control << endl;

        // buffer delay
        if (sample_command_delay_buffer > 0)
        {
            command_delay_buffer_tmp.setZero(2, sample_command_delay_buffer);
            command_delay_buffer_tmp.block(0, 1, 2, sample_command_delay_buffer - 1) = command_delay_buffer.block(0, 0, 2, sample_command_delay_buffer - 1);
            command_delay_buffer_tmp.block(0, 0, 2, 1) = u_control;
            command_delay_buffer = command_delay_buffer_tmp;
        }

        // std::cout << command_delay_buffer<< std::endl;
        _x_sim = x_sim;
        // std::cout << "Thread: " << u_control << std::endl;
        pthread_mutex_unlock(&nonBlockMutex);

        // Remove Contraints
        Stick.RemoveConstraints();
        Up.RemoveConstraints();
        Down.RemoveConstraints();
        counter++;
        // std::cout << "tempo loop: " << gettime() - time - t_ini<< std::endl;
        loop.sleep();
    }
    //*********** End Loop **************************************************
    pthread_exit((void *)0);
}
