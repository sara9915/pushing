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
    MatrixXd *pdelta_uMPC = my_data->_delta_uMPC;
    MatrixXd *pdelta_xMPC = my_data->_delta_xMPC;
    double *pTimeGlobal = my_data->_TimeGlobal;

    MatrixXd &q_slider = *pq_slider;
    MatrixXd &q_pusher = *pq_pusher;
    MatrixXd &u_control = *pu_control;
    MatrixXd &delta_uMPC = *pdelta_uMPC;
    MatrixXd &delta_xMPC = *pdelta_xMPC;
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

    // Define local matrix variables
    MatrixXd fval(3, 1);       //[fval1 fval2 fval3]
    MatrixXd _q_slider_(3, 1); //[x y theta]
    MatrixXd _q_pusher_(2, 1); //[rx ry]
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


    // std::cout << "before loop" << std::endl;

    //**********************************************************************
    //************************ Begin Loop **********************************
    //**********************************************************************
    ros::Rate loop(40);
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
            fval1 = 100000;
        }
        try
        {
            fval2 = Up.OptimizeModel();
        }
        catch (...)
        {
            fval2 = 100000;
        }
        try
        {
            fval3 = Down.OptimizeModel();
        }
        catch (...)
        {
            fval3 = 100000;
        }
        // Find best control input
        fval << fval1, fval2, fval3;
        min = fval.minCoeff(&minIndex, &maxCol);
        // Assign new control input to shared variables
        pthread_mutex_lock(&nonBlockMutex);

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
            std::cout << " Sliding Up " << std::endl;
        }
        else
        {
            u_control = Down.delta_u;
            delta_uMPC = Down.solutionU;
            delta_xMPC = Down.solutionX;
            std::cout << " Sliding down " << std::endl;
            // std::cout << delta_uMPC << endl;
        }
        std::cout << "u_control" << endl;
        std::cout << u_control << endl;

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
