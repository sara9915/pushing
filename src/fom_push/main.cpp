//~ //System
#include "pushing/headerFiles.h"
#include "pushing/debugger.h"

// Define shortcuts
using namespace tf;
using namespace std;
using Eigen::MatrixXd;

//***********************Define Global Variables*************************************
// Thread
pthread_mutex_t nonBlockMutex;
// Structures
struct thread_data thread_data_array[1];
// Doubles
double TimeGlobal = 0;

/*// JSON Variables
Json::StyledWriter styledWriter;
Json::Value JsonOutput;
Json::Value timeJSON;
Json::Value q_sliderJSON;
Json::Value q_pusher_sensedJSON;
Json::Value q_pusher_commandedJSON;
Json::Value u_controlJSON;
Json::Value delta_uMPCJSON;
Json::Value delta_xMPCJSON;
Json::Value vipiJSON;

void updateJson(const double &_x_tcp, const double &_y_tcp, const double &x_tcp, const double &y_tcp, const MatrixXd &_q_slider, const MatrixXd &_u_control, const MatrixXd &_delta_uMPC, const MatrixXd &_delta_xMPC, const MatrixXd &vipi)
{
    // Update JSON Arrays
    timeJSON.append(time);
    q_pusher_sensedJSON[0].append(_x_tcp);
    q_pusher_sensedJSON[1].append(_y_tcp);
    q_pusher_commandedJSON[0].append(x_tcp);
    q_pusher_commandedJSON[1].append(y_tcp);
    for (int j = 0; j < 3; j++)
    {
        q_sliderJSON[j].append(_q_slider(j));
    }

    for (int j = 0; j < 2; j++)
    {
        u_controlJSON[j].append(_u_control(j));
        vipiJSON[j].append(vipi(j));
    }

    for (int j = 0; j < 70; j++)
    {
        delta_uMPCJSON[j].append(_delta_uMPC(j));
    }
    for (int j = 0; j < 140; j++)
    {
        delta_xMPCJSON[j].append(_delta_xMPC(j));
    }
}
*/

bool getMPCPose(const geometry_msgs::Pose2D::ConstPtr &mpc_pose, MatrixXd *q_slider, bool *has_mpc_pose)
{
    // *q_slider << -mpc_pose->y, mpc_pose->x, (mpc_pose->theta + M_PI_2);
    *q_slider << mpc_pose->x, mpc_pose->y, mpc_pose->theta;
    *has_mpc_pose = true;
    return true;
}

bool getRobotPose(MatrixXd &q_pusher, tf::TransformListener &listener, tf::StampedTransform &T_BS0)
{
    tf::StampedTransform T_PB;
    try
    {
        listener.waitForTransform("/base_link", "/push_frame", ros::Time(0), ros::Duration(0.1));
        listener.lookupTransform("/base_link", "/push_frame",
                                 ros::Time(0), T_PB);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }

    tf::Transform T_PS0;
    T_PS0 = T_BS0 * T_PB;

    q_pusher(0) = T_PS0.getOrigin().getX();
    q_pusher(1) = T_PS0.getOrigin().getY();

    return true;
}

//*********************** Main Program *************************************
int main(int argc, char *argv[])
{
    //~Ros parameters---------------------------------------------------------------------------------------
    ros::init(argc, argv, "push_control");
    ros::NodeHandle n;

    //********Define Local Variables-------------------------------------------------------------------------
    // Mutex
    pthread_mutex_init(&nonBlockMutex, NULL);
    pthread_t rriThread;
    pthread_attr_t attrR;
    pthread_attr_init(&attrR);
    pthread_attr_setdetachstate(&attrR, PTHREAD_CREATE_JOINABLE);

    // Matrices
    MatrixXd x_sim(5, 1);
    x_sim.setZero(5, 1);
    MatrixXd q_pusher(2, 1);
    q_pusher.setZero(2, 1); // rx ry
    MatrixXd _q_pusher_sensor(2, 1);
    _q_pusher_sensor.setZero(2, 1);
    MatrixXd q_slider(3, 1);
    q_slider.setZero(3, 1); // x y theta
    MatrixXd q_slider_init(3, 1);
    q_slider_init.setZero(3, 1);
    MatrixXd u_control(2, 1);
    u_control.setZero(2, 1); // vn vt

    MatrixXd u_control_sd(2, 1);
    u_control_sd.setZero(2, 1); // vn vt
    MatrixXd u_control_su(2, 1);
    u_control_su.setZero(2, 1); // vn vt
    MatrixXd u_control_st(2, 1);
    u_control_st.setZero(2, 1); // vn vt
    MatrixXd _u_control_sd(2, 1);
    u_control_sd.setZero(2, 1); // vn vt
    MatrixXd _u_control_su(2, 1);
    u_control_su.setZero(2, 1); // vn vt
    MatrixXd _u_control_st(2, 1);
    u_control_st.setZero(2, 1); // vn vt

    MatrixXd delta_uMPC(2, NUM_STEPS);
    delta_uMPC.setZero(2, NUM_STEPS);
    MatrixXd delta_xMPC(4, NUM_STEPS);
    delta_xMPC.setZero(4, NUM_STEPS);
    MatrixXd _q_pusher(2, 1);
    _q_pusher.setZero(2, 1);
    MatrixXd _q_slider(3, 1);
    _q_slider.setZero(3, 1);
    MatrixXd _x_sim(5, 1);
    _x_sim.setZero(5, 1);
    MatrixXd _u_control(2, 1);
    _u_control.setZero(2, 1);
    MatrixXd _delta_uMPC(2 * NUM_STEPS, 1);
    _delta_uMPC.setZero(2 * NUM_STEPS, 1);
    MatrixXd _delta_xMPC(4 * NUM_STEPS, 1);
    _delta_xMPC.setZero(4 * NUM_STEPS, 1);
    MatrixXd vipi(2, 1);
    vipi.setZero(2, 1);
    MatrixXd vbpi(2, 1);
    vbpi.setZero(2, 1);
    MatrixXd Cbi(2, 2);
    Cbi.setZero(2, 2);

    Eigen::Isometry3d transf_base_slid0;
    Eigen::Isometry3d transf_slid0_base;
    Eigen::Vector3d vel_pusher_inertial(0.0, 0.0, 0.0);
    Eigen::Vector3d vel_pusher_base(0.0, 0.0, 0.0);

    // Integers
    int tmp = 0;
    int lv1 = 0;

    // Doubles
    double z_tcp, x_tcp, y_tcp;    // ROBOT
    double _z_tcp, _x_tcp, _y_tcp; // ROBOT
    double time, t_ini;
    double theta;
    double control_rate = 40; // Hz
    double h = 1.0f / control_rate;
    double exp_time = 150; //[s]
    double _TimeGlobal = 0.0;
    Eigen::Vector2d u_star;
    u_star << 0.01, 0;

    // Booleans
    bool has_robot = false;
    bool has_mpc_pose = false;
    bool has_vicon_vel = false;

    // Variable to pass to thread
    thread_data_array[0]._q_pusher = &q_pusher;
    thread_data_array[0]._q_slider = &q_slider;
    thread_data_array[0]._u_control = &u_control;

    thread_data_array[0]._u_control_st = &u_control_st;
    thread_data_array[0]._u_control_sd = &u_control_sd;
    thread_data_array[0]._u_control_su = &u_control_su;

    thread_data_array[0]._delta_uMPC = &delta_uMPC;
    thread_data_array[0]._delta_xMPC = &delta_xMPC;
    thread_data_array[0]._TimeGlobal = &TimeGlobal;
    thread_data_array[0]._x_sim = &x_sim;

    MatrixXd ripi(2, 1);
    ripi.setZero(2, 1);
    MatrixXd ribi(2, 1);
    ribi.setZero(2, 1);
    MatrixXd ripb(2, 1);
    ripb.setZero(2, 1);
    MatrixXd rbpb(2, 1);
    rbpb.setZero(2, 1);
    double rx, ry;

    // Subscriber
    tf::TransformListener listener;
    ros::Subscriber mpc_pose_sub = n.subscribe<geometry_msgs::Pose2D>("/mpc_pose", 1, boost::bind(&getMPCPose, _1, &q_slider, &has_mpc_pose));
    ros::Publisher control_pub = n.advertise<geometry_msgs::Pose2D>("/u_control", 1);
    ros::Publisher plane_command_pub = n.advertise<geometry_msgs::Twist>("/command_vel_des", 1);
    ros::Publisher pusher_body_pub = n.advertise<geometry_msgs::Pose2D>("/pusher_body", 1);
    ros::Publisher debugger_pub = n.advertise<pushing::debugger>("/debugger", 1);
    geometry_msgs::Pose2D u_control_msg;
    geometry_msgs::Pose2D pusher_body_msg;
    geometry_msgs::Twist plane_command;

    //-------------------------------------------------------------------------------------------------------------------------------
    // Check Tracker and Robot
    tf::StampedTransform T_BS0;
    try
    {
        listener.waitForTransform("/slider0", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/slider0", "/base_link",
                                 ros::Time(0), T_BS0);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }

    std::cout << "Check tracker and robot state..." << std::endl;
    ros::Rate read_mpc_pose_rate(100);
    while (!has_robot && ros::ok())
    {
        has_robot = getRobotPose(q_pusher, listener, T_BS0);
    }
    while (!has_mpc_pose && ros::ok())
    {
        ros::spinOnce();
        read_mpc_pose_rate.sleep();
    }
    q_slider_init = q_slider;

    // Print q_pusher and q_slider
    std::cout << "q_pusher" << endl;
    std::cout << q_pusher << endl;
    std::cout << "q_slider" << endl;
    std::cout << q_slider << endl;
    //  return 0;

    // Create Action Client robot commands
    actionlib::SimpleActionClient<pushing::plane_command_vel_Action> plane_command_ac("plane_command_robot", true);

    // Create Thread------------------------------------------------------------------------------------------------------
    pthread_create(&rriThread, &attrR, rriMain, (void *)&thread_data_array[0]);

    // Loop (Give time to thread to initialize)-------------------------------------------------------------------
    for (int i = 0; i < 100; i++)
    {
        // std::cout << " In Second loop " << i << std::endl;
        usleep(4e3);
    }

    //*************************************************************************************************************************
    //************** Main Control Loop ****************************************************************************************
    //*************************************************************************************************************************
    ros::Rate r(control_rate);

    for (int i = 0; i < (exp_time * control_rate) && ros::ok(); i++)
    {
        // Get time---------------------------------------------------------------------------------------------------------------------
        if (i == 0)
        {
            t_ini = gettime();
        }
        time = gettime() - t_ini;

        // Read data and store----------------------------------------------------------------------------------------------------------
        pthread_mutex_lock(&nonBlockMutex);

        ros::spinOnce(); // getMPCPose
        getRobotPose(q_pusher, listener, T_BS0);

        // Assign local variables
        _q_slider = q_slider;
        _q_pusher = q_pusher;
        _u_control = u_control;
        _delta_uMPC = delta_uMPC;
        _delta_xMPC = delta_xMPC;
        TimeGlobal = time;
        x_tcp = q_pusher(0);
        y_tcp = q_pusher(1);
        _x_sim = x_sim;
        _u_control_sd = u_control_sd;
        _u_control_su = u_control_su;
        _u_control_st = u_control_st;
        pthread_mutex_unlock(&nonBlockMutex);

        // Position Control Parameters --------------------------------------------------------------------------------------------------
        if (x_tcp > 0.25)
        {
            vipi(0) = 0;
            vipi(1) = 0;
        }
        else
        {
            // Convert u_control from body to intertial reference frame
            theta = _q_slider(2);
            Cbi << cos(theta), sin(theta), -sin(theta), cos(theta);
            vbpi(0) = _u_control(0) + u_star(0);
            vbpi(1) = _u_control(1);       // body velocity
            vipi = Cbi.transpose() * vbpi; // inertial velocity
        }

        // std::cout << "--------------------" << std::endl;
        // std::cout << "x_tcp: " << x_tcp << std::endl;
        // std::cout << "y_tcp: " << y_tcp << std::endl;
        // std::cout << "u_control: " << _u_control << std::endl;
        // std::cout << "--------------------" << std::endl;

        // updateJson(_x_tcp, _y_tcp, x_tcp, y_tcp, _q_slider, _u_control, _delta_uMPC, _delta_xMPC, vipi);

        /********************************************************
         *            Calling plane command action server
         *********************************************************/
        tf::transformTFToEigen(T_BS0, transf_base_slid0);
        transf_slid0_base = transf_base_slid0.inverse();

        vel_pusher_inertial << vipi, 0.0;
        vel_pusher_base = transf_slid0_base.rotation() * vel_pusher_inertial;

        plane_command.angular.x = 0.0;
        plane_command.angular.y = 0.0;
        plane_command.angular.z = 0.0;

        plane_command.linear.x = vel_pusher_base(0);
        plane_command.linear.y = vel_pusher_base(1);
        plane_command.linear.z = 0.0;
        plane_command_pub.publish(plane_command);

        u_control_msg.x = vbpi(0);
        u_control_msg.y = vbpi(1);
        control_pub.publish(u_control_msg);

        ripi << _q_pusher(0), _q_pusher(1); // pusher world-frame
        ribi << _q_slider(0), _q_slider(1); // slider world-frame

        ripb = ripi - ribi;
        rbpb = Cbi * ripb;
        rx = rbpb(0); //-0.082 / 2;
        ry = rbpb(1) - 0.005;
        std::cout << "Main: " << ry << std::endl;

        pusher_body_msg.x = rx;
        pusher_body_msg.y = ry;
        pusher_body_pub.publish(pusher_body_msg);

        // DEBUG PUBLISHER
        pushing::debugger debugger_msgs;
        debugger_msgs.mpc_pose.x = x_sim(0);
        debugger_msgs.mpc_pose.y = x_sim(1);
        debugger_msgs.mpc_pose.theta = x_sim(2);
        debugger_msgs.pusher_body.x = x_sim(3);
        debugger_msgs.pusher_body.y = x_sim(4);
        debugger_msgs.u_control.linear.x = vbpi(0);
        debugger_msgs.u_control.linear.y = vbpi(1);
        debugger_msgs.u_control_st.linear.x = _u_control_st(0);
        debugger_msgs.u_control_st.linear.y = _u_control_st(1);
        debugger_msgs.u_control_su.linear.x = _u_control_su(0);
        debugger_msgs.u_control_su.linear.y = _u_control_su(1);
        debugger_msgs.u_control_sd.linear.x = _u_control_sd(0);
        debugger_msgs.u_control_sd.linear.y = _u_control_sd(1);

        debugger_pub.publish(debugger_msgs);
        r.sleep();
    }

    // Save JSON Output file
    // JsonOutput["timeJSON"] = timeJSON;
    // JsonOutput["q_sliderJSON"] = q_sliderJSON;
    // JsonOutput["q_pusher_sensedJSON"] = q_pusher_sensedJSON;
    // JsonOutput["q_pusher_commandedJSON"] = q_pusher_commandedJSON;
    // JsonOutput["u_controlJSON"] = u_controlJSON;
    // JsonOutput["delta_uMPCJSON"] = delta_uMPCJSON;
    // JsonOutput["delta_xMPCJSON"] = delta_xMPCJSON;
    // JsonOutput["vipiJSON"] = vipiJSON;

    // ofstream myOutput;
    // myOutput.open("/home/mcube/cpush/catkin_ws/src/push_control/data/TargetTracking01.json");
    // myOutput << styledWriter.write(JsonOutput);
    // myOutput.close();

    cout << "End of Program" << endl;

    return 0;
}
