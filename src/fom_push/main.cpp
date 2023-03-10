//~ //System
#include "pushing/headerFiles.h"

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
double Flag = 0;

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
bool getMPCPose(const geometry_msgs::Pose2D::ConstPtr &mpc_pose, MatrixXd *q_slider)
{
    *q_slider << mpc_pose->x, mpc_pose->y, mpc_pose->theta;
    return true;
}

bool getRobotPose(MatrixXd &q_pusher)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.waitForTransform("/push_frame", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/push_frame", "/base_link",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    q_pusher(0) = transform.getOrigin().getX();
    q_pusher(1) = transform.getOrigin().getY();

    return true;
}

//*********************** Main Program *************************************
int main(int argc, char *argv[])
{
    //~Ros parameters---------------------------------------------------------------------------------------
    ros::init(argc, argv, "push_control");
    ros::NodeHandle n;
    tf::TransformListener listener;

    //********Define Local Variables-------------------------------------------------------------------------
    // Mutex
    pthread_mutex_init(&nonBlockMutex, NULL);
    pthread_t rriThread;
    pthread_attr_t attrR;
    pthread_attr_init(&attrR);
    pthread_attr_setdetachstate(&attrR, PTHREAD_CREATE_JOINABLE);

    // Matrices
    MatrixXd q_pusher(2, 1); // rx ry
    MatrixXd _q_pusher_sensor(2, 1);
    MatrixXd q_slider(3, 1);  // x y theta
    MatrixXd u_control(2, 1); // vn vt
    MatrixXd delta_uMPC(2, NUM_STEPS);
    MatrixXd delta_xMPC(4, NUM_STEPS);
    MatrixXd _q_pusher(2, 1);
    MatrixXd _q_slider(3, 1);
    MatrixXd _u_control(2, 1);
    MatrixXd _delta_uMPC(2 * NUM_STEPS, 1);
    MatrixXd _delta_xMPC(4 * NUM_STEPS, 1);
    MatrixXd vipi(2, 1);
    MatrixXd vbpi(2, 1);
    MatrixXd Cbi(2, 2);

    // Integers
    int tmp = 0;
    int lv1 = 0;

    // Doubles
    double z_tcp, x_tcp, y_tcp;    // ROBOT
    double _z_tcp, _x_tcp, _y_tcp; // ROBOT
    double time, t_ini;
    double theta;
    double h = 1.0f / 1000;
    double control_rate = 30; // Hz
    double exp_time = 150;    //[s]

    // Booleans
    bool has_robot = false;
    bool has_mpc_pose = false;
    bool has_vicon_vel = false;

    // Variable to pass to thread
    thread_data_array[0]._q_pusher = &q_pusher;
    thread_data_array[0]._q_slider = &q_slider;
    thread_data_array[0]._u_control = &u_control;
    thread_data_array[0]._delta_uMPC = &delta_uMPC;
    thread_data_array[0]._delta_xMPC = &delta_xMPC;

    // Subscriber
    ros::Subscriber mpc_pose_sub = n.subscribe<geometry_msgs::Pose2D>("/mpc_pose", 1, boost::bind(&getMPCPose, _1, &q_slider));

    pushing::plane_command_Goal goal_plane_command;

    //-------------------------------------------------------------------------------------------------------------------------------
    // Check Tracker and Robot
    std::cout << "Check tracker and robot state..." << std::endl;
    while (!has_mpc_pose && ros::ok())
    {
        if (mpc_pose_sub.getNumPublishers() > 0)
        {
            ros::spinOnce();
            has_mpc_pose = true;
        }
    }
    has_robot = getRobotPose(q_pusher);

    // Print q_pusher and q_slider
    std::cout << "q_pusher" << endl;
    std::cout << q_pusher << endl;
    std::cout << "q_slider" << endl;
    std::cout << q_slider << endl;
    //  return 0;

    // Create Thread------------------------------------------------------------------------------------------------------
    pthread_create(&rriThread, &attrR, rriMain, (void *)&thread_data_array[0]);

    // Loop (Give time to thread to initialize)-------------------------------------------------------------------
    for (int i = 0; i < 1000; i++)
    {
        std::cout << " In Second loop " << i << std::endl;
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
        printf("q_slider %f %f %f \n", q_slider(0), q_slider(1), q_slider(2));
        getRobotPose(q_pusher);
        // Assign local variables
        _q_slider = q_slider;
        _q_pusher = q_pusher;
        _u_control = u_control;
        _delta_uMPC = delta_uMPC;
        _delta_xMPC = delta_xMPC;
        TimeGlobal = time;
        x_tcp = q_pusher(0);
        y_tcp = q_pusher(1);

        pthread_mutex_unlock(&nonBlockMutex);

        // Position Control Parameters --------------------------------------------------------------------------------------------------
        if (time <= 1)
        {
            x_tcp = x_tcp;
        }
        else
        {
            if (x_tcp > 0.55 or Flag == 3)
            {
                vipi(0) = 0;
                vipi(1) = 0;
            }
            else
            {
                // Convert u_control from body to intertial reference frame
                theta = _q_slider(2);
                Cbi << cos(theta), sin(theta), -sin(theta), cos(theta);
                vbpi(0) = _u_control(0) * 1 + 0.05 * 1;
                vbpi(1) = _u_control(1) * 1;   // body velocity
                vipi = Cbi.transpose() * vbpi; // inertial velocity
            }
            // Euler integration with h sample time of 0.001
            x_tcp = x_tcp + h * vbpi(0); // x robot position in base frame
            y_tcp = y_tcp + h * vbpi(1); // y robot position in base frame

            //updateJson(_x_tcp, _y_tcp, x_tcp, y_tcp, _q_slider, _u_control, _delta_uMPC, _delta_xMPC, vipi);
        }

        /********************************************************
         *            Calling plane command action server
         *********************************************************/
        actionlib::SimpleActionClient<pushing::plane_command_Action> plane_command_ac("plane_command_robot", true);

        goal_plane_command.pusher_position.x = x_tcp;
        goal_plane_command.pusher_position.y = y_tcp;
        plane_command_ac.sendGoal(goal_plane_command);


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
