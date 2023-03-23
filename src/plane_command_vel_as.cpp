
#include <ros/ros.h>
#include <pushing/plane_command_vel_Action.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

void read_joint_values(const sensor_msgs::JointState::ConstPtr &msg, const moveit::core::JointModelGroup *joint_model_group, moveit::core::RobotStatePtr &kinematic_state, const ros::V_string joints_name)
{
    std::vector<double> joints_values;

    for (int i = 0; i < joints_name.size(); i++)
    {
        // std::cout << joints_name[i] << std::endl;
        for (int j = 0; j < msg->position.size(); j++)
        {
            // std::cout << msg->name[j]<< std::endl;
            if (msg->name[j] == joints_name.at(i))
            {
                joints_values.push_back(msg->position.at(j));
                break;
            }
        }
    }

    kinematic_state->setJointGroupPositions(joint_model_group, joints_values);
    kinematic_state->update();
}

void executeCB(const pushing::plane_command_vel_GoalConstPtr &goal, actionlib::SimpleActionServer<pushing::plane_command_vel_Action> *as, ros::NodeHandle *nh, const moveit::core::JointModelGroup *joint_model_group, moveit::core::RobotStatePtr &kinematic_state, ros::Publisher *joint_cmd_pub, ros::Publisher *cart_cmd_pub)
{
    pushing::plane_command_vel_Feedback feedback_;
    pushing::plane_command_vel_Result result_;
    Eigen::Matrix<double, 6, 1> vipi;
    vipi << goal->pusher_vel.linear.x, goal->pusher_vel.linear.y, goal->pusher_vel.linear.z, goal->pusher_vel.angular.x, goal->pusher_vel.angular.y, goal->pusher_vel.angular.z; // desired pusher velocity in world frame
    Eigen::VectorXd q_dot;
    Eigen::VectorXd q;
    bool success = true;
    ros::Rate loop(50);
    double euler_frequency = 40; // [Hz]
    double euler_sample_time = 1 / euler_frequency;

    // Reading robot configuration q0
    Eigen::VectorXd joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // get Jacobian
    ROS_INFO_STREAM("vipi: \n"
                    << vipi);
    Eigen::MatrixXd jacobian;
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);  
    if (kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian))
    {
        // Eigen::MatrixXd jacobian_xyz = jacobian.block(0,0,2,jacobian.cols());
        ROS_INFO_STREAM("Jacobian: \n"
                        << jacobian);
        // ROS_INFO_STREAM("\nJacobian_xy: \n" << jacobian_xy);

        try
        {
            // compute q_dot = pinv(J)*vipi
            q_dot = jacobian.completeOrthogonalDecomposition().solve(vipi);
            // Euler integration
            q = joint_values + euler_sample_time * q_dot;
            kinematic_state->setJointGroupPositions(joint_model_group, q);

            // send command
            const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
            sensor_msgs::JointState joint_cmd;

            if (as->isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Preempted");
                // set the action state to preempted
                as->setPreempted();
                success = false;
            }

            ROS_INFO_STREAM("Joints value: \n"
                            << joint_values);// * 180 / M_PI);

            for (std::size_t i = 0; i < q.size(); ++i)
            {
                joint_cmd.name.push_back(joint_names.at(i));
                joint_cmd.position.push_back(q[i]);
                joint_cmd.velocity.push_back(q_dot[i]);
            }

            joint_cmd.header.stamp = ros::Time::now();
            joint_cmd_pub->publish(joint_cmd);
        }
        catch (...)
        {
            ROS_ERROR("ALERT: robot command failure");
            success = false;
        }
    }
    else
    {
        ROS_ERROR("Error calculating Jacobian");
        success = false;
    }

    result_.success = success;
    ROS_INFO("Plane Command Action Server terminated with: %d", success);
    // set the action state to succeeded
    as->setSucceeded(result_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plane_command_vel_as");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "yaskawa_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    ROS_INFO_STREAM("Model frame: " << kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    ros::Subscriber joints_sub = nh.subscribe<sensor_msgs::JointState>("/full_joint_states", 1, boost::bind(&read_joint_values, _1, joint_model_group, kinematic_state, joint_names));
    ros::Publisher joint_cmd_pub = nh.advertise<sensor_msgs::JointState>("/motoman/joint_ll_control", 1);
    ros::Publisher cart_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_control", 1);

    /* Creazione del ros action */
    actionlib::SimpleActionServer<pushing::plane_command_vel_Action> as(nh, "plane_command_robot", boost::bind(&executeCB, _1, &as, &nh, joint_model_group, kinematic_state, &joint_cmd_pub, &cart_cmd_pub), false); // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    as.start();

    ros::waitForShutdown();

    return 0;
}