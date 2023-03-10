
#include <ros/ros.h>
#include <pushing/plane_command_Action.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

void read_joint_values(const sensor_msgs::JointState::ConstPtr &msg, const moveit::core::JointModelGroup *joint_model_group, moveit::core::RobotStatePtr &kinematic_state, const ros::V_string joints_name)
{
    std::vector<double> joints_values;
    // ROS_INFO_STREAM("Joints Name size: " << joints_name.size());
    // for (auto element : joints_name)
    // {
    //     ROS_INFO_STREAM(element);
    // }
    // ROS_INFO_STREAM("Joints Name size: " << msg->name.size());
    // for (auto element : msg->name)
    // {
    //     ROS_INFO_STREAM(element);
    // }

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
}

void executeCB(const pushing::plane_command_GoalConstPtr &goal, actionlib::SimpleActionServer<pushing::plane_command_Action> *as, ros::NodeHandle *nh, const moveit::core::JointModelGroup *joint_model_group, moveit::core::RobotStatePtr &kinematic_state, ros::Publisher *joint_cmd_pub)
{
    pushing::plane_command_Feedback feedback_;
    pushing::plane_command_Result result_;
    bool success = true;
    ros::Rate loop(50);

    // ros::spinOnce();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (int i = 0; i < joint_values.size(); i++)
    {
        std::cout << joint_values.at(i) * 180 / M_PI << std::endl;
    }
    const Eigen::Isometry3d &push_state = kinematic_state->getGlobalLinkTransform("push_frame");
    Eigen::Isometry3d push_state_updated(push_state);
    push_state_updated.translation().x() = goal->pusher_position.x;
    push_state_updated.translation().y() = goal->pusher_position.y;
    const Eigen::Isometry3d &push_state_updated_ref = push_state_updated;

    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    sensor_msgs::JointState joint_cmd;

    // Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^
    double timeout = 0.1;

    bool found_ik = kinematic_state->setFromIK(joint_model_group, push_state_updated_ref, timeout);

    if (found_ik)
    {
        if (as->isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            // set the action state to preempted
            as->setPreempted();
            success = false;
        }
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_values.size() - 1; ++i)
        {
            ROS_INFO("Joint: %f", joint_values[i] * 180 / M_PI);
            joint_cmd.name.push_back(joint_names.at(i));
            joint_cmd.position.push_back(joint_values.at(i));
            joint_cmd.velocity.push_back(0);
        }

        joint_cmd.header.stamp = ros::Time::now();
        joint_cmd_pub->publish(joint_cmd);
        
    }
    else
    {
        ROS_INFO("Did not find IK solution");
        success = false;
    }

    if (success)
    {
        result_.success = success;
        ROS_INFO("Plane Command Action Server Succeeded");
        // set the action state to succeeded
        as->setSucceeded(result_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plane_coomand_robot_as");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "yaskawa_push";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    ros::Subscriber joints_sub = nh.subscribe<sensor_msgs::JointState>("/full_joint_states", 1, boost::bind(&read_joint_values, _1, joint_model_group, kinematic_state, joint_names));
    ros::Publisher joint_cmd_pub = nh.advertise<sensor_msgs::JointState>("/motoman/joint_ll_control", 1);

    /* Creazione del ros action */
    actionlib::SimpleActionServer<pushing::plane_command_Action> as(nh, "plane_command_robot", boost::bind(&executeCB, _1, &as, &nh, joint_model_group, kinematic_state, &joint_cmd_pub), false); // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    as.start();

    ros::waitForShutdown();

    return 0;
}