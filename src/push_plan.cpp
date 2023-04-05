#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <grasp_dope/quintic_traj.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>

#include <pushing/push_plan_action_Action.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

bool success_planning_pp = false;
bool success;
double rate = 50; // Hz
std::vector<double> homing;

moveit::planning_interface::MoveGroupInterface::Plan planning_joint(const geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface) //, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    move_group_interface.setPoseTarget(target_pose, "tool0");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setPlanningTime(10);
    move_group_interface.setPlannerId("RRTstarkConfigDefault");

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_STREAM("Planning result: " << success);
    ROS_INFO_STREAM(move_group_interface.getEndEffectorLink());
    return my_plan;
}

auto planning_cartesian(const geometry_msgs::Pose &target_pose, moveit::planning_interface::MoveGroupInterface &move_group_interface) //, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    move_group_interface.setPoseTarget(target_pose, "push_frame");
    move_group_interface.setPlanningTime(2);
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction != 1)
    {
        ROS_INFO_STREAM("Fraction value: " << fraction);
        success = 0;
    }
    else
        success = 1;

    return trajectory;
}

auto get_tool_pose(const geometry_msgs::Pose &pose)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.waitForTransform("/push_frame", "/tool0", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/push_frame", "/tool0",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    Eigen::Quaterniond q_TP(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
    Eigen::Isometry3d T_TP(q_TP); // omogeneous transfrom from tool0 to push_frame
    T_TP.translation().x() = transform.getOrigin().getX();
    T_TP.translation().y() = transform.getOrigin().getY();
    T_TP.translation().z() = transform.getOrigin().getZ();

    Eigen::Quaterniond q_PB(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Isometry3d T_PB(q_PB);
    T_PB.translation().x() = pose.position.x;
    T_PB.translation().y() = pose.position.y;
    T_PB.translation().z() = pose.position.z;

    Eigen::Isometry3d T_TB(T_PB * T_TP);

    geometry_msgs::Pose start_state_pose;
    Eigen::Quaterniond start_state_quat(T_TB.rotation());
    start_state_pose.orientation.w = start_state_quat.w();
    start_state_pose.orientation.x = start_state_quat.x();
    start_state_pose.orientation.y = start_state_quat.y();
    start_state_pose.orientation.z = start_state_quat.z();

    start_state_pose.position.x = T_TB.translation().x();
    start_state_pose.position.y = T_TB.translation().y();
    start_state_pose.position.z = T_TB.translation().z();

    return start_state_pose;
}

void execute_trajectory(const moveit_msgs::RobotTrajectory &my_plan, ros::NodeHandle &nh, bool scale)
{
    ros::Rate loop_rate(rate);
    ros::Publisher joint_cmd_pub = nh.advertise<sensor_msgs::JointState>("/motoman/joint_ll_control", 1);

    sensor_msgs::JointState joint_cmd;
    joint_cmd.position.resize(my_plan.joint_trajectory.points.at(0).positions.size());
    joint_cmd.velocity.resize(my_plan.joint_trajectory.points.at(0).positions.size());
    joint_cmd.name.resize(my_plan.joint_trajectory.points.at(0).positions.size());
    joint_cmd.header = my_plan.joint_trajectory.header;
    joint_cmd.name = my_plan.joint_trajectory.joint_names;

    // coeff_q1 coeff_q2 coeff_q3 ... coeff_q7
    //  a5
    //  a4
    //  ...
    //  a0
    Eigen::Matrix<double, 6, 7> coeff;

    auto num_points_traj = my_plan.joint_trajectory.points.size();
    double tf = 0;
    double t = 0;
    double t0 = 0;
    double scale_factor = 10.0;

    ROS_INFO_STREAM("NUMERO DI PUNTI PIANIFICATI: " << num_points_traj);
    std::vector<sensor_msgs::JointState> joint_cmd_vect;
    for (int j = 0; j < num_points_traj - 1; j++)
    {
        tf = my_plan.joint_trajectory.points[j + 1].time_from_start.toSec() - my_plan.joint_trajectory.points[j].time_from_start.toSec();

        auto qi = my_plan.joint_trajectory.points[j].positions;
        auto qf = my_plan.joint_trajectory.points[j + 1].positions;

        auto qi_dot = my_plan.joint_trajectory.points[j].velocities;
        auto qf_dot = my_plan.joint_trajectory.points[j + 1].velocities;

        auto qi_dot_dot = my_plan.joint_trajectory.points[j].accelerations;
        auto qf_dot_dot = my_plan.joint_trajectory.points[j + 1].accelerations;

        if (scale)
        {
            tf = tf * scale_factor;
            for (int m = 0; m < qi.size(); m++)
            {
                qi_dot[m] = qi_dot[m] / scale_factor;
                qi_dot_dot[m] = qi_dot_dot[m] / pow(scale_factor, 2);

                qf_dot[m] = qf_dot[m] / scale_factor;
                qf_dot_dot[m] = qf_dot_dot[m] / pow(scale_factor, 2);
            }
        }

        update_coeff(coeff, tf, qi, qi_dot, qi_dot_dot, qf, qf_dot, qf_dot_dot, num_points_traj);

        t = 0;
        t0 = ros::Time::now().toSec();
        while (t <= tf)
        {
            t = ros::Time::now().toSec() - t0;
            for (int i = 0; i < qi.size(); i++)
            {
                joint_cmd.position.at(i) = quintic_q(t, coeff, i);
                joint_cmd.velocity.at(i) = quintic_qdot(t, coeff, i);
            }

            joint_cmd_pub.publish(joint_cmd);
            loop_rate.sleep();
        }
    }
}

bool executeCB(const pushing::push_plan_action_GoalConstPtr &goal, actionlib::SimpleActionServer<pushing::push_plan_action_Action> *as, ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *move_group_interface, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface, const moveit::core::JointModelGroup *joint_model_group, moveit::core::RobotStatePtr &kinematic_state)
{
    // create messages that are used to published feedback/result
    pushing::push_plan_action_Feedback feedback;
    pushing::push_plan_action_Result result;

    geometry_msgs::Pose pre_push;
    geometry_msgs::Pose push = goal->push_pose.pose;
    geometry_msgs::Pose post_push;

    // moveit::core::RobotState start_state(*move_group_interface->getCurrentState());

    moveit::planning_interface::MoveGroupInterface::Plan plan_pre_push;
    moveit_msgs::RobotTrajectory plan_push;
    moveit_msgs::RobotTrajectory plan_post_push;

    moveit::planning_interface::MoveGroupInterface::Plan plan_homing;

    ros::NodeHandle temp;
    ros::Publisher push_pub = temp.advertise<geometry_msgs::PoseStamped>("/push_pose", 1);

    pre_push = push;
    pre_push.position.z = pre_push.position.z + 0.10;
    pre_push.position.y = pre_push.position.y + 0.05;
    post_push = pre_push;

    int i = 0;
    ros::Rate loop_rate(1);
    geometry_msgs::PoseStamped pre_push_stamped;
    pre_push_stamped.pose = pre_push;
    pre_push_stamped.header.frame_id = "base_link";
    pre_push_stamped.header.stamp = ros::Time::now();

    ROS_INFO_STREAM("--- pre_push pose --- "
                    << "\n"
                    << pre_push);

    if (as->isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("Preempted");
        as->setPreempted();
        success = false;
        return success;
    }

    /* Planning to pre-grasp pose */

    moveit::core::RobotState start_state(*move_group_interface->getCurrentState());
    move_group_interface->setStartState(start_state);

    plan_pre_push = planning_joint(get_tool_pose(pre_push), *move_group_interface);

    ROS_INFO_STREAM("Result planning pre-push pose: " << success);

    if (success)
    {
        std::cout << "Press Enter to Continue";
        std::cin.ignore();
        push.position.z = push.position.z + 0.01;
        push.position.y = push.position.y + 0.01;
        while (i < 10)
        {
            push_pub.publish(goal->push_pose);
            i++;
            loop_rate.sleep();
        }
        i = 0;

        ROS_INFO_STREAM("--- Push pose ---"
                        << "\n"
                        << push);

        /* Planning to grasp pose */
        // start_state.setFromIK(joint_model_group, get_tool_pose(pre_push));
        start_state.setJointGroupPositions(joint_model_group, plan_pre_push.trajectory_.joint_trajectory.points.back().positions);
        move_group_interface->setStartState(start_state);

        plan_push = planning_cartesian(get_tool_pose(push), *move_group_interface);

        ROS_INFO_STREAM("Result planning push pose: " << success);

        if (success)
        {
            std::cout << "Press Enter to Continue";
            std::cin.ignore();

            success_planning_pp = true;
        }
        else
        {
            ROS_INFO_STREAM("Planning failed...");
        }
    }
    else
    {
        ROS_INFO_STREAM("Planning failed...");
    }

    if (success_planning_pp)
    {
        std::cout << "Press Enter to start executing";
        std::cin.ignore();
        ROS_INFO_STREAM("Executing trajectory pre_push...");
        execute_trajectory(plan_pre_push.trajectory_, *nh, false);

        std::cout << "Press Enter to Continue";
        std::cin.ignore();

        ROS_INFO_STREAM("Executing trajectory push...");
        execute_trajectory(plan_push, *nh, true);
        std::cout << "Press Enter to Continue";
        std::cin.ignore();
    }

    result.success = success;
    as->setSucceeded(result);
    bool success_homing = false;
    success_planning_pp = false;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pushing_plan");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    /* Parametri per la costruzione della scena */
    static const std::string PLANNING_GROUP = "yaskawa_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    auto start_joints_values = ros::topic::waitForMessage<sensor_msgs::JointState>("/motoman/joint_states");
    // auto start_joints_values = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    std::vector<double> joints_values;

    for (auto element : start_joints_values->position)
    {
        joints_values.push_back(element);
        ROS_INFO_STREAM(element);
    }
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    // std::vector<double> joints_values_tmp;
    // for (int i = 0; i < 7; i++)
    // {
    //     joints_values_tmp.push_back(joints_values[i]);
    // }
    // kinematic_state->setJointGroupPositions(joint_model_group, joints_values_tmp);

    kinematic_state->setJointGroupPositions(joint_model_group, joints_values);
    homing = move_group_interface.getCurrentJointValues();

    for (auto element : move_group_interface.getCurrentJointValues())
    {
        ROS_INFO_STREAM(element);
    }

    /* Creazione del ros action */
    actionlib::SimpleActionServer<pushing::push_plan_action_Action> as(nh, "pushing_planner", boost::bind(&executeCB, _1, &as, &nh, &move_group_interface, &planning_scene_interface, joint_model_group, kinematic_state), false); // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    as.start();

    ros::waitForShutdown();

    return 0;
}