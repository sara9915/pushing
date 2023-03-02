#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <eigen3/Eigen/Geometry>

#include <pushing/push_as_action_Action.h>
#include <pushing/push_plan_action_Action.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <visp_tracking/tracking_mode_Action.h>
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>

void modify_wrl(const std::string &model_color, const std::string &extension_file, const std::string &folder_file, const std::string &replace, const std::string &replace_with)
{
    std::cout << "Opening: " + folder_file << model_color << extension_file << std::endl;
    std::ifstream input_file(folder_file + model_color + "_original" + extension_file);

    if (input_file.is_open())
    {
        std::string line;
        std::vector<std::string> lines;

        while (std::getline(input_file, line))
        {
            // std::cout << line << std::endl;

            std::string::size_type pos = 0;

            while ((pos = line.find(replace, pos)) != std::string::npos)
            {
                line.replace(pos, line.size(), replace_with);
                pos += replace_with.size();
            }

            lines.push_back(line);
        }

        input_file.close();
        std::ofstream output_file(folder_file + model_color + extension_file);
        if (output_file.is_open())
        {
            for (const auto &i : lines)
            {
                output_file << i << std::endl;
            }
        }
        output_file.close();
    }
    else
    {
        std::cout << "Error opening file" << folder_file << model_color << extension_file;
    }
}

Eigen::Isometry3d getTransform(const std::string &frame1, const std::string &frame2)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform(frame1, frame2, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform(frame1, frame2,
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    Eigen::Quaterniond q_transf(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
    Eigen::Isometry3d T_transf(q_transf); // omogeneous transfrom from tool0 to push_frame
    T_transf.translation().x() = transform.getOrigin().getX();
    T_transf.translation().y() = transform.getOrigin().getY();
    T_transf.translation().z() = transform.getOrigin().getZ();

    return T_transf;
}

geometry_msgs::PoseStamped get_pose_obj_in_camera(const geometry_msgs::PoseStamped &obj_pose)
{
    Eigen::Isometry3d T_CB = getTransform("/base_link", "/camera_color_optical_frame");

    /* Object pose detected with best score */
    Eigen::Vector3d translation_OB(obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z);
    Eigen::Quaterniond orientation_OB(obj_pose.pose.orientation.w, obj_pose.pose.orientation.x, obj_pose.pose.orientation.y, obj_pose.pose.orientation.z);
    Eigen::Isometry3d T_OB(orientation_OB);
    T_OB.translation() = translation_OB;

    /* Calculate frame object-base*/
    Eigen::Isometry3d T_OC;
    T_OC = T_CB.inverse() * T_OB;

    geometry_msgs::PoseStamped pose_obj_camera;
    Eigen::Quaterniond orientation_init(T_OC.rotation());
    pose_obj_camera.header = obj_pose.header;
    pose_obj_camera.header.frame_id = "camera_color_optical_frame";
    pose_obj_camera.pose.position.x = T_OC.translation().x();
    pose_obj_camera.pose.position.y = T_OC.translation().y();
    pose_obj_camera.pose.position.z = T_OC.translation().z();

    pose_obj_camera.pose.orientation.w = orientation_init.w();
    pose_obj_camera.pose.orientation.x = orientation_init.x();
    pose_obj_camera.pose.orientation.y = orientation_init.y();
    pose_obj_camera.pose.orientation.z = orientation_init.z();
    ROS_INFO_STREAM(pose_obj_camera);
    return pose_obj_camera;
}

bool executeCB(const pushing::push_as_action_GoalConstPtr &goal, actionlib::SimpleActionServer<pushing::push_as_action_Action> *as)
{
    bool success = true;
    // create messages that are used to published feedback/result
    pushing::push_as_action_Feedback feedback;
    pushing::push_as_action_Result result;

    feedback.status.data = "Calculating pose for push...";

    if (as->isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("Preempted");
        as->setPreempted();
        success = false;
        return success;
    }

    Eigen::Quaterniond q_pose_obj_ref(goal->pose_obj_ref.pose.orientation.x,goal->pose_obj_ref.pose.orientation.y, goal->pose_obj_ref.pose.orientation.z, goal->pose_obj_ref.pose.orientation.w);
    Eigen::Matrix3d rot_pose_obj_ref;
    rot_pose_obj_ref = q_pose_obj_ref.toRotationMatrix();

    Eigen::Matrix3d rotation_PB;
    rotation_PB << 1, 0, 0,
        0, 0, -1,
        0, 1, 0;

    // rotation_PB << 0, -1, 0,
    //     -1, 0, 0,
    //     0, 0, -1;
    Eigen::Quaterniond q_PB(rotation_PB);

    geometry_msgs::PoseStamped push_pose;
    push_pose.header.stamp = ros::Time::now();
    push_pose.header.frame_id = "base_link";
    push_pose.pose.position = goal->pose_obj_ref.pose.position;
    if(rot_pose_obj_ref(1,1)>0)
    {
        push_pose.pose.position.y = push_pose.pose.position.y + 0.05;
    }
    push_pose.pose.orientation.w = q_PB.w();
    push_pose.pose.orientation.x = q_PB.x();
    push_pose.pose.orientation.y = q_PB.y();
    push_pose.pose.orientation.z = q_PB.z();

    // Save T1W: camera pose in world frame in the initial robot configuration
    Eigen::Isometry3d T_C1B = getTransform("/base_link", "/camera_color_optical_frame");

    // Save T01C: object pose in camera frame when robot is in initial configuration
    geometry_msgs::PoseStamped o1c_pose = get_pose_obj_in_camera(goal->pose_obj_ref);
    Eigen::Quaterniond q_O1C(o1c_pose.pose.orientation.w, o1c_pose.pose.orientation.x, o1c_pose.pose.orientation.y, o1c_pose.pose.orientation.z);
    Eigen::Isometry3d T_O1C(q_O1C);
    T_O1C.translation().x() = o1c_pose.pose.position.x;
    T_O1C.translation().y() = o1c_pose.pose.position.y;
    T_O1C.translation().z() = o1c_pose.pose.position.z;

    /* Calling push plan action server */
    feedback.status.data = "Calling plan push action server...";
    actionlib::SimpleActionClient<pushing::push_plan_action_Action> push_plan_ac("pushing_planner", true);

    ROS_INFO("Waiting for push plan action server to start.");
    push_plan_ac.waitForServer(); // will wait for infinite time
    ROS_INFO("Push plan action server started, sending goal.");
    pushing::push_plan_action_Goal goal_push_plan;

    goal_push_plan.push_pose = push_pose;

    push_plan_ac.sendGoalAndWait(goal_push_plan);
    // ROS_INFO_STREAM(push_plan_ac.getResult()->success);

    if (push_plan_ac.getResult()->success)
    {
        ROS_INFO_STREAM("Calculating new pose obj to tracker...");
        feedback.status.data = "Calculating new pose obj to tracker...";

        /* Calcultate new pose obj to track */
        // Save T2W: camera pose in world frame in the push robot configuration
        Eigen::Isometry3d T_C2B = getTransform("/base_link", "/camera_color_optical_frame");

        Eigen::Isometry3d T_O2C;
        T_O2C = T_C2B.inverse() * (T_C1B * T_O1C);

        geometry_msgs::PoseStamped tracker_init_pose;
        tracker_init_pose.header.frame_id = "camera_color_optical_frame";
        tracker_init_pose.header.stamp = ros::Time::now();
        tracker_init_pose.pose.position.x = T_O2C.translation().x();
        tracker_init_pose.pose.position.y = T_O2C.translation().y();
        tracker_init_pose.pose.position.z = T_O2C.translation().z();

        Eigen::Matrix3d rotation_o2c(T_O2C.rotation());
        Eigen::Quaterniond q_o2c(rotation_o2c);

        tracker_init_pose.pose.orientation.w = q_o2c.w();
        tracker_init_pose.pose.orientation.x = q_o2c.x();
        tracker_init_pose.pose.orientation.y = q_o2c.y();
        tracker_init_pose.pose.orientation.z = q_o2c.z();

        /* Calling tracking action server */
        actionlib::SimpleActionClient<visp_tracking::tracking_mode_Action> ac_tracking("tracker_as", true);

        ROS_INFO("Waiting for tracker action server to start.");
        ac_tracking.waitForServer(); // will wait for infinite time
        ROS_INFO("Tracker action server started, sending goal.");
        visp_tracking::tracking_mode_Goal goal_tracker;

        /* First scale cad model in wrl file */
        std::string model_color = "banana2_centered_46_88_vrt_fc";
        std::string extension_file = ".wrl";
        std::string folder_file = "/home/workstation/dope_ros_ws/src/visp_tracking/src/generic-rgbd/model/banana_6/source/banana/46_88/";

        std::cout << "Replacing string..." << std::endl;
        std::string replace = "  scale 1 1 1";
        std::string replace_with = "  scale " + std::to_string(goal->scale_obj) + " " + std::to_string(goal->scale_obj) + " " + std::to_string(goal->scale_obj);
        std::cout << replace_with << std::endl;
        modify_wrl(model_color, extension_file, folder_file, replace, replace_with);
        std::cout << "Replaced string..." << std::endl;

        goal_tracker.use_depth = true;
        goal_tracker.use_edges = false;
        goal_tracker.use_ktl = false;
        goal_tracker.path_wrl = folder_file + model_color + extension_file; //"/home/workstation/dope_ros_ws/src/visp_tracking/src/generic-rgbd/model/banana_6/source/banana/46_88/banana2_centered_46_88_vrt_fc.wrl";
        goal_tracker.initial_pose = tracker_init_pose;
        /* Tracker activate */
        ac_tracking.sendGoal(goal_tracker);
        // ROS_INFO_STREAM(ac_tracking.getResult()->success);
    }
    else
    {
        ROS_INFO_STREAM("Error push_plan");
        success = false;
    }

    result.success = success;
    as->setSucceeded(result);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "push_as");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    /* Creazione del ros action */
    actionlib::SimpleActionServer<pushing::push_as_action_Action> as(nh, "push_as", boost::bind(&executeCB, _1, &as), false); // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    as.start();

    ros::waitForShutdown();

    return 0;
}