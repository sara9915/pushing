#include "ros/ros.h"
#include <std_srvs/Trigger.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Geometry>
#include <math.h> /* fmod */

bool activate_srv = false;
geometry_msgs::PoseStamped tracker_pose;

bool converter(std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res)
{
    res.success = true;
    activate_srv = true;
    return true;
}

void callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tracker_pose.header = msg->header;
    tracker_pose.pose = msg->pose;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "converter_tracker_fom");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::ServiceServer service =
        nh.advertiseService("track_to_fom", converter);

    while (!activate_srv)
    {
        ros::spinOnce();
        loop_rate.sleep();
        // Wait until the service being active
    }

    //******************** SERVICE ACTIVATED ****************************
    // Create msg for FOM_MPC
    geometry_msgs::Pose2D mpc_pose;
    geometry_msgs::PoseStamped tracker_cam_pose;
    ros::Publisher mpc_pose_pub = nh.advertise<geometry_msgs::Pose2D>("mpc_pose", 1);
    ros::Publisher tracker_cam_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("tracker_cam_pose", 1);
    ros::Subscriber tracker_pose_sub = nh.subscribe("/tracker_pose", 1, callback);
    Eigen::Isometry3d TTB; // homogenous matrix from tacker pose to base link
    Eigen::Vector3d rpw_tracker;

    ros::Rate loop_rate_2(40);
    tf::TransformListener listener;
    while (ros::ok())
    {
        ros::spinOnce();
        tf::StampedTransform transform;
        // Read tracker pose in world frame through tf
        try
        {
            listener.waitForTransform("/base_link", "/camera_color_optical_frame", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/base_link", "/camera_color_optical_frame",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        ////homogenous matriz from camera color optical frame to base link
        Eigen::Quaterniond qCB(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
        Eigen::Isometry3d TCB(qCB);
        TCB.translation().x() = transform.getOrigin().getX();
        TCB.translation().y() = transform.getOrigin().getY();
        TCB.translation().z() = transform.getOrigin().getZ();

        // homogeneous matrix from tracker pose to camera color optical frame
        Eigen::Quaterniond qTC(tracker_pose.pose.orientation.w, tracker_pose.pose.orientation.x, tracker_pose.pose.orientation.y, tracker_pose.pose.orientation.z);
        Eigen::Isometry3d TTC(qTC); // homogenous matrix from tracker to camera color optical frame
        TTC.translation().x() = tracker_pose.pose.position.x;
        TTC.translation().y() = tracker_pose.pose.position.y;
        TTC.translation().z() = tracker_pose.pose.position.z;

        TTB = TCB * TTC;

        tracker_cam_pose.header.frame_id = "base_link";
        tracker_cam_pose.header.stamp = ros::Time::now();
        tracker_cam_pose.pose.position.x = TTB.translation().x();
        tracker_cam_pose.pose.position.y = TTB.translation().y();
        tracker_cam_pose.pose.position.z = TTB.translation().z();
        Eigen::Quaterniond q_TB(TTB.rotation());
        tracker_cam_pose.pose.orientation.w = q_TB.w();
        tracker_cam_pose.pose.orientation.x = q_TB.x();
        tracker_cam_pose.pose.orientation.y = q_TB.y();
        tracker_cam_pose.pose.orientation.z = q_TB.z();

        tracker_cam_pose_pub.publish(tracker_cam_pose);

        tf::Quaternion q(q_TB.x(), q_TB.y(), q_TB.z(), q_TB.w());
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Update mpc_pose
        mpc_pose.x = TTB.translation().x();
        mpc_pose.y = TTB.translation().y();

        // rpw_tracker = TTB.rotation().eulerAngles(2, 1, 0);
        // ROS_INFO_STREAM(rpw_tracker);

        // rpw_tracker(0) = fmod(rpw_tracker(0), (2 * M_PI));
        // mpc_pose.x = roll;     // z - roll
        // mpc_pose.y = pitch;     // y - pitch
        mpc_pose.theta = yaw; // x - yaw
        
        mpc_pose_pub.publish(mpc_pose);
        //loop_rate_2.sleep();
    }

    return 0;
}