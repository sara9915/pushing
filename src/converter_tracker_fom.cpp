#include "ros/ros.h"
#include <std_srvs/SetBool.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Geometry>
#include <math.h> /* fmod */

bool activate_srv = false;
bool first = true;
geometry_msgs::PoseStamped tracker_pose;

bool converter(std_srvs::SetBool::Request &req,
               std_srvs::SetBool::Response &res)
{
    res.success = true;
    activate_srv = req.data;
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
    geometry_msgs::PoseStamped slider_slider0_pose;
    ros::Publisher mpc_pose_pub = nh.advertise<geometry_msgs::Pose2D>("mpc_pose", 1);
    ros::Publisher tracker_cam_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("tracker_cam_pose", 1);
    ros::Publisher slider_slider0_pub = nh.advertise<geometry_msgs::PoseStamped>("slider_slider0_pose", 1);
    ros::Subscriber tracker_pose_sub = nh.subscribe("/tracker_pose", 1, callback);
    Eigen::Isometry3d TT1B; // homogenous matrix from tacker pose to base link
    Eigen::Isometry3d TT1_0_B_;
    Eigen::Vector3d rpw_tracker;

    ros::Rate loop_rate_2(80);
    tf::TransformListener listener;

    // Initial 2D pose
    double x_start = 0.0;
    double y_start = 0.0;
    double theta_start = 0.0;
    double roll, pitch, yaw;

    Eigen::Isometry3d T_T1_T1_0;
    int i = 0;

    while (ros::ok() && activate_srv)
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

        // Transform TTB in inertial frame for mpc pushing
        // First change cad frame rotating it (ONLY FOR BANANA, it depends on the cad frame)
        Eigen::Matrix3d rotation_T1_T;
        // rotation_T1_T << 0, 0, 1,
        //     1, 0, 0,
        //     0, 1, 0;
        rotation_T1_T << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        Eigen::Quaterniond qT1_T(rotation_T1_T);
        Eigen::Isometry3d TT1_T(qT1_T); // homogenous matrix from tracker to camera color optical frame
        TT1_T.translation().x() = 0;
        TT1_T.translation().y() = 0;
        TT1_T.translation().z() = 0;

        TT1B = TCB * TTC * TT1_T;

        // publish tracker pose in base link frame
        tracker_cam_pose.header.frame_id = "base_link";
        tracker_cam_pose.header.stamp = ros::Time::now();
        tracker_cam_pose.pose.position.x = TT1B.translation().x();
        tracker_cam_pose.pose.position.y = TT1B.translation().y();
        tracker_cam_pose.pose.position.z = TT1B.translation().z();
        Eigen::Quaterniond q_T1B(TT1B.rotation());
        tracker_cam_pose.pose.orientation.w = q_T1B.w();
        tracker_cam_pose.pose.orientation.x = q_T1B.x();
        tracker_cam_pose.pose.orientation.y = q_T1B.y();
        tracker_cam_pose.pose.orientation.z = q_T1B.z();
        tracker_cam_pose_pub.publish(tracker_cam_pose);

        if (first || i < 100)
        {

            Eigen::Quaterniond qT1B_p(TT1B.rotation());
            tf::Quaternion q_p(qT1B_p.x(), qT1B_p.y(), qT1B_p.z(), qT1B_p.w());
            tf::Matrix3x3 m_p(q_p);
            m_p.getRPY(roll, pitch, yaw);
            Eigen::Matrix3d m_z_thetap(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond q_thetap(m_z_thetap);
            Eigen::Isometry3d TT1_0_B(q_thetap);
            TT1_0_B.translation().x() = TT1B.translation().x();
            TT1_0_B.translation().y() = TT1B.translation().y();
            TT1_0_B.translation().z() = 0.0;
            TT1_0_B_ = TT1_0_B;
            first = false;
            i++;
        }
        else
        {
            // tf broadcaster for slider0
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(TT1_0_B_.translation().x(), TT1_0_B_.translation().y(), TT1_0_B_.translation().z()));
            Eigen::Quaterniond q_TT1_0_B(TT1_0_B_.rotation());
            tf::Quaternion q(q_TT1_0_B.x(), q_TT1_0_B.y(), q_TT1_0_B.z(), q_TT1_0_B.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "slider0"));

            Eigen::Quaterniond TT1B_p(TT1B.rotation());
            tf::Quaternion q_p(TT1B_p.x(), TT1B_p.y(), TT1B_p.z(), TT1B_p.w());
            tf::Matrix3x3 m_p(q_p);
            m_p.getRPY(roll, pitch, yaw);
            Eigen::Matrix3d m_z_thetap(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond q_thetap(m_z_thetap);
            Eigen::Isometry3d TT1_B_(q_thetap);
            TT1_B_.translation().x() = TT1B.translation().x();
            TT1_B_.translation().y() = TT1B.translation().y();
            TT1_B_.translation().z() = 0.0;

            T_T1_T1_0 = TT1_0_B_.inverse() * TT1_B_;

            // Update mpc_pose
            mpc_pose.x = T_T1_T1_0.translation().x();
            mpc_pose.y = T_T1_T1_0.translation().y();
            Eigen::Quaterniond q_T1_T1_0(T_T1_T1_0.rotation());
            tf::Quaternion q_(q_T1_T1_0.x(), q_T1_T1_0.y(), q_T1_T1_0.z(), q_T1_T1_0.w());
            tf::Matrix3x3 m_(q_);
            m_.getRPY(roll, pitch, yaw);
            mpc_pose.theta = yaw; // x - yaw

            Eigen::Matrix3d m_z_theta(Eigen::AngleAxisd(mpc_pose.theta, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond q_theta(m_z_theta);

            slider_slider0_pose.header.frame_id = "base_link";
            slider_slider0_pose.header.stamp = ros::Time::now();
            slider_slider0_pose.pose.position.x = TT1B.translation().x();
            slider_slider0_pose.pose.position.y = TT1B.translation().y();
            slider_slider0_pose.pose.position.z = 0;
            slider_slider0_pose.pose.orientation.w = q_thetap.w();
            slider_slider0_pose.pose.orientation.x = q_thetap.x();
            slider_slider0_pose.pose.orientation.y = q_thetap.y();
            slider_slider0_pose.pose.orientation.z = q_thetap.z();

            mpc_pose_pub.publish(mpc_pose);
            slider_slider0_pub.publish(slider_slider0_pose);
        }

        loop_rate_2.sleep();
    }

    return 0;
}