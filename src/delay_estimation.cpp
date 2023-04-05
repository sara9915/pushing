#include <ros/ros.h>
#include "pushing/headerFiles.h"

//*********************** Main Program *************************************
int main(int argc, char *argv[])
{

    ros::init(argc, argv, "delay_estimation");
    ros::NodeHandle n;
    ros::Rate loop(40);

    tf::TransformListener listener;
    geometry_msgs::PoseStamped pose_;
    ros::Publisher delay_pub = n.advertise<geometry_msgs::PoseStamped>("/delay", 1);

    while (ros::ok())
    {
        tf::StampedTransform transform;

        try
        {
            listener.waitForTransform("/base_link", "/push_frame", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", "/push_frame",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }

        if (transform.stamp_ != pose_.header.stamp)
        {
            pose_.header.frame_id = "base_link";
            pose_.header.stamp = transform.stamp_;
            pose_.pose.position.x = transform.getOrigin().x();
            pose_.pose.position.y = transform.getOrigin().y();
            pose_.pose.position.z = transform.getOrigin().z();
            delay_pub.publish(pose_);
        }
        // loop.sleep();
        ros::WallDuration(0.0001).sleep();
    }

    return 0;
}