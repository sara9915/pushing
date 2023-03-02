#include "ros/ros.h"
#include <std_srvs/Trigger.h>
#include "grasp_dope/desired_grasp_pose_activate.h"
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include "pushing/push_as_action_Action.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client_action");
    ros::NodeHandle nh;

    /*Activating get_grasp_pose service*/
    std::cout << "Press Enter to Start";
    std::cin.ignore();
    ROS_INFO_STREAM("--- Activating get_grasp_pose service ---");
    ros::service::waitForService("/get_grasp_pose_service");
    grasp_dope::desired_grasp_pose_activate::Request req;
    req.activate = true;
    grasp_dope::desired_grasp_pose_activate::Response res;
    if (!ros::service::call<grasp_dope::desired_grasp_pose_activate::Request, grasp_dope::desired_grasp_pose_activate::Response>("/get_grasp_pose_service", req, res))
    {
        ROS_INFO_STREAM("Error activating service get_grasp_pose...");
        return -1;
    }
    geometry_msgs::PoseStamped grasp_pose = res.refined_pose;

    /* Calling planning action server */
    actionlib::SimpleActionClient<pushing::push_as_action_Action> ac_planning("push_as", true);

    ROS_INFO("Waiting for push action server to start.");
    ac_planning.waitForServer(); // will wait for infinite time
    ROS_INFO("Push action server started, sending goal.");
    pushing::push_as_action_Goal goal_push_as;
    
    goal_push_as.pose_obj_ref = grasp_pose;
    goal_push_as.scale_obj = res.scale_obj;

    /* Planning and execute to pre_grasp_pose*/
    ac_planning.sendGoalAndWait(goal_push_as);
    ROS_INFO_STREAM(ac_planning.getResult()->success);

    // ros::spin();
    return 0;
}