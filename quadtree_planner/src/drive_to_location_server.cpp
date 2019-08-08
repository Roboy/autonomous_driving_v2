//
// Created by maximilian on 08.08.19.
//
#include "ros/ros.h"
#include "roboy_cognition_msgs/DriveToLocation.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

int eta = 0;
std_msgs::String error_message;
ros::Publisher navGoalPublisher_;
bool eta_received = false;
bool error_message_received = false;

void etaCallback(const std_msgs::Int16::ConstPtr& msg)
{
    eta = msg->data;
    eta_received = true;
}

void errorMessageCallback(const std_msgs::String::ConstPtr& msg)
{
    error_message.data = msg->data;
    error_message_received = true;
}

bool driveToLocation(roboy_cognition_msgs::DriveToLocation::Request  &req,
         roboy_cognition_msgs::DriveToLocation::Response &res)
{
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/QuadTreePlanner/eta", 100, etaCallback);
    ros::Subscriber sub2 = n.subscribe("/QuadTreePlanner/error_message", 100, errorMessageCallback);
    ROS_INFO("request: %s", req.coordinates.c_str());
    std::string coordinates = req.coordinates.c_str();
    eta_received = false;
    error_message_received = false;
    // Send 2D nav goal according to received coordinates(string) from Luigi
    if(coordinates == "test_location") {
        // Publish simple 2D nav goal
        move_base_msgs::MoveBaseActionGoal ActionGoal;
        move_base_msgs::MoveBaseGoal goal;

        //we'll send a goal to the robot to move 1 meter forward
        ActionGoal.header.frame_id = "";
        ActionGoal.header.stamp = ros::Time::now();

        ActionGoal.goal.target_pose.header.frame_id = "map";
        ActionGoal.goal.target_pose.header.stamp = ros::Time::now();

        ActionGoal.goal.target_pose.pose.position.x = 17.0;
        ActionGoal.goal.target_pose.pose.position.y = 1.0;
        ActionGoal.goal.target_pose.pose.orientation.w = 1.0;

        navGoalPublisher_.publish(ActionGoal);
    }

    // Wait for messages
    while( (!eta_received) || (!error_message_received) ) {
        ros::spinOnce();;
    }

    res.eta = eta;    // Subscribes to eta from QuadTreePlanner
    res.error_message = error_message.data; // Subscribes to error_message from QuadTreePlanner
    ROS_INFO("sending back response eta: [%i], error_message: [%s]", res.eta, res.error_message.c_str());
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_driving");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("autonomous_driving", driveToLocation);
    navGoalPublisher_ = n.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal",1);
    ROS_INFO("Ready to provide service DriveToLocation.");
    ros::spin();

    return 0;
}