//
// Created by maximilian on 08.08.19.
//
#include "ros/ros.h"
#include "roboy_cognition_msgs/DriveToLocation.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

int eta = 0;
std_msgs::String error_message;
std_msgs::Bool pathFound_message;
ros::Publisher navGoalPublisher_;
bool eta_received = false;
bool error_message_received = false;
bool path_found_received = false;

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

void pathFoundMessageCallback(const std_msgs::Bool::ConstPtr& msg)
{
    pathFound_message.data = msg->data;
    path_found_received = true;
}

bool driveToLocation(roboy_cognition_msgs::DriveToLocation::Request  &req,
         roboy_cognition_msgs::DriveToLocation::Response &res)
{
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/QuadTreePlanner/eta", 1, etaCallback);
    ros::Subscriber sub2 = n.subscribe("/QuadTreePlanner/error_message", 1, errorMessageCallback);
    ros::Subscriber sub3 = n.subscribe("/QuadTreePlanner/path_found", 1, pathFoundMessageCallback);
    ROS_INFO("request: %s", req.destination.c_str());
    std::string destination = req.destination.c_str();
    eta_received = false;
    error_message_received = false;
    path_found_received = false;
    bool location_unknown = false;
    // Send 2D nav goal according to received destination (string) from Luigi
    // Publish simple 2D nav goal
    move_base_msgs::MoveBaseActionGoal ActionGoal;
    move_base_msgs::MoveBaseGoal goal;

    // Get Base Link coordinates
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    double base_link_x = 0.0;
    double base_link_y = 0.0;
    try{
        transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(10));
        base_link_x = transformStamped.transform.translation.x;
        base_link_y = transformStamped.transform.translation.y;
        ROS_INFO("Base Link x: %f y: %f", base_link_x, base_link_y);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    ActionGoal.header.frame_id = "";
    ActionGoal.header.stamp = ros::Time::now();
    ActionGoal.goal.target_pose.header.frame_id = "map";
    ActionGoal.goal.target_pose.header.stamp = ros::Time::now();
    // IMPORTANT: Positions are just mock positions as map is not avaible yet
    // ToDo: Replace positions by correct values once the maps are available
    if(destination == "midnightsurprise") {
        ActionGoal.goal.target_pose.pose.position.x = 25.7466831207;
        ActionGoal.goal.target_pose.pose.position.y = -163.38621521;
        ActionGoal.goal.target_pose.pose.orientation.z = -0.529101508046;
        ActionGoal.goal.target_pose.pose.orientation.w = 0.848558539044;
    } else if (destination == "interimsfront") {
        if(base_link_y > ((1.0/3.5)*base_link_x - 18.5) ) {
            // When starting from interimssideutum
            ActionGoal.goal.target_pose.pose.position.x = 47.3372039795;
            ActionGoal.goal.target_pose.pose.position.y = -7.63010787964;
            ActionGoal.goal.target_pose.pose.orientation.z = -0.611320751869;
            ActionGoal.goal.target_pose.pose.orientation.w = 0.79138292775;
        } else {
            // When starting from InterimsSideMensa
            ActionGoal.goal.target_pose.pose.position.x = 46.3632125854;
            ActionGoal.goal.target_pose.pose.position.y = -4.0682258606;
            ActionGoal.goal.target_pose.pose.orientation.z = 0.789524533964;
            ActionGoal.goal.target_pose.pose.orientation.w = 0.613718999436;
        }
        // When starting from interimssideutum
    } else if (destination == "interimssidemensa") {
        ActionGoal.goal.target_pose.pose.position.x = 21.0;
        ActionGoal.goal.target_pose.pose.position.y = 1.0;
        ActionGoal.goal.target_pose.pose.orientation.z = 0.0;
        ActionGoal.goal.target_pose.pose.orientation.w = 1.0;
    } else if (destination == "interimssideutum") {
        ActionGoal.goal.target_pose.pose.position.x = 23.0;
        ActionGoal.goal.target_pose.pose.position.y = 1.0;
        ActionGoal.goal.target_pose.pose.orientation.z = 0.0;
        ActionGoal.goal.target_pose.pose.orientation.w = 1.0;
    } else if (destination == "mwchicco") {
        // Starting from mwfachschaft
        if(base_link_y < -base_link_x) {
            ActionGoal.goal.target_pose.pose.position.x = 1.74308013916;
            ActionGoal.goal.target_pose.pose.position.y = -9.9398727417;
            ActionGoal.goal.target_pose.pose.orientation.z = 0.346074328657;
            ActionGoal.goal.target_pose.pose.orientation.w = 0.938207098164;
        } else {
            // Starting from mwstucafe
            ActionGoal.goal.target_pose.pose.position.x = 7.88451004028;
            ActionGoal.goal.target_pose.pose.position.y = -5.58188390732;
            ActionGoal.goal.target_pose.pose.orientation.z = 0.945949866243;
            ActionGoal.goal.target_pose.pose.orientation.w = -0.3243128899;
        }
    } else if (destination == "mwstucafe") {
        ActionGoal.goal.target_pose.pose.position.x = 76.2451324463;
        ActionGoal.goal.target_pose.pose.position.y = 37.1527862549;
        ActionGoal.goal.target_pose.pose.orientation.z = -0.463249795557;
        ActionGoal.goal.target_pose.pose.orientation.w = 0.886227751154;
    } else if (destination == "mwfachschaft") {
        ActionGoal.goal.target_pose.pose.position.x = -45.2332038879;
        ActionGoal.goal.target_pose.pose.position.y = -39.2914505005;
        ActionGoal.goal.target_pose.pose.orientation.z = 0.892010303234;
        ActionGoal.goal.target_pose.pose.orientation.w = 0.45201506493;
    }  else {
        location_unknown = true;
    }



    if (location_unknown) {
       res.eta = 32767;
       res.error_message = "Location unknown!";
       res.path_found = false;
    }
    else {
       navGoalPublisher_.publish(ActionGoal);
       // Wait for messages
       while ( (!eta_received) || (!error_message_received) || (!path_found_received) ) {
           ros::spinOnce();
       }
        res.eta = eta;    // Subscribes to eta from QuadTreePlanner
        res.error_message = error_message.data; // Subscribes to error_message from QuadTreePlanner
        res.path_found = pathFound_message.data; // Subscribes to pathFound_message from QuadTreePlanner
    }

    std::string path_found_string;
    if(res.path_found == true) {
        path_found_string = "True";
    } else {
        path_found_string = "False";
    }
    ROS_INFO("sending back response eta: [%i], error_message: [%s], path_found [%s]", res.eta, res.error_message.c_str(), path_found_string.c_str());
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