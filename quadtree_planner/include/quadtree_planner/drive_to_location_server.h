//
// Created by maximilian on 01.09.19.
//

#ifndef SRC_DRIVE_TO_LOCATION_SERVER_H
#define SRC_DRIVE_TO_LOCATION_SERVER_H

/**
 * Callback that is called when the eta ROS message is received from the quadtreeplanner node
 * @param msg pointer to eta ROS message
 */
void etaCallback(const std_msgs::Int16::ConstPtr& msg);

/**
 * Callback that is called when the error_message ROS message is received from the quadtreeplanner node
 * @param msg pointer to error_message ROS message
 */
void errorMessageCallback(const std_msgs::String::ConstPtr& msg);

/**
 * Callback that is called when the pathFound ROS message is received from the quadtreeplanner node
 * @param msg pointer to pathFound ROS message
 */
void pathFoundMessageCallback(const std_msgs::Bool::ConstPtr& msg);

/**
 * This function is called when the ROS service DriveToLocation.srv is requested.
 * @param req request by the caller of the ROS service DriveToLocation.srv
 * @param res response by the ROS service DriveToLocation.srv
 * @return signalizes successful execution of the service
 */
bool driveToLocation(roboy_cognition_msgs::DriveToLocation::Request  &req,
        roboy_cognition_msgs::DriveToLocation::Response &res);

#endif //SRC_DRIVE_TO_LOCATION_SERVER_H
