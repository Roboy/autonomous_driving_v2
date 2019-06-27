//
// Created by maximilian on 25.06.19.
//

#include <ros/console.h>
#include "../include/quadtree_planner/quadtree_datastructure.h"

// Insert a cell into the quadtree
int Quadtree_Cell::insert()
{
    // We are at a cell of unit area
    // We cannot subdivide this cell further
    if ((topLeft.x - botRight.x) <= 1 &&
        (topLeft.y - botRight.y) <= 1) {
        return -1;
    }

    if (cost == 0) { // no obstacle --> no subdivision of this cell required
        return -2;
    }   else {    // obstacle is contained in the current cell --> subdivision required
            unsigned int costTopLeft = 0; // ToDo: Get real cost of the new cell
            topLeftCell = new Quadtree_Cell( Point(topLeft.x, topLeft.y), Point((topLeft.x + botRight.x) / 2, (topLeft.y + botRight.y) / 2), costTopLeft);
            topLeftCell->insert();
            unsigned int costBotLeft = 0;  // ToDo: Get real cost of the new cell
            botLeftCell = new Quadtree_Cell( Point(topLeft.x,(topLeft.y + botRight.y) / 2), Point((topLeft.x + botRight.x) / 2, botRight.y), costBotLeft);
            botLeftCell->insert();
            unsigned int costTopRight = 0; // ToDo: Get real cost of the new cell
            topRightCell = new Quadtree_Cell( Point((topLeft.x + botRight.x) / 2, topLeft.y), Point(botRight.x, (topLeft.y + botRight.y) / 2), costTopRight);
            topRightCell->insert();
            unsigned int costBotRight = 0;
            botRightCell = new Quadtree_Cell( Point((topLeft.x + botRight.x) / 2, (topLeft.y + botRight.y) / 2), Point(botRight.x, botRight.y), costBotRight);
            botRightCell->insert();
    }
}

void Quadtree_Cell::testQuadtree(ros::Publisher marker_publisher_) {
    ROS_INFO("Position Top Left x: %i, Position Top Left y: %i Position Bottom Right x: %i Position Bottom Right y: %i Cost: %i", topLeft.x, topLeft.y, botRight.x, botRight.y, cost );
    publishVisualization(marker_publisher_,  ( double ((topLeft.x + botRight.x) /2.0)),  (( double( (topLeft.y + botRight.y)) / 2.0) ), double (botRight.x - topLeft.x), double(botRight.y -topLeft.y) );
    if (topLeftCell != nullptr) {
        ROS_INFO("Showing top left cell");
        topLeftCell->testQuadtree(marker_publisher_);
    }
    if (botLeftCell != nullptr) {
        ROS_INFO("Showing bottom left cell");
        botLeftCell->testQuadtree(marker_publisher_);
    }
    if (topRightCell != nullptr) {
        ROS_INFO("Showing top right cell");
        topRightCell->testQuadtree(marker_publisher_);
    }
    if (botRightCell != nullptr) {
        ROS_INFO("Showing bottom right cell");
        botRightCell->testQuadtree(marker_publisher_);
    }
}




// Visualization
void Quadtree_Cell::publishVisualization(ros::Publisher marker_pub, double marker_pose_x, double marker_pose_y, double marker_scale_x,
                                         double marker_scale_y) {
    //  ros::Rate r(1);
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
    static int idCounter = 0;

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = idCounter;
    idCounter++;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = marker_pose_x;
    marker.pose.position.y = marker_pose_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = marker_scale_x;
    marker.scale.y = marker_scale_y;
    marker.scale.z = 1+idCounter;


    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // Change colors
    int modulo_value = idCounter % 4;
    if ( modulo_value == 0)  {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    } else if ( modulo_value == 1)  {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    } else if ( modulo_value == 2)  {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
    } else if ( modulo_value == 3)  {
        marker.color.r = 0.33f;
        marker.color.g = 0.33f;
        marker.color.b = 0.33f;
    }



    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);

    //   r.sleep();
}