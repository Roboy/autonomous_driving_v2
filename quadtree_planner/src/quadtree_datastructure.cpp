//
// Created by maximilian on 25.06.19.
//

#include <ros/console.h>
#include "../include/quadtree_planner/quadtree_datastructure.h"


// Builds a quadtree (recursive implementation)
// Lowest level of quadtree contains either homogenous cells or cells with an area < 4
// Lowest level can be determined by checking if topLeftCell == nullptr
int Quadtree_Cell::buildQuadtree(quadtree_planner::Costmap* costmap, long long * area_)
{

    // We are at a cell of area < 4
    // We do not subdivide this cell further as we cannot define four child cells in this case
    // or we have an uniform area --> no subdivision of this cell required
    int cell_area = (botRight.x - topLeft.x + 1) * (botRight.y - topLeft.y + 1);
    bool isAreaUniform = isCostOfAreaUniform(topLeft, botRight, costmap);

    if ( (cell_area < 4) || (isAreaUniform == true) ) {
        *area_ += cell_area;    // Debugging
        ROS_INFO("Create low level cell");
        return -1;
    } else {

        // no uniform area in the current cell and cell size is > 4 --> subdivision required
        Point topL = Point(topLeft.x, topLeft.y);
        Point botR = Point((topLeft.x + botRight.x) / 2, (topLeft.y + botRight.y) / 2);
        unsigned int costTopLeft = getMaximumCostOfArea(topL, botR, costmap);
        topLeftCell = new Quadtree_Cell(topL, botR, costTopLeft);
        topLeftCell->buildQuadtree(costmap, area_);

        topL = Point(topLeft.x, (topLeft.y + botRight.y) / 2 + 1);
        botR = Point((topLeft.x + botRight.x) / 2, botRight.y);
        unsigned int costBotLeft = getMaximumCostOfArea(topL, botR, costmap);
        botLeftCell = new Quadtree_Cell(topL, botR, costBotLeft);
        botLeftCell->buildQuadtree(costmap, area_);

        topL = Point((topLeft.x + botRight.x) / 2 + 1, topLeft.y);
        botR = Point(botRight.x, (topLeft.y + botRight.y) / 2);
        unsigned int costTopRight = getMaximumCostOfArea(topL, botR, costmap);
        topRightCell = new Quadtree_Cell(topL, botR, costTopRight);
        topRightCell->buildQuadtree(costmap, area_);

        topL = Point((topLeft.x + botRight.x) / 2 + 1, (topLeft.y + botRight.y) / 2 + 1);
        botR = Point(botRight.x, botRight.y);
        unsigned int costBotRight = getMaximumCostOfArea(topL, botR, costmap);
        botRightCell = new Quadtree_Cell(topL, botR, costBotRight);
        botRightCell->buildQuadtree(costmap, area_);
    }

   return 0;
}

unsigned int Quadtree_Cell::getMaximumCostOfArea(Point topL, Point botR, quadtree_planner::Costmap* costmap) {
    unsigned int maxCost = 0;
    for(unsigned int x = topL.x; x <= botR.x; x++) {
        for (unsigned int y = topL.y; y <= botR.y; y++) {
            unsigned int currentCost = costmap->getCost(x,y);
            if (currentCost > maxCost) {
                maxCost = currentCost;
            }
        }
    }
    return maxCost;
}

bool Quadtree_Cell::isCostOfAreaUniform(Point topL, Point botR, quadtree_planner::Costmap *costmap) {
    bool freeArea = false;
    bool obstacleArea = false;

    for(unsigned int x = topL.x; x <= botR.x; x++) {
        for (unsigned int y = topL.y; y <= botR.y; y++) {
            if(costmap->getCost(x,y) == 0) {
                freeArea = true;
            } else {
                obstacleArea = true;
            }
        }
    }

    if(freeArea && obstacleArea) {  // area is not uniform as there are free cells and occupied cells
        return false;
    } else {
        return true;
    }
}

// This method is used only for testing purposes - not required for the actual path planning!
void Quadtree_Cell::testQuadtree(ros::Publisher marker_publisher_, double resolution, bool showOnlyLowestLevel) {
    // origin: [-30.133392, -47.747141, 0.0] values taken from map.pgm file according to map.yaml file (subfolder navigation/config)
    double x_origin = -30.133392;
    double y_origin = -47.747141;
  //  ROS_INFO("Position Top Left x: %i, Position Top Left y: %i Position Bottom Right x: %i Position Bottom Right y: %i Cost: %i", topLeft.x, topLeft.y, botRight.x, botRight.y, cost );
    if(showOnlyLowestLevel == false) {
        publishVisualization(marker_publisher_, (double((topLeft.x + botRight.x) * resolution / 2.0 + x_origin)),
                             ((double((topLeft.y + botRight.y) * resolution) / 2.0 + y_origin)),
                             double(botRight.x - topLeft.x + 1) * resolution, double(botRight.y - topLeft.y + 1) * resolution);
        ROS_INFO("Publish marker");
        ros::Duration(0.004).sleep();
    } else if (topLeftCell == nullptr) {    // We are on lowest level of quadtree
        publishVisualization(marker_publisher_, (double((topLeft.x + botRight.x) * resolution / 2.0 + x_origin)),
                             ((double((topLeft.y + botRight.y) * resolution) / 2.0 + y_origin)),
                             double(botRight.x - topLeft.x + 1) * resolution, double(botRight.y - topLeft.y + 1) * resolution);
        ROS_INFO("Publish marker");
        ros::Duration(0.004).sleep();
    }

    if (topLeftCell != nullptr) {
  //      ROS_INFO("Showing top left cell");
        topLeftCell->testQuadtree(marker_publisher_, resolution, showOnlyLowestLevel);
    }
    if (botLeftCell != nullptr) {
   //     ROS_INFO("Showing bottom left cell");
        botLeftCell->testQuadtree(marker_publisher_, resolution, showOnlyLowestLevel);
    }
    if (topRightCell != nullptr) {
   //     ROS_INFO("Showing top right cell");
        topRightCell->testQuadtree(marker_publisher_, resolution, showOnlyLowestLevel);
    }
    if (botRightCell != nullptr) {
    //    ROS_INFO("Showing bottom right cell");
        botRightCell->testQuadtree(marker_publisher_, resolution, showOnlyLowestLevel);
    }
}



// Visualization - not required for actual path planning!
void Quadtree_Cell::publishVisualization(ros::Publisher marker_pub, double marker_pose_x, double marker_pose_y, double marker_scale_x,
                                         double marker_scale_y) {
    //  ros::Rate r(1);
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
    static long int idCounter = 0;

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "quadtree_namespace";
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
    marker.scale.z = 0.1;


    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5f;

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

void Quadtree_Cell::printQuadtree() {
    ROS_INFO("topLeft.x = %i, topLeft.y = %i, bottomRight.x = %i, bottomRight.y = %i !", topLeft.x, topLeft.y, botRight.x, botRight.y);
    ROS_INFO("Cost: %i", cost);
}