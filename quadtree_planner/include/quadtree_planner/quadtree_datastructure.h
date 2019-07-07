//
// Created by maximilian on 25.06.19.
//


#ifndef SRC_QUADTREE_DATASTRUCTURE_H
#define SRC_QUADTREE_DATASTRUCTURE_H

#include <ros/ros.h>

// Visualization
#include <visualization_msgs/Marker.h>
#include "costmap.h"

// structure for positions
struct Point
{
    unsigned int x;
    unsigned int y;
    Point(unsigned int _x, unsigned int _y)
    {
        x = _x;
        y = _y;
    }
    Point()
    {
        x = 0;
        y = 0;
    }
};


// The main quadtree class
class Quadtree_Cell
{
private:
    // details of the boundary of this node
    Point topLeft;
    Point botRight;

    // costmap costs
    unsigned int cost;

    // Children of this cell
    Quadtree_Cell *topLeftCell;
    Quadtree_Cell *topRightCell;
    Quadtree_Cell *botLeftCell;
    Quadtree_Cell *botRightCell;

public:
    Quadtree_Cell()
    {
        topLeft = Point(0, 0);
        botRight = Point(0, 0);
        cost = 0;
        topLeftCell = nullptr;
        topRightCell = nullptr;
        botLeftCell = nullptr;
        botRightCell = nullptr;
    }
    Quadtree_Cell(Point _topL, Point _botR, unsigned int _cost)
    {
        cost = _cost;
        topLeftCell = nullptr;
        topRightCell = nullptr;
        botLeftCell = nullptr;
        botRightCell = nullptr;
        topLeft = _topL;
        botRight = _botR;
    }
    int buildQuadtree(quadtree_planner::Costmap* costmap, long long * area_);
    void testQuadtree(ros::Publisher marker_publisher_, double resolution, bool showOnlyLowestLevel);
    unsigned int getMaximumCostOfArea(Point topL, Point botR, quadtree_planner::Costmap* costmap);
    bool isCostOfAreaUniform(Point topL, Point botR, quadtree_planner::Costmap* costmap);   // Checks if a cell is completely free or completely occupied by an obstacle

    // Visualization
    void publishVisualization(ros::Publisher marker_pub, double marker_pose_x, double marker_pose_y, double marker_scale_x,
                              double marker_scale_y);

    // Debugging
    void printQuadtree();
 //   Quadtree_Cell* search(Point);
 //   bool inBoundary(Point);
};



#endif //SRC_QUADTREE_DATASTRUCTURE_H
