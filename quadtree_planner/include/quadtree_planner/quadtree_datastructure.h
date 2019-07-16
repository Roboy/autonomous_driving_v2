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

    bool operator==(const Point &p) const {
        return x == p.x && y == p.y;
    }
};


class Quadtree_SearchCell
{
private:



public:
    // details of the boundary of this node
    Point topLeft;
    Point botRight;

    // costmap costs
    unsigned int cost;

    // Neighbors of this cell
    std::vector<Quadtree_SearchCell*>  neighbors;

    Quadtree_SearchCell()
    {
        topLeft = Point(0, 0);
        botRight = Point(0, 0);
        cost = 0;
        neighbors.push_back(nullptr);
    }
    Quadtree_SearchCell(Point _topL, Point _botR, unsigned int _cost, std::vector<Quadtree_SearchCell*>  _neighbors)
    {
        cost = _cost;
        topLeft = _topL;
        botRight = _botR;
        neighbors = _neighbors;
    }

    bool operator==(const Quadtree_SearchCell &other) const;

    Point getTopLeft();
    Point getBotRight();
    unsigned int getCost();
    std::vector<Quadtree_SearchCell*> getNeighbors();
    void setNeighbors(std::vector<Quadtree_SearchCell*> _neighbors);
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

    // Search Cell vector utilities
    Quadtree_SearchCell convertToQuadtreeSearchCell();
    void createSearchCellVector(std::vector<Quadtree_SearchCell>* quadVector);
    void findNeighborsInSearchCellVector(std::vector<Quadtree_SearchCell> & quadVector);

    // Visualization
    void publishVisualization(ros::Publisher marker_pub, double marker_pose_x, double marker_pose_y, double marker_scale_x,
                              double marker_scale_y, bool free_space);

    bool operator==(const Quadtree_Cell &p) const {
        return topLeft == p.topLeft && botRight == p.botRight && cost == p.cost && topLeftCell == p.topLeftCell && topRightCell == p.topRightCell && botLeftCell == p.botLeftCell && botRightCell == p.botRightCell;
    }

    // Debugging
    void printQuadtree();
 //   Quadtree_Cell* search(Point);
 //   bool inBoundary(Point);
};



#endif //SRC_QUADTREE_DATASTRUCTURE_H
