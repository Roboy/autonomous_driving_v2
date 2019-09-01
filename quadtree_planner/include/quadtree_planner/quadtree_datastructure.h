//
// Created by maximilian on 25.06.19.
//


#ifndef SRC_QUADTREE_DATASTRUCTURE_H
#define SRC_QUADTREE_DATASTRUCTURE_H

#include <ros/ros.h>

// Visualization
#include <visualization_msgs/Marker.h>
#include "costmap.h"

/**
 * structure for positions (points) that contains an x and y coordinate (unsigned int)
 */
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

/**
 * used as nodes in the A* search algorithm
 */
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

    /**
     * @brief  Returns the top left point of the quadtree search cell
     * @return top left point of the quadtree search cell
     */
    Point getTopLeft();

    /**
     * @brief  Returns the bottom right point of the quadtree search cell
     * @return bottom right point of the quadtree search cell
     */
    Point getBotRight();

    /**
     * @brief  returns the cost of the quadtree search cell
     * @return cost of the quadtree search cell
     */
    unsigned int getCost();

    /**
     * @brief  Returns a std::vector of pointers to the neighbors of this quadtree search cell
     * @return a std::vector of pointers to the neighbors of this quadtree search cell
     */
    std::vector<Quadtree_SearchCell*> getNeighbors();

    /**
     * @brief  Sets a std::vector of pointers to the neighbors of this quadtree search cell
     * @param _neighbors std::vector of pointers to the neighbors of this quadtree search cell
     */
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

    /**
    * @brief  returns the maximum cost of the costmap points in the rectangle defined by topL and botR
    * @param topL top left point of rectangle to be checked
    * @param botR bottom right point of rectangle to be checked
    * @param costmap pointer to costmap that is used to determine the costs
    * @return maximum cost of the costmap points in the rectangle defined by topL and botR
    */
    unsigned int getMaximumCostOfArea(Point topL, Point botR, quadtree_planner::Costmap* costmap);

    /**
    * @brief  returns true if the area between topL and botR is completely free or completely occupied
    * @param topL top left point of rectangle to be checked
    * @param botR bottom right point of rectangle to be checked
    * @param costmap pointer to costmap that is used to determine the costs
    * @return true if the area between topL and botR is completely free or completely occupied; false if it is partly free and partly occupied
    */
    bool isCostOfAreaUniform(Point topL, Point botR, quadtree_planner::Costmap* costmap);

    // Search Cell vector utilities
    /**
     * @brief  Converts a Quadtree_Cell object to a Quadtree_SearchCell object
     * @return Quadtree_SearchCell object based on the Quadtree_Cell
     */
    Quadtree_SearchCell convertToQuadtreeSearchCell();

    void publishVisualization(ros::Publisher marker_pub, double marker_pose_x, double marker_pose_y, double marker_scale_x,
                              double marker_scale_y, bool free_space);

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

    /**
     * @brief  This method builds a quadtree datastructure based on a 2D costmap as input. It subdivides cells as long
     * as they are not uniform yet (uniform means whole cell is free or whole cell is occupied).
     * @param costmap pointer to the costmap that is used to construct the quadtree
     * @param area_ debug variable that stores the area of the whole quadtree
     * @return 0 if is creation succesful, -1 otherwise
     */
    int buildQuadtree(quadtree_planner::Costmap* costmap, long long * area_);

    /**
     * @brief  This method visualizes the quadtree cells in rviz.
     * @param marker_publisher_ ros::publisher to publish the cells as markers to rviz
     * @param resolution resolution of the costmap in meters per cell
     * @param showOnlyLowestLevel  if true, show only the lowest level of the quadtree
     * @param origin_x origin in x direction of costmap in meters
     * @param origin_y_origin in y direction of costmap in meters
     */
    void testQuadtree(ros::Publisher marker_publisher_, double resolution, bool showOnlyLowestLevel, double origin_x,
                      double origin_y);

    /**
     * @brief  Creates a std::vector with Quadtree_SearchCell objects based on the Quadtree_Cell
     * @param quadVector std::vector to store the Quadtree_SearchCell objects
     */
    void createSearchCellVector(std::vector<Quadtree_SearchCell>* quadVector);

    /**
     * @brief  Finds the neighbors of all Quadtree_SearchCells contained in the input std::vector
     * @param quadVector neighbors of Quadtree_SearchCells in this vector are found and set by this method
     */
    void findNeighborsInSearchCellVector(std::vector<Quadtree_SearchCell> & quadVector);

    bool operator==(const Quadtree_Cell &p) const {
        return topLeft == p.topLeft && botRight == p.botRight && cost == p.cost && topLeftCell == p.topLeftCell && topRightCell == p.topRightCell && botLeftCell == p.botLeftCell && botRightCell == p.botRightCell;
    }

    /**
     * @brief  Prints topL, botR and cost of the Quadtree_Cell object
     */
    void printQuadtree();
};



#endif //SRC_QUADTREE_DATASTRUCTURE_H
