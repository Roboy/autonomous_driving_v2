//
// Created by Maximilian Kempa on 11.06.19.
//

#ifndef QUADTREE_PLANNER_H
#define QUADTREE_PLANNER_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>
// Abstract global planner from move_base
#include <nav_core/base_global_planner.h>
#include <vector>
#include <unordered_map>

#include "costmap.h"
#include "utils.h"

// Visualization
#include <visualization_msgs/Marker.h>

#include "../include/quadtree_planner/quadtree_datastructure.h"

namespace quadtree_planner {

    class QuadTreePlanner : public nav_core::BaseGlobalPlanner {
    public:
        QuadTreePlanner();

        ~QuadTreePlanner();

        /**
          * Initialization function for the AStarPlanner object
          * @param  name The name of this planner
          * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
          */
        void initialize(std::string name,
                        costmap_2d::Costmap2DROS *costmap_ros) override;

        /**
         * Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
        */
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan) override;

    public:
        // Methods that are left public for unit tests

        /**
         * Initialization function for the AStarPlanner object
         */
        void initialize(std::string name, Costmap *costmap);

        bool makePlan(const Pose &start, const Pose &goal, std::vector<Pose> &path);

        /**
         * Get poses reachable from the given pose by integrating a discrete set of controls
         * over a short period of time.
         * @param pos Pose from which to start.
         * @return a vector of reachable poses.
         */
        std::vector<PoseWithDist> getNeighbors(const Pose &pos) const;

        std::vector<CellWithDist> getNeighborCells(const Cell &cell) const;

        std::vector<QuadtreeCellWithDist> getNeighborQuads(QuadtreeCellWithDist &quad, Pose goal) const;


    private:
        /**
         * Reconstruct the path to the goal position given a set of child/parent pairs.
         * A child position was visited from the parent position when running the planning algorithm.
         * @param parents a map of child/parent relationships. Keys are children, values are parents.
         * @param goal_pos goal position, is assumed to have no children.
         * @param path conainer where the path is be stored.
         *              The first element is the start position, the last element - goal position.
         */
        void getPath(const std::unordered_map<Pose, Pose> &parents,
                     const Pose &goal_pos,
                     std::vector<Pose> &path) const;

        void publishPlan(std::vector<geometry_msgs::PoseStamped> &path);

        /**
         * Estimated distance between two poses.
         */
        double distEstimate(const Pose &pose1, const Pose &pose2) const;

        /**
         * Snap pose to a costmap cell.
         */
        Cell getCell(const Pose &pos) const;


        // Quad Tree based search
        /** Snap pose to a quadtree cell
         *
         * @param pos
         * @param quadtreeCellObject
         * @return
         */
        Quadtree_SearchCell getQuad(const Pose &pos, std::vector<Quadtree_SearchCell> QuadtreeSearchCellVectorObject);

        Pose getPoseFromQuad(Quadtree_SearchCell &quad, Pose goal) const;

        bool hasReachedGoalQuad(Quadtree_SearchCell &quad, const Pose &goal);

        // Cell based search
        Pose getPoseFromCell(const Cell &cell) const;

        void loadParameters();

        bool validateParameters() const;

        bool hasReachedGoal(const Pose &pos, const Pose &goal);

        bool hasReachedGoalCell(const Cell &cell, const Pose &goal);

        bool checkBounds(const Pose &pos) const;

        bool checkBoundsCell(const Cell &cell) const;

        PoseWithDist goStraight(const Pose &pos) const;

        PoseWithDist turnLeft(const Pose &pos, double angle) const;

        PoseWithDist turnRight(const Pose &pos, double angle) const;

        void publishVisualization(ros::Publisher marker_pub, double marker_pose_x, double marker_pose_y, double marker_scale);

    private:
        std::string name_;
        Costmap *costmap_;
        std::string global_frame_;
        ros::Publisher plan_publisher_;
        int max_allowed_time_;

        double step_size_;
        double turning_radius_;
        double goal_tolerance_;

        // Visualization
        ros::Publisher marker_publisher_;

        // Quadtree planning
        Quadtree_Cell QuadtreeCellObject;
        std::vector<Quadtree_SearchCell> QuadtreeSearchCellVector;

        // Debugging
        long long area;
    };


}

#endif //QuadTreePlanner_H
