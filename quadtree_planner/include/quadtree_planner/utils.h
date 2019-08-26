//
// Created by alex on 15.02.19.
//

#ifndef QUADTREE_PLANNER_UTILS_H
#define QUADTREE_PLANNER_UTILS_H

#include <geometry_msgs/PoseStamped.h>
#include <ostream>
#include "../include/quadtree_planner/quadtree_datastructure.h"
#include "../include/quadtree_planner/dubins.h"


namespace quadtree_planner {

    /**
     * Container for storing pose of a 2D object in space.
     */
    struct Pose {
        double x;
        double y;
        double th;

        Pose();

        Pose(double x, double y, double th);

        Pose(const geometry_msgs::PoseStamped &pose);

        geometry_msgs::PoseStamped toPoseStamped();

        geometry_msgs::Pose toPose();

        bool operator==(const Pose &other) const;

        friend std::ostream& operator<<(std::ostream &out, const Pose &pos);

    };

    /**
 * Container for storing valid angle combinations
 */
    struct IntermediatePathAngles {
        double first_theta;
        double second_theta;
        double pathLength;
        DubinsPathType dubinsPathType;

        IntermediatePathAngles();

        IntermediatePathAngles(double first_theta, double second_theta, double pathLength, DubinsPathType dubinsPathType);

        bool operator==(const IntermediatePathAngles &other) const;

        friend std::ostream& operator<<(std::ostream &out, const IntermediatePathAngles &intermediatePathAngles);
    };

    /**
     * Container for storing DubinsSubpath
     */
    struct DubinsSubpath {
        Pose q0;
        Pose q1;
        double turning_radius;
        DubinsPathType dubinsPathType;

        DubinsSubpath();

        DubinsSubpath(Pose q0, Pose q1, double turning_radius, DubinsPathType dubinsPathType);
    };

    /**
    * Container for storing intermediate paths (possible angle combinations for each index combination)
    */
    struct IntermediatePaths {
        int first_index;
        int second_index;
        std::vector<IntermediatePathAngles> intermediatePathAngles;

        IntermediatePaths();

        IntermediatePaths(int first_index, int second_index, std::vector<IntermediatePathAngles> intermediatePathAngles);

        bool operator==(const IntermediatePaths &other) const;

        friend std::ostream& operator<<(std::ostream &out, const IntermediatePaths &intermediatePaths);
    };

    /**
     * Container for storing object position and associated cost.
     */
    struct PoseWithDist {
        double dist;
        Pose pose;

        PoseWithDist(double dist, const Pose &pose) : dist(dist), pose(pose) {}

        bool operator<(const PoseWithDist &other) const;

        bool operator==(const PoseWithDist &other) const;
    };

    /**
     * Container for storing quadtree cells and associated cost
     */
     struct QuadtreeCellWithDist {
         double dist;
         Quadtree_SearchCell quadtreeCell;

         QuadtreeCellWithDist(double dist, Quadtree_SearchCell &quad) : dist(dist), quadtreeCell(quad) {}

         bool operator<(const QuadtreeCellWithDist &other) const;

         bool operator==(const QuadtreeCellWithDist &other) const;
     };


    /**
     * Compute Euclidean distance between two positions.
     */
    double euclid_dist(const Pose &pose1, const Pose &pose2);

    double normalize_angle(double angle);
}

namespace std {

    template<>
    struct hash<quadtree_planner::Pose> {
        std::size_t operator()(const quadtree_planner::Pose &pose) const {
            return (size_t(pose.x) << 40) +
                   (size_t(pose.y) << 20) + size_t(pose.th);
        }
    };

    template <>
    struct hash<Quadtree_SearchCell> {
        std::size_t operator()(const Quadtree_SearchCell &quad) const {
            return (size_t(quad.topLeft.x) << 40) +
                   (size_t(quad.topLeft.y) << 20) + (size_t(quad.botRight.x) << 10) + size_t(quad.botRight.y);
        }
    };

}



#endif //ASTAR_PLANNER_UTILS_H
