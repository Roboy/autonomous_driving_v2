#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include  "../include/quadtree_planner/utils.h"

namespace quadtree_planner {

    Pose::Pose(): Pose(0.0, 0.0, 0.0) {}

    Pose::Pose(double x, double y, double th) : x(x), y(y), th(th) {}

    Pose::Pose(const geometry_msgs::PoseStamped &pose) {
        tf2::Quaternion quat;
        tf2::fromMsg(pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        x = pose.pose.position.x;
        y = pose.pose.position.y;
        th = yaw;
    }

    geometry_msgs::PoseStamped Pose::toPoseStamped() {
        tf2::Quaternion quat;
        quat.setRPY(0, 0, th);
        auto pose = geometry_msgs::PoseStamped();
        pose.pose.orientation = tf2::toMsg(quat);
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        return pose;
    }

    geometry_msgs::Pose Pose::toPose(){
        tf2::Quaternion quat;
        quat.setRPY(0, 0, th);
        auto pose = geometry_msgs::Pose();
        pose.orientation = tf2::toMsg(quat);
        pose.position.x = x;
        pose.position.y = y;
        return pose;
    }

    IntermediatePathAngles::IntermediatePathAngles(): IntermediatePathAngles(0.0,0.0,0.0, LSL) {}

    IntermediatePathAngles::IntermediatePathAngles(double first_theta, double second_theta, double pathLength, DubinsPathType dubinsPathType):
    first_theta(first_theta), second_theta(second_theta), pathLength(pathLength), dubinsPathType(dubinsPathType)  {}

    bool IntermediatePathAngles::operator==(const quadtree_planner::IntermediatePathAngles &other) const {
        return first_theta == other.first_theta && second_theta == other.second_theta && pathLength == other.pathLength;
    }

    IntermediatePaths::IntermediatePaths(){
        first_index = 0;
        second_index = 0;
        std::vector<IntermediatePathAngles> intermediatePathAngles_;
        intermediatePathAngles = intermediatePathAngles_;
    }

    IntermediatePaths::IntermediatePaths(int first_index, int second_index,
                                         std::vector<IntermediatePathAngles> intermediatePathAngles): first_index(first_index), second_index(second_index), intermediatePathAngles(intermediatePathAngles) {}

    bool IntermediatePaths::operator==(const quadtree_planner::IntermediatePaths &other) const {
        return first_index == other.first_index && second_index == other.second_index;

    }

    DubinsSubpath::DubinsSubpath(): DubinsSubpath(Pose(), Pose(), 0, LSL) {}

    DubinsSubpath::DubinsSubpath(Pose q0, Pose q1, double turning_radius, DubinsPathType dubinsPathType):
    q0(q0), q1(q1), turning_radius(turning_radius), dubinsPathType(dubinsPathType) {}

    std::ostream& operator<<(std::ostream &out, const IntermediatePaths &intermediatePaths) {
        out << std::setprecision(3) << "IntermediatedPaths(" <<  ", " << intermediatePaths.first_index << ", " << intermediatePaths.second_index;
    }

    bool Pose::operator==(const quadtree_planner::Pose &other) const {
        return x == other.x && y == other.y && th == other.th;

    }

    std::ostream& operator<<(std::ostream &out, const Pose &pos) {
        out << std::setprecision(3) << "Pos(" << pos.x << ", " << pos.y << ", " << pos.th / M_PI << "pi)";
    }

    bool PoseWithDist::operator==(const PoseWithDist &other) const {
        return std::equal_to<Pose>()(pose, other.pose);
    }

    bool PoseWithDist::operator<(const PoseWithDist &other) const {
        return dist < other.dist;
    }

    bool QuadtreeCellWithDist::operator==(const QuadtreeCellWithDist &other) const {
        return std::equal_to<Quadtree_SearchCell>()(quadtreeCell, other.quadtreeCell);
    }

    bool QuadtreeCellWithDist::operator<(const QuadtreeCellWithDist &other) const {
        return dist < other.dist;
    }

    Coordinates::Coordinates(): Coordinates(0, 0){}

    Coordinates::Coordinates(int x, int y) :  x(x), y(y) {}

    double euclid_dist(const Pose &pose1, const Pose &pose2) {
        return sqrt(pow(pose1.x - pose2.x, 2)+ pow(pose1.y - pose2.y, 2));
    }

    /**
     * Normalize angle to [0, 2pi]
     */
    double normalize_angle(double angle) {
        return angle >= 0 ? fmod(angle, 2*M_PI) : 2 * M_PI - fmod(fabs(angle), 2*M_PI);
    }
}
