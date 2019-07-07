//
// Created by Maximilian Kempa on 11.06.19.
//

#include <pluginlib/class_list_macros.h>

#include <chrono>
#include <ctime>
#include <math.h>
#include <queue>
#include <unordered_map>

#include <ros/console.h>
#include <nav_msgs/Path.h>
#include "../include/quadtree_planner/quadtree_planner.h"
#include "../include/quadtree_planner/utils.h"
#include "../include/quadtree_planner/costmap.h"
#include "../include/quadtree_planner/quadtree_datastructure.h"


PLUGINLIB_EXPORT_CLASS(quadtree_planner::QuadTreePlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace quadtree_planner {

    QuadTreePlanner::QuadTreePlanner() :
        name_(""), costmap_(nullptr), step_size_(0.0), turning_radius_(0.0), global_frame_(""), area(0) {}

    void QuadTreePlanner::initialize(std::string name,
                                  costmap_2d::Costmap2DROS *costmap_ros) {
        global_frame_ = costmap_ros->getGlobalFrameID();
        initialize(name, new CostmapAdapter(costmap_ros->getCostmap()));
    };

    void QuadTreePlanner::initialize(std::string name, quadtree_planner::Costmap *costmap) {
        name_ = name;
        costmap_ = costmap;
        ros::NodeHandle n;
        plan_publisher_ = n.advertise<nav_msgs::Path>(name + "/global_plan", 1);
        marker_publisher_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        loadParameters();
        double dth = step_size_ / turning_radius_;
        ROS_INFO("QuadTreePlanner initialized with name '%s' ",
                 name_.c_str());

        // Testing of quadtree data structure
        Point botR = Point (costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
        Quadtree_Cell testQuadTreeObject (Point(0,0), botR, 255);
        testQuadTreeObject.printQuadtree();
        ROS_INFO("testQuadtreeObejct created");
        testQuadTreeObject.buildQuadtree(costmap, &area);
        ROS_INFO("Quadtree built successfully");
        ROS_INFO("Total area of quadtree is %lli", area);
        testQuadTreeObject.testQuadtree(marker_publisher_, costmap->getResolution(), true);
        ROS_INFO("Quadtree test was run");
    }

    void QuadTreePlanner::loadParameters() {
        ros::NodeHandle nh("~" + name_);
        nh.param<double>(std::string("turning_radius"), turning_radius_, 0.0);
        nh.param<double>("step_size", step_size_, 0.0);
        nh.param<int>("max_allowed_time", max_allowed_time_, 20);
        nh.param<double>("goal_tolerance", goal_tolerance_, 0.5);
    }

    bool QuadTreePlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan) {
        std::vector<Pose> positions;
        bool foundPlan;
        try {
            foundPlan = makePlan(Pose(start), Pose(goal), positions);
        } catch (exception &ex) {
            ROS_FATAL("QuadTreePlanner exception occured %s", ex.what());
            throw ex;
        }
        if (!foundPlan) {
            return false;
        }
        ros::Time plan_time = ros::Time::now();
        for (auto position : positions) {
            auto pose = position.toPoseStamped();
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame_;
            plan.push_back(pose);
        }
        publishPlan(plan);
        return true;
    }

    void QuadTreePlanner::publishPlan(std::vector<geometry_msgs::PoseStamped> &path) {
        nav_msgs::Path gui_path;
        gui_path.header.frame_id = global_frame_;
        gui_path.header.stamp = ros::Time::now();
        gui_path.poses.resize(path.size());
        std::copy(path.begin(), path.end(), gui_path.poses.begin());
        plan_publisher_.publish(gui_path);
    }


    bool QuadTreePlanner::makePlan(const Pose &start, const Pose &goal, vector<Pose> &path) {
        if (!validateParameters()) {
            return false;
        }
        auto cell_start = getCell(start);
        auto cell_goal = getCell(goal);
        // Instantiate data structures
  //      set<PoseWithDist> candidatePoses = {PoseWithDist(0.0, start)};  // set is sorted based on distance, pose with smallest distance is the first element
  //      unordered_map<Cell, double> pathLength = {{cell_start, 0.0}};
        //TODO(melkonyan): this is potentially very unoptimal, because poses will be copied many times.
  //      unordered_map<Pose, Pose> parents;
  //      Pose reached_pose;
  //      bool reached_goal = false;


        // Cell based search
        set<CellWithDist> candidateCells = {CellWithDist(0.0, getCell(start))};
        unordered_map<Cell, Cell> parentsCells;
        unordered_map<Pose, Pose> parentsCellsPoses;
        unordered_map<Cell, double> pathLengthCells = {{cell_start, 0.0}};
        Cell reached_cell;
        bool reached_goal_cell = false;

        // Create variables to log performance
        time_t start_time = time(NULL);
        int num_nodes_visited = 0;

        // Implementation from last semester:
        // Perform the search as long as there are still candidatePoses available
    /*    while (!candidatePoses.empty()) {
            // Checks if the maximum allowed time of the algorithm is already reached
            if (max_allowed_time_ > 0 && difftime(time(NULL), start_time) > max_allowed_time_) {
                break;
            }
            num_nodes_visited++;
            PoseWithDist cand = *candidatePoses.begin();
            auto cell_cand = getCell(cand.pose);
            if (hasReachedGoal(cand.pose, goal)) {
                reached_pose = cand.pose;
                reached_goal = true;
                break;
            }
            double l_cand = pathLength[cell_cand];

            candidatePoses.erase(cand);
            vector<PoseWithDist> neighbors = getNeighbors(cand.pose);
            for (auto &nbr : neighbors) {
                // Checks if neigboring pose is outside of the map
                if (!checkBounds(nbr.pose)) {
                    continue;
                }
                auto cell_nbr = getCell(nbr.pose);
                if (cell_cand == cell_nbr) {
                    ROS_WARN("AStarPlanner: Oops, ended up in the same cell.");
                    continue;
                }
                // Checks if the cell of the neighboring pose contains an obstacle (cost > 0)
                if (costmap_->getCost(cell_nbr.x, cell_nbr.y) > 0) {
                    continue;
                }
                // Check if cell_nbr is not contained in the pathLength set yet (meaning that it is not explored yet)
                // or if the distance (l_cand + nbr.dist) is smaller than the distance that is currently stored for
                // cell_nbr
                // if both conditions are fulfilled, the neighbor needs to be inserted in the candidatePoses set
                if (pathLength.find(cell_nbr) == pathLength.end() || l_cand + nbr.dist < pathLength[cell_nbr]) {
                    pathLength[cell_nbr] = l_cand + nbr.dist;
                    parents[nbr.pose] = cand.pose;
                    candidatePoses.insert(PoseWithDist(l_cand + nbr.dist + distEstimate(nbr.pose, goal), nbr.pose));
                }
            }

        }   */


        // Cell based search
        // Perform the search as long as there are still candidateCells available
       ROS_INFO("Starting cell based search.");
       while (!candidateCells.empty()) {
            // Checks if the maximum allowed time of the algorithm is already reached
            if (max_allowed_time_ > 0 && difftime(time(NULL), start_time) > max_allowed_time_) {
                break;
            }
            num_nodes_visited++;
            CellWithDist candCell = *candidateCells.begin();
            Pose candCellPose = getPoseFromCell(candCell.cell);
            if (hasReachedGoalCell(candCell.cell, goal)) {
                reached_cell = candCell.cell;
                reached_goal_cell = true;
                break;
            }
            else {
                double distToGoal = distEstimate(getPoseFromCell(candCell.cell), goal);
             //   ROS_INFO("The distance of the current candCell to the goal is %.2f", distToGoal);
            }
            double l_candCell = pathLengthCells[candCell.cell];

            // Visualization
            publishVisualization(marker_publisher_, candCellPose.x, candCellPose.y, costmap_->getResolution());

            candidateCells.erase(candCell);
            vector<CellWithDist> neighbors = getNeighborCells(candCell.cell);
            for (auto &nbr : neighbors) {
                if (candCell.cell == nbr.cell) {
                    ROS_WARN("AStarPlanner: Oops, ended up in the same cell.");
                    continue;
                }
                // Checks if the neighboring cell contains an obstacle (cost > 0)
                if (costmap_->getCost(nbr.cell.x, nbr.cell.y) > 0) {
                    continue;
                }
                // Check if cell_nbr is not contained in the pathLength set yet (meaning that it is not explored yet)
                // or if the distance (l_cand + nbr.dist) is smaller than the distance that is currently stored for
                // cell_nbr
                // if both conditions are fulfilled, the neighbor needs to be inserted in the candidatePoses set
                if (pathLengthCells.find(nbr.cell) == pathLengthCells.end() || l_candCell + nbr.dist < pathLengthCells[nbr.cell]) {
                    Pose nbr_cell_pose = getPoseFromCell(nbr.cell);
         //           ROS_INFO("Debug Info: New Cell with cell coordinates %i x %i y and cartesian coordinates %f x %f y inserted in candidateCells set", nbr.cell.x, nbr.cell.y, nbr_cell_pose.x, nbr_cell_pose.y);
                    pathLengthCells[nbr.cell] = l_candCell + nbr.dist;
                    parentsCells[nbr.cell] = candCell.cell;
                    parentsCellsPoses[nbr_cell_pose] = candCellPose;
                    candidateCells.insert(CellWithDist(l_candCell + nbr.dist + distEstimate(getPoseFromCell(nbr.cell), goal) ,nbr.cell));
                }
            }

        }


  /*      if (reached_goal) {
            getPath(parents, reached_pose, path);
        }   */



        // Cell based search
       if (reached_goal_cell) {
            ROS_INFO("Cell based search reached the goal cell");
            Pose goal_cell_pose = getPoseFromCell(reached_cell);
            ROS_INFO("The found goal cell has the cell coordinates %i x %i y and cartesian coordinates %f x %f y %f th", reached_cell.x, reached_cell.y, goal_cell_pose.x, goal_cell_pose.y, goal_cell_pose.th);
            double distanceGoalCellRealGoal = distEstimate(goal_cell_pose, goal);
            ROS_INFO("The distance of the goal cell to the real goal is %f m", distanceGoalCellRealGoal);
            getPath(parentsCellsPoses, goal_cell_pose, path);
        } else {
           ROS_INFO("Cell based search did not reach the goal cell");
       }

//        ROS_INFO("AStarPlanner finished in %.2fs, generated %d nodes, reached goal: %s",
//                 difftime(time(NULL), start_time), num_nodes_visited, reached_goal ? "true" : "false");
        ROS_INFO("QuadTreePlanner finished in %.2fs, generated %d nodes, reached goal: %s",
                 difftime(time(NULL), start_time), num_nodes_visited, reached_goal_cell ? "true" : "false");

        return reached_goal_cell;
  //      return reached_goal;

    }

    bool QuadTreePlanner::validateParameters() const {
        if (turning_radius_ <= 0) {
            ROS_ERROR("AStarPlanner: turning radius has invalid value=%.2f. Must be greater than zero.",
                      turning_radius_);
            return false;
        }
        if (step_size_ <= 0) {
            ROS_ERROR("AStarPlanner: step size has invalid value=%.2f. Must be greater than zero.",
                      step_size_);
            return false;
        }
        if (goal_tolerance_ <= 0) {
            ROS_ERROR("AStarPlanner: goal tolerance has invalid value=%.2f. Must be greater than zero",
                      goal_tolerance_);
            return false;
        }
        if (goal_tolerance_ < step_size_) {
            ROS_WARN("AStarPlanner: goal tolerance (=%.2f) is smaller than the step size (=%.2f). "
                     "Planner might fail to find a path.", goal_tolerance_, step_size_);
        }
        if (step_size_ < costmap_->getResolution()) {
            ROS_WARN("AStarPlannner: step size (=%.2f) is smaller than costmap resolution (=%.2f). "
                     "Planner might fail to exlore the map properly.", step_size_, costmap_->getResolution());
        }
        return true;
    }

    void QuadTreePlanner::getPath(const unordered_map<Pose, Pose> &parents,
                               const Pose &goal_pose,
                               vector<Pose> &path) const {
        auto curr_pose = goal_pose;
        while (true) {
            path.push_back(curr_pose);
            auto search = parents.find(curr_pose);
            if (search == parents.end()) {
                break;
            }
            curr_pose = search->second;
        }
        reverse(path.begin(), path.end());
    }

    bool QuadTreePlanner::hasReachedGoal(const Pose &pos, const Pose &goal) {
        return euclid_dist(pos, goal) <= goal_tolerance_;
    }

    // Cell based search
    bool QuadTreePlanner::hasReachedGoalCell(const Cell &cell, const Pose &goal)
    {
        Pose cellPose = getPoseFromCell(cell);
        return distEstimate(cellPose, goal) <= goal_tolerance_;
    }

    PoseWithDist QuadTreePlanner::turnLeft(const Pose &pos, double dth) const {
        double pos_dx = -sin(pos.th) * turning_radius_;
        double pos_dy = cos(pos.th) * turning_radius_;
        double c_x = pos.x - pos_dx;
        double c_y = pos.y - pos_dy;
        double new_dx = -sin(pos.th - dth) * turning_radius_;
        double new_dy = cos(pos.th - dth) * turning_radius_;
        auto new_pos = Pose(c_x + new_dx, c_y + new_dy, pos.th - dth);
        return PoseWithDist(step_size_, new_pos);
    }

    PoseWithDist QuadTreePlanner::goStraight(const Pose &pos) const {
        auto go_straight = Pose();
        go_straight.x = pos.x + step_size_ * cos(pos.th);
        go_straight.y = pos.y + step_size_ * sin(pos.th);
        go_straight.th = pos.th;
        return PoseWithDist(step_size_, go_straight);
    }

    PoseWithDist QuadTreePlanner::turnRight(const Pose &pos, double dth) const {
        double pos_dx = sin(pos.th) * turning_radius_;
        double pos_dy = -cos(pos.th) * turning_radius_;
        double c_x = pos.x - pos_dx;
        double c_y = pos.y - pos_dy;
        double new_dx = sin(pos.th + dth) * turning_radius_;
        double new_dy = -cos(pos.th + dth) * turning_radius_;
        auto new_pos = Pose(c_x + new_dx, c_y + new_dy, pos.th + dth);
        return PoseWithDist(step_size_, new_pos);
    }

    bool QuadTreePlanner::checkBounds(const Pose &pos) const {
        uint x, y;
        return costmap_->worldToMap(pos.x, pos.y, x, y);
    }


    vector<PoseWithDist> QuadTreePlanner::getNeighbors(const Pose &pos) const {
        double dth = step_size_ / turning_radius_;
        return {turnLeft(pos, dth), goStraight(pos), turnRight(pos, dth)};
    }


    vector<CellWithDist> QuadTreePlanner::getNeighborCells(const Cell &cell) const{
        CellWithDist upperCellWithDist = CellWithDist(step_size_, Cell());
        CellWithDist rightCellWithDist = CellWithDist(step_size_, Cell());
        CellWithDist lowerCellWithDist = CellWithDist(step_size_, Cell());
        CellWithDist leftCellWithDist =  CellWithDist(step_size_, Cell());

        Cell upperCell = Cell();
        Cell lowerCell = Cell();
        Cell rightCell = Cell();
        Cell leftCell = Cell();

        if(cell.x == costmap_->getSizeInCellsX()) {
            upperCell = cell;
            upperCellWithDist = CellWithDist(0.0, cell);
            ROS_INFO("Debug Info: Reached Upper Limit of Map");
        }
        else {
            upperCell = Cell(cell.x+1, cell.y, cell.th);
            upperCellWithDist = CellWithDist(distEstimate(getPoseFromCell(cell),getPoseFromCell(upperCell)), upperCell);
        }

        if(cell.y == costmap_->getSizeInCellsY()) {
            rightCell = cell;
            rightCellWithDist = CellWithDist(0.0, cell);
            ROS_INFO("Debug Info: Reached right Limit of Map");
        } else {
            rightCell = Cell(cell.x, cell.y+1, cell.th);
            rightCellWithDist = CellWithDist(distEstimate(getPoseFromCell(cell),getPoseFromCell(rightCell)), rightCell);
        }

        if(cell.x > 0) {
            lowerCell = Cell(cell.x-1, cell.y, cell.th);
            lowerCellWithDist = CellWithDist(distEstimate(getPoseFromCell(cell),getPoseFromCell(lowerCell)), lowerCell);
        } else {
            lowerCell = cell;
            lowerCellWithDist = CellWithDist(0.0, cell);
            ROS_INFO("Debug Info: Reached lower Limit of Map");
        }

        if(cell.y > 0) {
            leftCell = Cell(cell.x, cell.y-1, cell.th);
            leftCellWithDist = CellWithDist(distEstimate(getPoseFromCell(cell),getPoseFromCell(leftCell)), leftCell);
        } else {
            leftCell = cell;
            leftCellWithDist = CellWithDist(0.0, cell);
            ROS_INFO("Debug Info: Reached left Limit of Map");
        }
        return{upperCellWithDist, rightCellWithDist, lowerCellWithDist, leftCellWithDist};
    }

    Cell QuadTreePlanner::getCell(const Pose &pos) const {
        Cell cell;
        costmap_->worldToMap(pos.x, pos.y, cell.x, cell.y);
        cell.th = 0;
        return cell;
    }

    Pose QuadTreePlanner::getPoseFromCell(const Cell &cell) const {
        Pose pos;
        costmap_->mapToWorld(cell.x, cell.y, pos.x, pos.y);
        pos.th = 0; // ToDo Maximilian Kempa: Think of a way to calculate the proper theta value (orientation of robot)
        return pos;
    }

    double QuadTreePlanner::distEstimate(const Pose &pose1, const Pose &pose2) const {
        return euclid_dist(pose1, pose2);
    }

    QuadTreePlanner::~QuadTreePlanner(){
        delete costmap_;
    }

    // Visualization
    void QuadTreePlanner::publishVisualization(ros::Publisher marker_pub, double marker_pose_x, double marker_pose_y, double marker_scale) {
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
      marker.scale.x = marker_scale;
      marker.scale.y = marker_scale;
      marker.scale.z = marker_scale;


      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

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

}



