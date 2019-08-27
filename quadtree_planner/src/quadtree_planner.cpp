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
#include "../include/quadtree_planner/dubins.h"
#include "dubins.c"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

PLUGINLIB_EXPORT_CLASS(quadtree_planner::QuadTreePlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace quadtree_planner {

    QuadTreePlanner::QuadTreePlanner() :
        name_(""), costmap_(nullptr), turning_radius_(0.0), rickshaw_speed_(0.0), enable_pathRefinement_(true), global_frame_(""), area(0), QuadtreeCellObject(), QuadtreeSearchCellVector() {}

    void QuadTreePlanner::initialize(std::string name,
                                  costmap_2d::Costmap2DROS *costmap_ros) {
        global_frame_ = costmap_ros->getGlobalFrameID();
        initialize(name, new CostmapAdapter(costmap_ros->getCostmap()));
    }

    void QuadTreePlanner::initialize(std::string name, quadtree_planner::Costmap *costmap) {
        name_ = name;
        costmap_old = costmap;
        // inflated costmap initialisieren
        costmap_2d::Costmap2D costmap_ = costmap_2d::Costmap2D(costmap_old->getSizeInCellsX(), costmap_old->getSizeInCellsY(), costmap_old->getResolution(), 0, 0);
        inflateCostmap(0.2);
        ros::NodeHandle n;
        plan_publisher_ = n.advertise<nav_msgs::Path>(name + "/global_plan", 1);
        holonomic_plan_publisher_ = n.advertise<nav_msgs::Path>(name + "/holonomic_plan", 1);
        HolonomicPathPoses_publisher_ = n.advertise<geometry_msgs::PoseArray>(name + "/HolonomicPlanPoses", 1);
        nonHolonomicPathPoses_publisher_ = n.advertise<geometry_msgs::PoseArray>(name + "/nonHolonomicPlanPoses", 1);
        marker_publisher_ = n.advertise<visualization_msgs::Marker>(name + "/visualization_marker", 1);
        eta_publisher_ = n.advertise<std_msgs::Int16>(name + "/eta", 100, true);
        error_message_publisher_ = n.advertise<std_msgs::String>(name + "/error_message", 100, true);
        path_found_publisher_ = n.advertise<std_msgs::Bool>(name + "/path_found", 100, true);
        loadParameters();
        ROS_INFO("QuadTreePlanner initialized with name '%s' ",
                 name_.c_str());

        // Creation and testing of quadtree data structure
        Point botR = Point(costmap->getSizeInCellsX()-1, costmap->getSizeInCellsY()-1);
        QuadtreeCellObject = Quadtree_Cell(Point(0,0), botR, 255);
        QuadtreeCellObject.printQuadtree();
        ROS_INFO("testQuadtreeObject created");
        QuadtreeCellObject.buildQuadtree(costmap, &area);
        ROS_INFO("Quadtree built successfully");
        ROS_INFO("Total area of quadtree is %lli", area);
   //     ROS_INFO("Starting visualization of quadtree!");
   //     QuadtreeCellObject.testQuadtree(marker_publisher_, costmap->getResolution(), true);
   //     ROS_INFO("Quadtree test was run. Visualization finished.");
        QuadtreeCellObject.createSearchCellVector(&QuadtreeSearchCellVector);
        ROS_INFO("Creation of QuadtreeSearchCellVector was succesful");
        QuadtreeCellObject.findNeighborsInSearchCellVector(QuadtreeSearchCellVector);
        Quadtree_SearchCell* test = QuadtreeSearchCellVector.front().getNeighbors().front();
        ROS_INFO("Neighbor Search of QuadtreeSearchCellVector was succesful");
    }

    void QuadTreePlanner::loadParameters() {
        ros::NodeHandle nh("~" + name_);
        nh.param<double>(std::string("turning_radius"), turning_radius_, 0.0);
        nh.param<int>("max_allowed_time", max_allowed_time_, 20);
        nh.param<double>("goal_tolerance", goal_tolerance_, 0.5);
        nh.param<double>("rickshaw_speed", rickshaw_speed_, 1.0);
        nh.param<bool>("enable_pathRefinement", enable_pathRefinement_, true);
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
            std_msgs::Int16 msg;
            msg.data = 32767;   // Invalid path --> set eta to int16_max value
            eta_publisher_.publish(msg);
            std_msgs::String msg2;
            std_msgs::Bool msg3;
            msg3.data = false;
            msg2.data = "No path found!";
            error_message_publisher_.publish(msg2);
            path_found_publisher_.publish(msg3);
            return false;
        } else {
            calculateEta(positions);
            std_msgs::String msg2;
            std_msgs::Bool msg3;
            msg2.data = "";
            msg3.data = true;
            error_message_publisher_.publish(msg2);
            path_found_publisher_.publish(msg3);
        }
        ros::Time plan_time = ros::Time::now();
        for (auto position : positions) {
            auto pose = position.toPoseStamped();
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame_;
            plan.push_back(pose);
        }
        publishPlan(plan, plan_publisher_);

        return true;
    }

    void QuadTreePlanner::publishPlan(std::vector<geometry_msgs::PoseStamped> &path, ros::Publisher &plan_publisher_) {
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

        // Quadtree based search
        // Quadtree based search
        ROS_INFO("Starting instantiation of data structures");

        Quadtree_SearchCell quad_start = Quadtree_SearchCell();
        quad_start = Quadtree_SearchCell(getQuad(start, QuadtreeSearchCellVector));
        Quadtree_SearchCell quad_goal = Quadtree_SearchCell();
        quad_goal = Quadtree_SearchCell(getQuad(goal, QuadtreeSearchCellVector));
        ROS_INFO("quadStart Top Left x: %i y: %i, Bottom Right x: %i, y:%i, cost: %i", quad_start.topLeft.x, quad_start.topLeft.y, quad_start.botRight.x, quad_start.botRight.y, quad_start.cost);
        set<QuadtreeCellWithDist> candidateQuads = {QuadtreeCellWithDist(0.0, quad_start)};
        unordered_map<Quadtree_SearchCell, Quadtree_SearchCell> parentsQuads;
        unordered_map<Pose, Pose> parentsQuadsPoses;
        unordered_map<Quadtree_SearchCell, double> pathLengthQuads = {{quad_start, 0.0}};
        Quadtree_SearchCell reached_quad;
        bool reached_goal_quad = false;

        ROS_INFO("Instantiation of data structures succesful");

        // Create variables to log performance
        time_t start_time = time(NULL);
        int num_nodes_visited = 0;

        ROS_INFO("Starting quad tree cell based search.");
        while (!candidateQuads.empty()) {
            // Checks if the maximum allowed time of the algorithm is already reached
            if (max_allowed_time_ > 0 && difftime(time(NULL), start_time) > max_allowed_time_) {
                break;
            }
            num_nodes_visited++;
            QuadtreeCellWithDist candQuad = *candidateQuads.begin();
            Pose candQuadPose = getPoseFromQuad(candQuad.quadtreeCell, goal);
            if (hasReachedGoalQuad(candQuad.quadtreeCell, goal)) {
                reached_quad = candQuad.quadtreeCell;
                reached_goal_quad = true;
                break;
            }
            else {
                double distToGoal = distEstimate(getPoseFromQuad(candQuad.quadtreeCell, goal), goal);
                //   ROS_INFO("The distance of the current candCell to the goal is %.2f", distToGoal);
            }
            double l_candQuad = pathLengthQuads[candQuad.quadtreeCell];

            candidateQuads.erase(candQuad);
            vector<QuadtreeCellWithDist> neighbors = getNeighborQuads(candQuad, goal);
            for (auto &nbr : neighbors) {
                if (candQuad.quadtreeCell == nbr.quadtreeCell) {
                    ROS_WARN("QuadTreePlanner: Oops, ended up in the same cell.");
                    continue;
                }
                // Checks if the neighboring cell contains an obstacle (cost > 0)
                // Checks if the neighboring cell contains an obstacle (cost > 0)
                if (nbr.quadtreeCell.getCost() > 0) {
                    continue;
                }
                // Check if neighboring quad cell is not contained in the pathLength set yet (meaning that it is not explored yet)
                // or if the distance (l_cand + nbr.dist) is smaller than the distance that is currently stored for
                // cell_nbr
                // if both conditions are fulfilled, the neighbor needs to be inserted in the candidatePoses set
                if (pathLengthQuads.find(nbr.quadtreeCell) == pathLengthQuads.end() || l_candQuad + nbr.dist < pathLengthQuads[nbr.quadtreeCell]) {
                    Pose nbr_quad_pose = getPoseFromQuad(nbr.quadtreeCell, goal);
             //       ROS_INFO("Debug Info: New Quad with cell coordinates Top Left %i x %i y, Bottom Right %i x %i y and cartesian coordinates %f x %f y inserted in candidateCells set", nbr.quadtreeCell.topLeft.x, nbr.quadtreeCell.topLeft.y, nbr.quadtreeCell.botRight.x, nbr.quadtreeCell.botRight.y, nbr_quad_pose.x, nbr_quad_pose.y);
                    pathLengthQuads[nbr.quadtreeCell] = l_candQuad + nbr.dist;
                    parentsQuads[nbr.quadtreeCell] = candQuad.quadtreeCell;
                    parentsQuadsPoses[nbr_quad_pose] = candQuadPose;
                    candidateQuads.insert(QuadtreeCellWithDist(l_candQuad + nbr.dist + distEstimate(getPoseFromQuad(nbr.quadtreeCell, goal), goal) ,nbr.quadtreeCell));
                }
            }

        }

        // Quad based search
        if (reached_goal_quad) {

            ROS_INFO("Quad tree based search reached the goal quad");
            Pose goal_quad_pose = getPoseFromQuad(reached_quad, goal);
            ROS_INFO("The found goal quad has the cell coordinates Top Left %i x %i y, Bottom Right %i x %i y and cartesian coordinates %f x %f y %f th", reached_quad.topLeft.x, reached_quad.topLeft.y, reached_quad.botRight.x, reached_quad.botRight.y, goal_quad_pose.x, goal_quad_pose.y, goal_quad_pose.th);
            double distanceGoalQuadRealGoal = distEstimate(goal_quad_pose, goal);
            ROS_INFO("The distance of the reached goal position to the real goal is %f m", distanceGoalQuadRealGoal);
            getPath(parentsQuadsPoses, goal_quad_pose, start, path);

            // Publish holonomic plan for debugging purposes
            publishHolonomicPlan(path);
            if(enable_pathRefinement_) {
                pathRefinement(reached_goal_quad, path);
            }
        } else {
            ROS_INFO("Quadtree cell based search did not reach the goal cell");
        }

        ROS_INFO("QuadTreePlanner finished in %.2fs, generated %d nodes, reached goal: %s",
                 difftime(time(NULL), start_time), num_nodes_visited, reached_goal_quad ? "true" : "false");

        return reached_goal_quad;
    }

    void QuadTreePlanner::pathRefinement(bool &reached_goal_quad, std::vector<quadtree_planner::Pose> &path){
        // Path Refinement (Dubin's car)
        // Considering the non-holonomic constraints of the rickshaw and calculating a smooth path
        bool refinement_finished = false;
        int first_index = 0;
        int second_index = path.size()-1;
        // In the loop, we try to connect the first position with a collision-free smooth path to the last position
        // If this does not work, we use the cell next to the goal position
        // the second index is decremented until we find a possible connection
        // then the loop continues with the remaining not yet connected cells
        while(!refinement_finished) {
            double q0[] = {path.at(first_index).x, path.at(first_index).y, path.at(first_index).th};
            double q1[] = {path.at(second_index).x, path.at(second_index).y, path.at(second_index).th};
            DubinsPath DubinsPath;
            Dubins_Poses_temp.clear();
            dubins_shortest_path( &DubinsPath, q0, q1, turning_radius_);
            dubins_path_sample_many(&DubinsPath, 0.05, createDubinsConfiguration, NULL);

            // Checking for collisions
            if(IsTrajectoryCollisionFree(Dubins_Poses_temp) == false) {;
                refinement_finished = false;
                if(second_index > first_index+1) {
                    second_index--;
                    // Use orientation of final goal as heuristic for a meaningful orientation
                    path.at(second_index).th = path.at(path.size()-1).th;
                } else {
                    ROS_INFO("No path refinement possible!");
                    ROS_INFO("Quadtree cell based search did find a path but it is not feasible due to the non-holonomic constraints");
                    reached_goal_quad = false;  // goal is not reachable considering the non-holonomic constraints
                    // the reason for this is usually that the final orientation is not feasible.
                    // depening on the requirements regarding correct orientation of the final position some adaptions might be possible
                    // in order to ignore the orientation at the final position
                    refinement_finished = true;
                }
            }
            else {  // Path is collision free
                Dubins_Poses_final.insert(Dubins_Poses_final.end(),Dubins_Poses_temp.begin(),Dubins_Poses_temp.end());
                if(second_index == (path.size() -1)) {  // goal index is reached
                    refinement_finished = true;
                } else {
                    first_index = second_index;
                    path.at(first_index).th = q1[2];    // holonomic path does not contain valid theta values --> correct theta value
                                                        // (orientation at end of Dubin's curve) must be set here in order to calculate
                                                        // correct subsequent Dubin's curve
                    second_index = path.size()-1;
                }
            }
        }
        // Refined path via Dubin's car
        path = Dubins_Poses_final;
        Dubins_Poses_final.clear();

        visualizeNonHolonomicPoses(path);
    }

    void QuadTreePlanner::visualizeNonHolonomicPoses(std::vector<Pose> &path) {
        // Publish poses for debugging
        std::vector<geometry_msgs::Pose> nonHolonomicPoses;
        for (int i = 0; i < path.size(); i+=10) {
            auto pose = path.at(i).toPose();
            nonHolonomicPoses.push_back(pose);
        }
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = global_frame_;
        poseArray.header.stamp = ros::Time::now();
        poseArray.poses.resize(nonHolonomicPoses.size());
        std::copy(nonHolonomicPoses.begin(), nonHolonomicPoses.end(), poseArray.poses.begin());
        nonHolonomicPathPoses_publisher_.publish(poseArray);
    }

    void QuadTreePlanner::publishHolonomicPlan(std::vector<Pose> &path) {
        ros::Time plan_time = ros::Time::now();
        std::vector<geometry_msgs::PoseStamped> holonomicPlan;
        for (auto position : path) {
            auto pose = position.toPoseStamped();
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame_;
            holonomicPlan.push_back(pose);
        }
        publishPlan(holonomicPlan, holonomic_plan_publisher_);

        // Publish poses for debugging
        std::vector<geometry_msgs::Pose> holonomicPoses;
        for (auto position : path) {
            auto pose = position.toPose();
            holonomicPoses.push_back(pose);
        }
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = global_frame_;
        poseArray.header.stamp = ros::Time::now();
        poseArray.poses.resize(holonomicPoses.size());
        std::copy(holonomicPoses.begin(), holonomicPoses.end(), poseArray.poses.begin());
        HolonomicPathPoses_publisher_.publish(poseArray);
    }

    bool QuadTreePlanner::validateParameters() const {
        if (turning_radius_ <= 0) {
            ROS_ERROR("QuadtreePlanner: turning radius has invalid value=%.2f. Must be greater than zero.",
                      turning_radius_);
            return false;
        }
        if (goal_tolerance_ <= 0) {
            ROS_ERROR("QuadtreePlanner: goal tolerance has invalid value=%.2f. Must be greater than zero",
                      goal_tolerance_);
            return false;
        }

        if (rickshaw_speed_ < 0) {
            ROS_ERROR("QuadtreePlanner: rickshaw speed has invalid value=%.2f. Must be greater than or equal to zero", rickshaw_speed_);
        }
        return true;
    }

    void QuadTreePlanner::getPath(const unordered_map<Pose, Pose> &parents,
                               const Pose &goal_pos, const Pose &start_pos,
                               vector<Pose> &path) const {
        auto curr_pose = goal_pos;
        while (true) {
            path.push_back(curr_pose);
            auto search = parents.find(curr_pose);
            if (search == parents.end()) {
                break;
            }
            curr_pose = search->second;
        }
        reverse(path.begin(), path.end());
        // Use correct start position (real start position instead of center of start quad cell)
        path.erase(path.begin());
        path.insert(path.begin(),start_pos);
    }

    vector<QuadtreeCellWithDist> QuadTreePlanner::getNeighborQuads(quadtree_planner::QuadtreeCellWithDist &quad, Pose goal) const {
        Quadtree_SearchCell quadSearchCell = Quadtree_SearchCell(quad.quadtreeCell);
        Quadtree_SearchCell quadtreeSearchCellFromVector;
        for(auto quad_cell: QuadtreeSearchCellVector) {
            if(quad_cell == quadSearchCell) {
                quadtreeSearchCellFromVector = Quadtree_SearchCell(quad_cell);
              //  ROS_INFO("Found quad in quad vector!");
              //  ROS_INFO("quad_cell Top Left x: %i y: %i, Bottom Right x: %i, y:%i, cost: %i", quad_cell.topLeft.x, quad_cell.topLeft.y, quad_cell.botRight.x, quad_cell.botRight.y, quad_cell.cost);
                break;
            } else {
           //     ROS_INFO("quad_cell Top Left x: %i y: %i, Bottom Right x: %i, y:%i, cost: %i", quad_cell.topLeft.x, quad_cell.topLeft.y, quad_cell.botRight.x, quad_cell.botRight.y, quad_cell.cost);
           //     ROS_INFO("quadSearchCell Top Left x: %i y: %i, Bottom Right x: %i, y:%i, cost: %i", quadSearchCell.topLeft.x, quadSearchCell.topLeft.y, quadSearchCell.botRight.x, quadSearchCell.botRight.y, quadSearchCell.cost);
            }
        }

        vector<Quadtree_SearchCell*> neighborSearchCellsPointers = quadtreeSearchCellFromVector.getNeighbors();
        vector<QuadtreeCellWithDist> return_vector;

        for(auto neighborPointer: neighborSearchCellsPointers) {
            Quadtree_SearchCell neighbor = *neighborPointer;
            return_vector.push_back(QuadtreeCellWithDist(distEstimate(getPoseFromQuad(quad.quadtreeCell, goal),getPoseFromQuad(neighbor,goal)),neighbor));
        }
        return return_vector;
    }

    Quadtree_SearchCell QuadTreePlanner::getQuad(const Pose &pos, std::vector<Quadtree_SearchCell> QuadtreeSearchCellVectorObject) {
        unsigned int cell_x = 0;
        unsigned int cell_y = 0;
        costmap_->worldToMap(pos.x, pos.y, cell_x, cell_y);
        for(auto quad: QuadtreeSearchCellVectorObject) {
            if( (cell_x >= quad.getTopLeft().x) && (cell_x <= quad.getBotRight().x) && (cell_y >= quad.getTopLeft().y) && (cell_y <= quad.getBotRight().y) ) {
                Quadtree_SearchCell new_quad;
                new_quad.topLeft = quad.topLeft;
                new_quad.botRight = quad.botRight;
                new_quad.cost = quad.cost;
                return new_quad;
            }
        }
    }

    Pose QuadTreePlanner::getPoseFromQuad(Quadtree_SearchCell &quad, Pose goal) const {
        Pose pos;
        unsigned int goal_map_x = 0;
        unsigned int goal_map_y = 0;

        costmap_->worldToMap(goal.x, goal.y, goal_map_x, goal_map_y);

        // Check if goal is contained in current quad cell
        if( (goal_map_x >= quad.topLeft.x) && (goal_map_x <= quad.botRight.x) && (goal_map_y >= quad.topLeft.y) && (goal_map_y <= quad.botRight.y)) {
            pos = goal;
        } else {   // Use center of quad tree cell in case goal is not contained in the current cell
            unsigned int quad_x = (quad.getBotRight().x + quad.getTopLeft().x)/2;
            unsigned int quad_y = (quad.getBotRight().y + quad.getTopLeft().y)/2;
            costmap_->mapToWorld(quad_x,quad_y, pos.x, pos.y);
        }

        return pos;
    }

    bool QuadTreePlanner::hasReachedGoalQuad(Quadtree_SearchCell &quad, const quadtree_planner::Pose &goal){
        Pose quadPose = getPoseFromQuad(quad, goal);

        return distEstimate(quadPose, goal) <= goal_tolerance_;
    }



    double QuadTreePlanner::distEstimate(const Pose &pose1, const Pose &pose2) const {
        return euclid_dist(pose1, pose2);
    }

    double QuadTreePlanner::getPathLength(std::vector<Pose> &path) {
        double path_length = 0;
        for(int i = 1; i < path.size(); i++) {
            path_length+= distEstimate(path.at(i), path.at(i-1));
        }
        return path_length;
    }

    void QuadTreePlanner::calculateEta(std::vector<quadtree_planner::Pose> positions){
        double eta = getPathLength(positions) / rickshaw_speed_;
        std_msgs::Int16 msg;
        msg.data = (int)eta;
        eta_publisher_.publish(msg);
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

    // Path Refinement (Dubin's car)
    bool QuadTreePlanner::IsTrajectoryCollisionFree(std::vector<Pose> pathVector) {
        for (auto pos : pathVector) {
            unsigned int cell_x = 0;
            unsigned int cell_y = 0;
            costmap_->worldToMap(pos.x, pos.y, cell_x, cell_y);
            if(costmap_->getCost(cell_x,cell_y)> 0) {
                return false;
            }
        }
        return true;
    }

    void QuadTreePlanner::inflateCostmap(double inflate_radius) {
        vector<Coordinates> mask;
        Coordinates coor;
        int discrete_radius = (int) ceil(inflate_radius / costmap_old->getResolution());
        unsigned int max_x = costmap_old->getSizeInCellsX();
        unsigned int max_y = costmap_old->getSizeInCellsY();
        unsigned int cost;
        int x_inf;
        int y_inf;

        // create mask to inflate the obstacles
        for(int x=-discrete_radius; x<=discrete_radius; x++){
            for(int y=-discrete_radius; y<=discrete_radius; y++){
                if(((abs(x)-1)*(abs(x)-1) + y*y <= discrete_radius*discrete_radius) || ((abs(y)-1)*(abs(y)-1) + x*x <= discrete_radius*discrete_radius)){
                    coor.x = x;
                    coor.y = y;
                    mask.push_back(coor);
                }
            }
        }

        for(unsigned int x=0; x<max_x; x++){
            for(unsigned int y=0; y<max_y; y++){
                int cost = costmap_old->getCost(x,y);
                if(cost > 255){
                    cost = 255;
                }
                if(cost > 0){
                    for(std::vector<Coordinates>::iterator it = mask.begin(); it != mask.end(); ++it) {
                        x_inf = ((int) x) + it->x;
                        y_inf = ((int) y) + it->y;
                        if(not((x_inf < 0) || (x_inf >= max_x) || (y_inf < 0) || (y_inf >= max_y))){
                            if(costmap_->getCost((unsigned int) x_inf, (unsigned int) y_inf) < cost){
                                costmap_->setCost((unsigned int) x_inf, (unsigned int) y_inf, cost);
                            }
                        }
                    }
                }
            }
        }

    }

}

// Dubin's car
int createDubinsConfiguration(double q[3], double x, void* user_data) {
//    ROS_INFO("%f, %f, %f, %f\n", q[0], q[1], q[2], x);
    quadtree_planner::Pose current_pos;
    current_pos.x = q[0];
    current_pos.y = q[1];
    current_pos.th = q[2];
    Dubins_Poses_temp.push_back(current_pos);
    return 0;
}


PLUGINLIB_EXPORT_CLASS(quadtree_planner::QuadTreePlanner,quadtree_planner::QuadTreePlanner);
