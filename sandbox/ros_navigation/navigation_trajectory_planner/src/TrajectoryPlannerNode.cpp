/* 
 * File:   TrajectoryPlannerNode.cpp
 * Author: alexey
 * 
 * Created on May 30, 2012, 4:21 PM
 */
#include "costmap_2d/costmap_2d_ros.h"
#include "pluginlib/class_loader.h"
#include "navigation_trajectory_planner/TrajectoryPlannerNode.h"
#include "nav_core/base_global_planner.h"


using namespace std;

TrajectoryPlannerNode* plannerNodeHandle = NULL;

void goalCallback(const geometry_msgs::PoseStamped&  goal) {
    if (plannerNodeHandle!= NULL) {
       plannerNodeHandle->publishTrajectory(goal);
       ROS_INFO("Received new goal, compute trajectory");
    }
}

void TrajectoryPlannerNode::publishTrajectory(const geometry_msgs::PoseStamped& goal) {
    
    
    tf::Stamped<tf::Pose> robotGlobalPose;
    globalCostmap.getRobotPose(robotGlobalPose);
    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(robotGlobalPose, start);
    
    std::vector<geometry_msgs::PoseStamped> path;
    planner->makePlan(start, goal, path);
    ROS_INFO("Planned a path, size: %d points", path.size());
    
    navigation_trajectory_planner::Trajectory trajectory;
    
    nav_msgs::Odometry odometry;
   
    
    vector <nav_msgs::Odometry>& trRef = trajectory.trajectory;
    
    ROS_INFO("Converting path to a trajectory");
    
    
    for (unsigned int i = 0; i < path.size(); i++) {
    
        odometry.header = path[i].header;
        odometry.pose.pose = path[i].pose;
        trRef.push_back(odometry);
    
    }
    
    trajectoryPublisher.publish(trajectory);
}

TrajectoryPlannerNode::TrajectoryPlannerNode(std::string name, costmap_2d::Costmap2DROS& costmap) : nodeName(name), globalCostmap(costmap),
        bgpLoader("nav_core", "nav_core::BaseGlobalPlanner"){
    
    ros::NodeHandle node = ros::NodeHandle("~/");
       
    node.param("global_costmap/robot_base_frame", robotBaseFrame, string("base_link"));
    node.param("global_costmap/global_frame", globalFrame, string("map"));
    node.param("global_costmap/trajectory_planner", globalTrajectoryPlanner, string("navfn/NavfnROS"));
        
    trajectoryPublisher = node.advertise<navigation_trajectory_planner::Trajectory> ("planned_trajectory", 1);
    goalSubscriber = node.subscribe("goal", 1, &goalCallback); 
    
     
    planner = bgpLoader.createClassInstance(globalTrajectoryPlanner);
    planner->initialize(bgpLoader.getName(globalTrajectoryPlanner), &globalCostmap);
    
}

TrajectoryPlannerNode::~TrajectoryPlannerNode() {
    delete planner;
}



int main(int argc, char **argv) {

  
    std::string name = "trajectory_planner";
    ros::init(argc, argv, name);
    tf::TransformListener tf;
    costmap_2d::Costmap2DROS globalCostmap("global_costmap", tf);
    ROS_INFO("Initialize costmap size: %d, %d", globalCostmap.getSizeInCellsX(), globalCostmap.getSizeInCellsY());
    
    TrajectoryPlannerNode trajectoryPlannerNode(name, globalCostmap);
    plannerNodeHandle = &trajectoryPlannerNode;
    ros::spin();
    
    
   
    
    return 0;
}
