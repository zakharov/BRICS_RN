/* 
 * File:   TrajectoryPlannerNode.h
 * Author: alexey
 *
 * Created on May 30, 2012, 4:21 PM
 */

#ifndef TRAJECTORYPLANNERNODE_H
#define	TRAJECTORYPLANNERNODE_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "navigation_trajectory_planner/Trajectory.h"
#include "pluginlib/class_loader.h"
#include <string>
#include <vector>

namespace costmap_2d {
class Costmap2DROS;
}

namespace nav_core {
class BaseGlobalPlanner;
}

class TrajectoryPlannerNode {
public:
    TrajectoryPlannerNode(std::string name, costmap_2d::Costmap2DROS& costmap);
    TrajectoryPlannerNode(const TrajectoryPlannerNode& orig);
    void publishTrajectory(const geometry_msgs::PoseStamped& goal);
    virtual ~TrajectoryPlannerNode();
  
    
private:
    
    ros::NodeHandle node;
    
    ros::Publisher trajectoryPublisher;
    ros::Subscriber costmapSubscriber;
    ros::Subscriber goalSubscriber;
    
    std::string robotBaseFrame;
    std::string globalFrame;
    std::string nodeName;
    std::string globalTrajectoryPlanner;
    costmap_2d::Costmap2DROS& globalCostmap;
    nav_core::BaseGlobalPlanner* planner;
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgpLoader;
    
};

#endif	/* TRAJECTORYPLANNERNODE_H */

