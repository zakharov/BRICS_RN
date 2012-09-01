/* 
 * File:   PathPlannerRos.cpp
 * Author: alexey
 * 
 * Created on August 30, 2012, 5:17 PM
 */

#include "navigation_trajectory_adapter/PathPlannerRos.h"
#include "navigation_trajectory_adapter/FrameWithId.h"
#include "navigation_trajectory_adapter/Conversions.h"
#include "navigation_trajectory_adapter/Stopwatch.h"
#include "navigation_trajectory_adapter/Logger.h"

#include <nav_core/base_global_planner.h>

PathPlannerRos::PathPlannerRos(nav_core::BaseGlobalPlanner* planner) : planner(planner) {
}

PathPlannerRos::PathPlannerRos(const PathPlannerRos& orig) :  planner(planner) {
}

PathPlannerRos::~PathPlannerRos() {
}

bool PathPlannerRos::computePath(const FrameWithId& start, const FrameWithId& goal, std::vector <FrameWithId>& path) {
    
    if (!planner) {
        ROS_ERROR("Planner instance in not allocated");
        return false;
    }
    
    geometry_msgs::PoseStamped startRos, goalRos; 
    conversions::frameToPoseStampedRos(start, startRos);
    conversions::frameToPoseStampedRos(goal, goalRos);
    std::vector<geometry_msgs::PoseStamped> pathRos;
    
#ifdef DEBUG
    Stopwatch stopwatch;
    stopwatch.start();
#endif    

    bool result = planner->makePlan(startRos, goalRos, pathRos);

#ifdef DEBUG    
    stopwatch.stop();
    LOG("A* path planning:");
    LOG("  - start: %f,%f,%f", startRos.pose.position.x, startRos.pose.position.y, startRos.pose.position.z);
    LOG("  - goal: %f,%f,%f", goalRos.pose.position.x, goalRos.pose.position.y, goalRos.pose.position.z);
    LOG("  - output size: %lu", pathRos.size());
    LOG("  - duration : %f ms", stopwatch.getElapsedTime());
#endif 

    conversions::pathRosToPath(pathRos, path);
    return result;
}