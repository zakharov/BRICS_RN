/******************************************************************************
* Copyright (c) 2011
* GPS GmbH
*
* Author:
* Alexey Zakharov
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of GPS GmbH nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_loader.h>
#include <navigation_trajectory_planner/Trajectory.h>
#include <nav_msgs/Odometry.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_loader.h>
#include <string>

#include "navigation_trajectory_planner/TrajectoryPlanner.h"

using namespace std;

TrajectoryPlanner* plannerNodePtr = NULL;
ros::Publisher* trajectoryPublisherPtr = NULL;

void publishTrajectory(vector <nav_msgs::Odometry>& trajectory) {
    if (!trajectory.empty()) {
        ROS_INFO("Publishing trajectory, size: %d", trajectory.size());ROS_INFO("Received new goal, compute trajectory");
    
    }
    else {
        ROS_WARN("Trajectory planner returned empty trajectory, skipping.");
    }
        
}

void goalCallback(const geometry_msgs::PoseStamped& goal) {
    if (plannerNodePtr != NULL) {
        ROS_INFO("Trajectory planner received new goal, computing a trajectory");
              
        // Computing a path, given a goal pose
        std::vector<geometry_msgs::PoseStamped> plan;
        plannerNodePtr->computePlan(goal, plan);
        
        // Computing a pruned path, given the original path
        // TODO: make pruning optional 
        std::vector<geometry_msgs::PoseStamped> prunedPlan;
        plannerNodePtr->prunePlan(plan, prunedPlan);
        
        // Computing a trajectory, given the path
        std::vector<nav_msgs::Odometry> trajectory;
        plannerNodePtr->computeTrajectory(prunedPlan, trajectory);
        
        // Finally, publishing a trajectory
        publishTrajectory(trajectory);
    }
}


 int main(int argc, char **argv) {

    // initializing ROS node
    std::string name = "trajectory_planner";
    ros::init(argc, argv, name);
    
    // TODO:: read topic names and configuration parameters from yaml files
    // Instantiating a costmap
    tf::TransformListener tf;
    costmap_2d::Costmap2DROS globalCostmap("global_costmap", tf); 
    ROS_INFO("Initialize costmap size: %d, %d", globalCostmap.getSizeInCellsX(), globalCostmap.getSizeInCellsY());

    // Reading configuration parameters 
    ros::NodeHandle node = ros::NodeHandle("~/");
    string globalTrajectoryPlanner;
    node.param("global_costmap/trajectory_planner", globalTrajectoryPlanner, string("navfn/NavfnROS")); 
        
    // Initializing subscribers and publishers
    ros::NodeHandle globalNode = ros::NodeHandle("");
    ros::Publisher trajectoryPublisher;
    trajectoryPublisher = globalNode.advertise<navigation_trajectory_planner::Trajectory > ("globalTrajectory", 1);
    trajectoryPublisherPtr = &trajectoryPublisher;
    
    ros::Subscriber goalSubscriber;
    goalSubscriber = globalNode.subscribe("goal", 1, &goalCallback);

    // Loading a planner plugin
    nav_core::BaseGlobalPlanner* planner;
    //TODO: read planners names from configuration file
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgpLoader("nav_core", "nav_core::BaseGlobalPlanner");
    planner = bgpLoader.createClassInstance(globalTrajectoryPlanner);
    planner->initialize(bgpLoader.getName(globalTrajectoryPlanner), &globalCostmap);

    // Instantiating trajectory planner
    TrajectoryPlanner trajectoryPlanner;
    plannerNodePtr = &trajectoryPlanner;
    
    // Starting execution thread
    ros::spin();

    return 0;
}
