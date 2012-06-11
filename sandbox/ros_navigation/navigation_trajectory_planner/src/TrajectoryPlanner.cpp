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

#include "navigation_trajectory_planner/TrajectoryPlanner.h"
#include "navigation_trajectory_planner/ConversionUtils.h"

#include <kdl/frames.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>
#include <nav_core/base_global_planner.h>



TrajectoryPlanner::TrajectoryPlanner(nav_core::BaseGlobalPlanner* pathPlanner) {
    this->pathPlanner = pathPlanner;
}

TrajectoryPlanner::TrajectoryPlanner(const TrajectoryPlanner& orig) {
}

TrajectoryPlanner::~TrajectoryPlanner() {
}

bool TrajectoryPlanner::computePath(const KDL::Frame& initial, const KDL::Frame& goal, KDL::Path_Composite& path) {

    geometry_msgs::PoseStamped initialPoseStamped;
    geometry_msgs::PoseStamped goalPoseStamped;
    
    ConversionUtils conversion;
    
    conversion.poseKdlToRos(initial, initialPoseStamped.pose);
    conversion.ConversionUtils::poseKdlToRos(goal, goalPoseStamped.pose);
    
    
    std::vector<geometry_msgs::PoseStamped> poseStampedArray;
       
    pathPlanner->makePlan(initialPoseStamped, goalPoseStamped, poseStampedArray);

    conversion.pathRosToKdl(poseStampedArray, path);
    
    return true;
}

bool TrajectoryPlanner::computeTrajectory(const KDL::Path& path, KDL::Trajectory_Composite& trajectory) {

    const double maxVel = 1.0;
    const double maxAcc = 0.1;
    
    KDL::VelocityProfile_Trap* velocityProfile = new KDL::VelocityProfile_Trap(maxVel, maxAcc);

    KDL::Path* copyPath = const_cast<KDL::Path&>(path).Clone();        // Why KDL has no const version of Clone method?
                                                                       // They force me to do this!
    
    KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(copyPath, velocityProfile);
    trajectory.Add(trajectorySegment);
    
    return true;
}


/*
void TrajectoryPlannerNode::prunePlan(const std::vector<geometry_msgs::PoseStamped>& actualPlan, std::vector<geometry_msgs::PoseStamped>& prunedPlan) {

    if (actualPlan.size() > 2) { //if plan has more than 2 points, try to prune

        vector<geometry_msgs::PoseStamped>::const_iterator it = actualPlan.begin();
        geometry_msgs::Pose pose1;
        geometry_msgs::Pose pose2;
        geometry_msgs::Pose pose3;
        const float epsilon = 0.1;

        prunedPlan.clear();
        prunedPlan.push_back(actualPlan.front());
        it++;
        pose2 = it->pose;
        pose3 = (it + 1)->pose;

        while (it != actualPlan.end()) {

            pose1 = actualPlan.back().pose;
            pose2 = it->pose;

            if ((it + 1) == actualPlan.end()) {
                prunedPlan.push_back(*(it));
                ROS_INFO("Adding last point");
                break;
            }

            pose3 = (it + 1)->pose;

            float slope1 = (pose2.position.y - pose1.position.y) / (pose2.position.x - pose1.position.x);
            float slope2 = (pose3.position.y - pose1.position.y) / (pose3.position.x - pose1.position.x);

            if (fabs(slope1 - slope2) > epsilon) {

                prunedPlan.push_back(*(it));
                ROS_INFO("Adding a point");
            } else {

                ROS_INFO("Skipping a point");
            }

            ++it;
        }
    } else
        prunedPlan = actualPlan;

}

void TrajectoryPlannerNode::computeTrajectory(vector <nav_msgs::Odometry>& trajectory) {


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

    trRef.front().pose.pose.orientation = start.pose.orientation;
    trRef.back().pose.pose.orientation = goal.pose.orientation;
    geometry_msgs::Quaternion quat = trRef.back().pose.pose.orientation;
    ROS_INFO("Goal orientation x=%f y=%f z=%f w=%f", quat.x, quat.y, quat.z, quat.w);

    trajectoryPublisher.publish(trajectory);
}

TrajectoryPlannerNode::TrajectoryPlannerNode(std::string name, costmap_2d::Costmap2DROS& costmap) : nodeName(name), globalCostmap(costmap),
bgpLoader("nav_core", "nav_core::BaseGlobalPlanner") {

    ros::NodeHandle node = ros::NodeHandle("~/");
    ros::NodeHandle globalNode = ros::NodeHandle();

    node.param("global_costmap/robot_base_frame", robotBaseFrame, string("base_link"));
    node.param("global_costmap/global_frame", globalFrame, string("map"));
    node.param("global_costmap/trajectory_planner", globalTrajectoryPlanner, string("navfn/NavfnROS"));

    trajectoryPublisher = globalNode.advertise<navigation_trajectory_planner::Trajectory > ("trajectory", 1);
    goalSubscriber = globalNode.subscribe("goal", 1, &goalCallback);


    planner = bgpLoader.createClassInstance(globalTrajectoryPlanner);
    planner->initialize(bgpLoader.getName(globalTrajectoryPlanner), &globalCostmap);

}

TrajectoryPlannerNode::~TrajectoryPlannerNode() {
    delete planner;
}

 * */