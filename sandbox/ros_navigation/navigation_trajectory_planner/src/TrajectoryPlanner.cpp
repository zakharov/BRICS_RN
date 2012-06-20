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


void TrajectoryPlanner::setPathFrameId(std::string frameId) {
    this->frameId = frameId;
}

float getShortestAngle(float goalAngle, float actualAngle) {
    return atan2(sin(goalAngle - actualAngle), cos(goalAngle - actualAngle));
}

bool TrajectoryPlanner::computePath(const KDL::Frame& initial, const KDL::Frame& goal, KDL::Path_Composite& path) {

    geometry_msgs::PoseStamped initialPoseStamped;
    geometry_msgs::PoseStamped goalPoseStamped;
    ConversionUtils conversion;

    conversion.poseKdlToRos(initial, initialPoseStamped.pose);
    initialPoseStamped.header.frame_id = frameId;
    conversion.ConversionUtils::poseKdlToRos(goal, goalPoseStamped.pose);
    goalPoseStamped.header.frame_id = frameId;


    std::vector<geometry_msgs::PoseStamped> poseStampedArray;
    bool result = pathPlanner->makePlan(initialPoseStamped, goalPoseStamped, poseStampedArray);
    if (result == true) {       // plan has no orientation
        poseStampedArray.front() = initialPoseStamped;  
        std::vector<geometry_msgs::PoseStamped>::iterator it;
        
        ConversionUtils convert;
        
        
        
        double start = tf::getYaw(initialPoseStamped.pose.orientation);
        
        double end = tf::getYaw(goalPoseStamped.pose.orientation);
        
        
        double step = getShortestAngle(end, start) / poseStampedArray.size();
       
        
        
        for (it = poseStampedArray.begin(); it != poseStampedArray.end(); ++it) {
            
           
            
            KDL::Frame f(KDL::Rotation::RPY(0,0,start), KDL::Vector(0,0,0));
            start = start + step;
            
            geometry_msgs::Pose f1;
            convert.poseKdlToRos(f,f1);
            
            it->pose.orientation = f1.orientation;
        }
        
       
        
        ROS_INFO("Initial pose: %f,%f,%f,%f", initialPoseStamped.pose.orientation.w, 
                initialPoseStamped.pose.orientation.x, 
                initialPoseStamped.pose.orientation.y, 
                initialPoseStamped.pose.orientation.z);
        ROS_INFO("Goal pose: %f,%f,%f,%f", goalPoseStamped.pose.orientation.w,
                goalPoseStamped.pose.orientation.x,
                goalPoseStamped.pose.orientation.y,
                goalPoseStamped.pose.orientation.z);
      
        
    }
     
    conversion.pathRosToKdl(poseStampedArray, path);

    return true;
}

bool TrajectoryPlanner::computeTrajectory(const KDL::Path& path, KDL::Trajectory_Composite& trajectory) {

    const double maxVel = 0.5;
    const double maxAcc = 0.1;

    double dur = trajectory.Duration();


    KDL::VelocityProfile_Trap* velocityProfile = new KDL::VelocityProfile_Trap(maxVel, maxAcc);

    KDL::Path* copyPath = const_cast<KDL::Path&> (path).Clone(); // Why KDL has no const version of Clone method?
                                                                 // They force me to do this!

    velocityProfile->SetProfile(0, copyPath->PathLength());

    KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(copyPath, velocityProfile);

    trajectory.Add(trajectorySegment);

    return true;
}
