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

#include <iostream>

using namespace std;

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
    if (result == true) { // plan has no orientation

        ROS_INFO("Found a path, which has %lu points", poseStampedArray.size());
        ROS_INFO("Initial pose origin (x,y,z): %f, %f, %f",
                poseStampedArray.front().pose.position.x,
                poseStampedArray.front().pose.position.y,
                poseStampedArray.front().pose.position.z);
        ROS_INFO("      orientation (w,x,y,z): %f, %f, %f, %f",
                poseStampedArray.front().pose.orientation.w,
                poseStampedArray.front().pose.orientation.x,
                poseStampedArray.front().pose.orientation.y,
                poseStampedArray.front().pose.orientation.z);

        ROS_INFO("Goal pose origin (x,y,z): %f, %f, %f",
                poseStampedArray.back().pose.position.x,
                poseStampedArray.back().pose.position.y,
                poseStampedArray.back().pose.position.z);
        ROS_INFO("   orientation (w,x,y,z): %f, %f, %f, %f",
                poseStampedArray.back().pose.orientation.w,
                poseStampedArray.back().pose.orientation.x,
                poseStampedArray.back().pose.orientation.y,
                poseStampedArray.back().pose.orientation.z);

        poseStampedArray.front() = initialPoseStamped;
        std::vector<geometry_msgs::PoseStamped>::iterator it;

        ConversionUtils convert;

        double start = tf::getYaw(initialPoseStamped.pose.orientation);
        double end = tf::getYaw(goalPoseStamped.pose.orientation);
        double step = getShortestAngle(end, start) / poseStampedArray.size();

        for (it = poseStampedArray.begin(); it != poseStampedArray.end() - 1; ++it) {

            KDL::Frame f(KDL::Rotation::RPY(0, 0, start), KDL::Vector(0, 0, 0));

            start = start + step;

            geometry_msgs::Pose f1;
            convert.poseKdlToRos(f, f1);

            it->pose.orientation = f1.orientation;

        }


        KDL::Frame f(KDL::Rotation::RPY(0, 0, end), KDL::Vector(0, 0, 0));

        geometry_msgs::Pose f1;
        convert.poseKdlToRos(f, f1);

        poseStampedArray.back().pose.orientation = f1.orientation;
    }

    conversion.pathRosToKdl(poseStampedArray, path);

    return true;
}

bool TrajectoryPlanner::computeTrajectory(const KDL::Path& path, KDL::Trajectory_Composite& trajectory) {

    const double maxVel = 0.3;
    const double maxAcc = 0.05;

    ROS_INFO("Path converted to a trajectory, max velocity: %f m/s, max acceleration: %f m/s^2", maxVel, maxAcc);

    KDL::VelocityProfile_Trap* velocityProfile = new KDL::VelocityProfile_Trap(maxVel, maxAcc);
    KDL::Path* copyPath = const_cast<KDL::Path&> (path).Clone(); // Why KDL has no const version of Clone method?
    // They force me to do this!

    velocityProfile->SetProfile(0, copyPath->PathLength());
    KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(copyPath, velocityProfile);

    trajectory.Add(trajectorySegment);

    double dt = 1.0;
    for (double i = 0; i <= trajectory.Duration() + dt; i = i + dt) {
        cout << "pos" << ":" << trajectory.Pos(i).p <<  endl;
        cout << "vel" << ":" << trajectory.Vel(i).vel <<  endl;
        cout << "acc" << ":" << trajectory.Acc(i).vel <<  endl;
        cout << "---" << endl;
    }
    
    
    return true;
}
