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

#ifndef TRAJECTORYFOLLOWERNODE_H
#define	TRAJECTORYFOLLOWERNODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "Trajectory.h"
#include <geometry_msgs/Twist.h>
#include <kdl/frames.hpp>

#include <string>
#include <vector>

class PositionController;


namespace KDL {
    class Trajectory;
    class Frame;
    class Trajectory_Composite;
    class VelocityProfile_Trap;

};

class TrajectoryFollowerNode {
public:
    TrajectoryFollowerNode(std::string name);
    TrajectoryFollowerNode(const TrajectoryFollowerNode& orig);
    void publishTwist(const geometry_msgs::Twist& twist);

    void setActualOdometry(const nav_msgs::Odometry& odometry);
    void setActualTrajectory(const navigation_trajectory_planner::Trajectory& trajectory);


    virtual ~TrajectoryFollowerNode();
    void controlLoop();

private:

    double actualTime, startTime;

    KDL::Trajectory*createTrajectoryKDL(const navigation_trajectory_planner::Trajectory& trajectory);
    void createKDLFrame(const nav_msgs::Odometry& odometry, KDL::Frame& frame);

    KDL::Trajectory* actualTrajectoryKDL;

    ros::NodeHandle node;

    ros::Subscriber trajectorySubscriber;
    ros::Subscriber odomSubscriber;
    ros::Publisher twistPublisher;

    std::string nodeName;

    PositionController* controller;
    nav_msgs::Odometry actualOdometry;
    navigation_trajectory_planner::Trajectory actualTrajectory;
    geometry_msgs::Twist actualAcceleration;
    KDL::Twist desiredTwist;
    KDL::Frame desiredPose;

    KDL::Trajectory_Composite* trajectoryComposite_x;
    KDL::Trajectory_Composite* trajectoryComposite_y;
    KDL::VelocityProfile_Trap* velpref_x;
    KDL::VelocityProfile_Trap* velpref_y;

};

#endif	/* TRAJECTORYFOLLOWERNODE_H */

