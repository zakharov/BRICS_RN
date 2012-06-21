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

#include "navigation_trajectory_adapter/TrajectoryAdapterNode.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <cmath>

using namespace std;

TrajectoryAdapterNode* adapterNodeHandle = NULL;

void odomCallback(const nav_msgs::Odometry& odometry) {
    adapterNodeHandle->setActualOdometry(odometry);
}

void trajectoryCallback(const navigation_trajectory_planner::Trajectory& trajectory) {
    adapterNodeHandle->setActualTrajectory(trajectory);
}

void TrajectoryAdapterNode::setActualOdometry(const nav_msgs::Odometry& odometry) {
    actualOdometry = odometry;
}

void TrajectoryAdapterNode::setActualTrajectory(const navigation_trajectory_planner::Trajectory& trajectory) {
    adaptedTrajectory.trajectory.clear();
    prune(trajectory, adaptedTrajectory);

}

void TrajectoryAdapterNode::publishTrajectory(const navigation_trajectory_planner::Trajectory& trajectory) {
    trajectoryPublisher.publish(trajectory);
}

void TrajectoryAdapterNode::prune(const navigation_trajectory_planner::Trajectory& globalTrajectory,
        navigation_trajectory_planner::Trajectory& localTrajectory) {

    using namespace navigation_trajectory_planner;

    Trajectory::_trajectory_type& localTrajectoryRef = localTrajectory.trajectory;
    const Trajectory::_trajectory_type& globalTrajectoryRef = globalTrajectory.trajectory;

    Trajectory::_trajectory_type::const_iterator it = globalTrajectoryRef.begin();


    geometry_msgs::Pose pose1;
    geometry_msgs::Pose pose2;
    geometry_msgs::Pose pose3;
    const float epsilon = 0.1;

    localTrajectoryRef.clear();
    localTrajectoryRef.push_back(*it);
    it++;

    pose2 = it->pose.pose;
    pose3 = (it + 1)->pose.pose;

    for (; it != globalTrajectoryRef.end(); it++) {

        pose1 = globalTrajectoryRef.back().pose.pose;
        pose2 = it->pose.pose;

        if ((it + 1) == globalTrajectoryRef.end()) {
            localTrajectoryRef.push_back(*(it));
            ROS_INFO("Adding last point");
            break;
        }

        pose3 = (it + 1)->pose.pose;

        float slope1 = (pose2.position.y - pose1.position.y) / (pose2.position.x - pose1.position.x);

        float slope2 = (pose3.position.y - pose1.position.y) / (pose3.position.x - pose1.position.x);

        float angle = atan(slope1);
        angle = angle + M_PI;
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
        ROS_INFO("Angle: %f,  Quat: %f,  %f,  %f, %f", angle, quat.x, quat.y, quat.z, quat.w);

        if (fabs(slope1 - slope2) > epsilon) {

            localTrajectoryRef.push_back(*(it));
            localTrajectoryRef.back().pose.pose.orientation = globalTrajectoryRef.back().pose.pose.orientation;
            ROS_INFO("Adding a point");
        } else {

            ROS_INFO("Skipping a point");
        }


    }


}

TrajectoryAdapterNode::TrajectoryAdapterNode(std::string name) : nodeName(name) {

    ros::NodeHandle node = ros::NodeHandle("~/");
    ros::NodeHandle globalNode = ros::NodeHandle();

    trajectoryPublisher = globalNode.advertise<navigation_trajectory_planner::Trajectory > ("localTrajectory", 1);
    odomSubscriber = globalNode.subscribe("odom", 1, &odomCallback);
    trajectorySubscriber = globalNode.subscribe("globalTrajectory", 1, &trajectoryCallback);
}

TrajectoryAdapterNode::~TrajectoryAdapterNode() {

}

double getDistance(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2) {
    double x1 = odom1.pose.pose.position.x;
    double y1 = odom1.pose.pose.position.y;
    double x2 = odom2.pose.pose.position.x;
    double y2 = odom2.pose.pose.position.y;

    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

bool TrajectoryAdapterNode::slideWindow(const navigation_trajectory_planner::Trajectory& globalTrajectory,
        const nav_msgs::Odometry& actualOdometry,
        navigation_trajectory_planner::Trajectory& localTrajectory) {

    using namespace navigation_trajectory_planner;

    Trajectory::_trajectory_type& localTrajectoryRef = localTrajectory.trajectory;
    const Trajectory::_trajectory_type& globalTrajectoryRef = globalTrajectory.trajectory;

    if (globalTrajectory.trajectory.empty()) {
        localTrajectoryRef.push_back(actualOdometry);
        return false;
    }

    double min = getDistance(globalTrajectoryRef[0], actualOdometry);
    unsigned int minIndex = 0;

    for (unsigned int i = 1; i < globalTrajectoryRef.size(); i++) {
        double actual = getDistance(globalTrajectoryRef[i], actualOdometry);
        if (actual < min) {
            min = actual;
            minIndex = i;
        }
    }

    if (minIndex >= globalTrajectoryRef.size()) {
        localTrajectoryRef.push_back(globalTrajectoryRef.back());
        return false;
    }





    for (unsigned int i = minIndex + 1; i < globalTrajectoryRef.size(); i++) {
        localTrajectoryRef.push_back(globalTrajectoryRef[i]);
    }

    return true;
}

void TrajectoryAdapterNode::controlLoop() {
    navigation_trajectory_planner::Trajectory trajectory;

    slideWindow(adaptedTrajectory, actualOdometry, trajectory);
    ROS_INFO("Publishing trajectory with size %d", trajectory.trajectory.size());
    publishTrajectory(trajectory);

}

int main(int argc, char **argv) {


    std::string name = "trajectory_adapter";
    ros::init(argc, argv, name);

    TrajectoryAdapterNode trajectoryAdapterNode(name);
    adapterNodeHandle = &trajectoryAdapterNode;

    ros::Rate r(20); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        trajectoryAdapterNode.controlLoop();
        r.sleep();
    }

    return 0;
}
