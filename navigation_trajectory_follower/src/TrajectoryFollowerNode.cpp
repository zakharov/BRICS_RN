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
#include "omnidrive_controller/OmniDrivePositionController.h"
#include "trajectory_generator/OmniDriveTrajectoryGenerator.h"

#include "navigation_trajectory_common/Conversions.h"
#include "navigation_trajectory_common/Utilities.h"
#include "navigation_trajectory_follower/TrajectoryFollowerNode.h"

#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"

#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include "kdl/trajectory_composite.hpp"

using namespace std;

ros::Publisher twistPublisher;
Odometry actualOdometry;
IPositionController* controller = 0;
ITrajectoryGenerator* trajectoryGenerator = 0;
ros::Time startTime;

ros::Publisher debugPath;

void odomCallback(const nav_msgs::Odometry& odometry) {

    const geometry_msgs::Pose pose = odometry.pose.pose;
    const geometry_msgs::Twist twist = odometry.twist.twist;

    Pose2D pose2d;
    Twist2D twist2d;

    conversions::poseRosToPose2d(pose, pose2d);
    conversions::twistRosToTwist2d(twist, twist2d);

    actualOdometry = Odometry(pose2d, twist2d);

}

void computePathComposite(const std::vector<FrameWithId>& path, KDL::Path_Composite& pathComposite) {
    std::vector<FrameWithId>::const_iterator it;

    if (path.size() > 1) {

        FrameWithId p1 = path.front();
        FrameWithId p2;

        for (it = path.begin() + 1; it != path.end(); ++it) {
            p2 = *it;
            KDL::Frame f1, f2;
            f1 = p1.getFrame();
            f2 = p2.getFrame();

            KDL::RotationalInterpolation_SingleAxis* rot = new KDL::RotationalInterpolation_SingleAxis();
            //   rot->Vel(0.1,0.01);
            //   rot->Acc(0.1,0.01,0.001);
            KDL::Path_Line* pathLine = new KDL::Path_Line(f1, f2, rot, 0.001);
            pathComposite.Add(pathLine);

            p1 = p2;
        }

    }
}

void interpolateRotation(const std::vector<FrameWithId>& path, std::vector<FrameWithId>& pathWithRotation) {

    if (path.size() <= 1)
        return;

    pathWithRotation.clear();

    double r, p, y;
    path.front().getFrame().M.GetRPY(r, p, y);
    double start = y;

    path.back().getFrame().M.GetRPY(r, p, y);
    double end = y;
    double step = utilities::getShortestAngle(end, start) / (path.size() - 1);


    std::vector<FrameWithId>::const_iterator it;
    for (it = path.begin(); it != path.end(); ++it) {

        KDL::Frame f;
        std::string id = it->id;

        f.p = it->getFrame().p;
        f.M = KDL::Rotation::RPY(0, 0, start);

        pathWithRotation.push_back(FrameWithId(f, id));

        start = start + step;
    }

}

void pathCallback(const nav_msgs::Path& path) {
    std::vector<FrameWithId> pathFWI;
    conversions::pathRosToPath(path, pathFWI);
    ROS_INFO("Path size: %u", pathFWI.size());

    /*
     * This is ugly and should really be part of trajectory generation, however
     * ITrajeczoryGenerator is (currently) defined as expecting a path including
     * rotation, i.e. poses instead of positions.
     */
    std::vector<FrameWithId> pathWithRotation;
    interpolateRotation(pathFWI, pathWithRotation);

    KDL::Path_Composite pathComposite;
    computePathComposite(pathWithRotation, pathComposite);
    /* End of what should go into trajectory generation. */

    KDL::Trajectory_Composite trajectoryKDL;

    trajectoryGenerator->computeTrajectroy(pathComposite, trajectoryKDL);

    startTime = ros::Time::now();
    controller->setTargetTrajectory(trajectoryKDL);
}

void publishOdometry(const Odometry& odometry) {

    geometry_msgs::Twist twist;
    geometry_msgs::Pose pose;
    conversions::pose2dToPoseRos(odometry.getPose2D(), pose);
    conversions::twist2dToTwistRos(odometry.getTwist2D(), twist);

    nav_msgs::Odometry odomRos;
    odomRos.pose.pose = pose;
    odomRos.twist.twist = twist;
    odomRos.header.stamp = ros::Time::now();

    twistPublisher.publish(twist);

}

void controlLoop() {

    double elapsedTime = ros::Duration(ros::Time::now() - startTime).toSec();
    Odometry newOdometry = controller->computeNewOdometry(actualOdometry, elapsedTime);
    publishOdometry(newOdometry);

}

int main(int argc, char **argv) {

    std::string name = "trajectory_follower";
    ros::init(argc, argv, name);

    ros::NodeHandle node = ros::NodeHandle("~/");
    ros::NodeHandle globalNode = ros::NodeHandle();

    double cycleFrequencyInHz;

    double velocityGainTranslation;
    double positionGainTranslation;
    double velocityToleranceTranslation;
    double positionToleranceTranslation;

    double velocityGainRotation;
    double positionGainRotation;
    double velocityToleranceRotation;
    double positionToleranceRotation;

    string inputPathTopic;
    string inputOdometryTopic;
    string inputVelocityTopic;

    node.param("cycleFrequencyInHz", cycleFrequencyInHz, 50.0);

    node.param("velocityGainTranslation", velocityGainTranslation, 1.33);
    node.param("positionGainTranslation", positionGainTranslation, 0.2);
    node.param("positionToleranceTranslation", positionToleranceTranslation, 0.1);
    node.param("velocityToleranceTranslation", velocityToleranceTranslation, 0.01);

    node.param("velocityGainRotation", velocityGainRotation, 1.35);
    node.param("positionGainRotation", positionGainRotation, 1.0);
    node.param("positionToleranceRotation", positionToleranceRotation, 0.1);
    node.param("velocityToleranceRotation", velocityToleranceRotation, 0.01);

    node.param<string > ("inputPathTopic", inputPathTopic, "simplified_path");
    node.param<string > ("inputOdometryTopic", inputOdometryTopic, "odom");
    node.param<string > ("outputVelocityTopic", inputVelocityTopic, "cmd_vel");


    ROS_INFO("velocityGainTranslation: %f", velocityGainTranslation);
    ROS_INFO("velocityGainRotation: %f", velocityGainRotation);
    ROS_INFO("positionGainTranslation: %f", positionGainTranslation);
    ROS_INFO("positionGainRotation: %f", positionGainRotation);


    //ros::Subscriber trajectorySubscriber;
    ros::Subscriber pathSubscriber;
    ros::Subscriber odomSubscriber;
    twistPublisher = globalNode.advertise<geometry_msgs::Twist > (inputVelocityTopic, 1);
    odomSubscriber = globalNode.subscribe(inputOdometryTopic, 1, &odomCallback);
    pathSubscriber = globalNode.subscribe(inputPathTopic, 1, &pathCallback);

    trajectoryGenerator = new OmniDriveTrajectoryGenerator;

    controller = new OmniDrivePositionController(positionGainTranslation,
            velocityGainTranslation,
            positionGainRotation,
            velocityGainRotation,
            positionToleranceTranslation,
            velocityToleranceTranslation,
            positionToleranceRotation,
            velocityToleranceRotation);

    ros::Rate r(cycleFrequencyInHz);

    debugPath = globalNode.advertise <nav_msgs::Path > ("debugRollingWindow", 1);

    while (ros::ok()) {
        ros::spinOnce();
        controlLoop();
        r.sleep();
    }

    delete controller;

    delete trajectoryGenerator;

    return 0;
}
