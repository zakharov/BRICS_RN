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

#include "trajectory_generator/OmniDriveTrajectoryGenerator.h"
#include "navigation_trajectory_common/Conversions.h"

#include "omnidrive_controller/OmniDrivePositionController.h"
#include "navigation_trajectory_follower/TrajectoryFollowerNode.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include <navigation_trajectory_follower/ConversionUtils.h>

using namespace std;

ros::Publisher twistPublisher;
Odometry actualOdometry;
PositionController* controller;
ros::Time startTime;

ros::Publisher debugPath;

void poseRosToPose2d(const geometry_msgs::Pose& pose, Pose2D& pose2d) {

    double x = pose.position.x;
    double y = pose.position.y;
    double theta = tf::getYaw(pose.orientation);

    pose2d = Pose2D(x, y, theta);
}

void twistRosToTwist2d(const geometry_msgs::Twist& twist, Twist2D& twist2d) {

    double x = twist.linear.x;
    double y = twist.linear.y;
    double theta = twist.angular.z;

    twist2d = Twist2D(x, y, theta);

}

void pose2dToPoseRos(const Pose2D& pose2d, geometry_msgs::Pose& pose) {

    pose.position.x = pose2d.getX();
    pose.position.y = pose2d.getY();
    tf::Quaternion quat;

    quat.setRPY(0, 0, pose2d.getTheta());
    tf::quaternionTFToMsg(quat, pose.orientation);

}



void trajectoryRosToOdomVector(const navigation_trajectory_msgs::Trajectory& trajectoryROS, std::vector<Odometry>& odometryVector) {

    nav_msgs::Odometry odom;
    navigation_trajectory_msgs::Trajectory::_trajectory_type::const_iterator it;

    for (it = trajectoryROS.trajectory.begin(); it != trajectoryROS.trajectory.end(); ++it) {

        odom = *it;
        Pose2D pose2d;
        poseRosToPose2d(odom.pose.pose, pose2d);
        Twist2D twist2d;
        twistRosToTwist2d(odom.twist.twist, twist2d);
        odometryVector.push_back(Odometry(pose2d, twist2d));

    }

}

void twist2dToTwistRos(const Twist2D& twist2d, geometry_msgs::Twist& twist) {

    twist.linear.x = twist2d.getX();
    twist.linear.y = twist2d.getY();
    twist.angular.z = twist2d.getTheta();

}

void odomCallback(const nav_msgs::Odometry& odometry) {

    const geometry_msgs::Pose pose = odometry.pose.pose;
    const geometry_msgs::Twist twist = odometry.twist.twist;

    Pose2D pose2d;
    Twist2D twist2d;

    poseRosToPose2d(pose, pose2d);
    twistRosToTwist2d(twist, twist2d);

    actualOdometry = Odometry(pose2d, twist2d);

}

void pathCallback(const nav_msgs::Path& path) {
    OmniDriveTrajectoryGenerator trajectoryGenerator;
    
    std::vector<FrameWithId> pathKDL;
    conversions::pathRosToPath(path, pathKDL);
    ROS_INFO("Path size: %d", pathKDL.size());
    
    
    KDL::Trajectory_Composite trajectoryKDL;
    trajectoryGenerator.computeTrajectroy(pathKDL, trajectoryKDL);
    
    startTime = ros::Time::now();
    controller->setTargetTrajectory(trajectoryKDL);
}

void trajectoryCallback(const navigation_trajectory_msgs::Trajectory& trajectory) {

    std::vector <Odometry> odometry;
    trajectoryRosToOdomVector(trajectory, odometry);

    startTime = ros::Time::now();
    controller->setTargetTrajectory(odometry);

    nav_msgs::Path path;

    for (unsigned int i = 0; i < trajectory.trajectory.size(); i++) {

        geometry_msgs::PoseStamped pose;
        pose.pose = trajectory.trajectory[i].pose.pose;
        pose.header = trajectory.trajectory[i].header;
        pose.header.frame_id = "odom";
        path.poses.push_back(pose);

    }

    path.header = trajectory.header;
    path.header.frame_id = "odom";

    ROS_INFO("published %lu poses", path.poses.size());
    debugPath.publish(path);

}

void publishOdometry(const Odometry& odometry) {

    geometry_msgs::Twist twist;
    geometry_msgs::Pose pose;
    pose2dToPoseRos(odometry.getPose2D(), pose);
    twist2dToTwistRos(odometry.getTwist2D(), twist);

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

    string inputTrajectoryTopic;
    string inputPathTopic;
    string inputOdometryTopic;
    string inputVelocityTopic;

    node.param("cycleFrequencyInHz", cycleFrequencyInHz, 50.0);

    node.param("velocityGainTranslation", velocityGainTranslation, 1.33);
    node.param("positionGainTranslation", positionGainTranslation, 0.2);
    node.param("positionToleranceTranslation", positionToleranceTranslation, 0.1);
    node.param("velocityToleranceTranslation", velocityToleranceTranslation, 0.01);

    node.param("velocityGainRotation", velocityGainRotation, 1.5);
    node.param("positionGainRotation", positionGainRotation, 0.2);
    node.param("positionToleranceRotation", positionToleranceRotation, 0.1);
    node.param("velocityToleranceRotation", velocityToleranceRotation, 0.01);

    node.param<string > ("inputTrajectoryTopic", inputTrajectoryTopic, "localTrajectory");
    node.param<string > ("inputPathTopic", inputPathTopic, "simplified_path");
    node.param<string > ("inputOdometryTopic", inputOdometryTopic, "odom");
    node.param<string > ("outputVelocityTopic", inputVelocityTopic, "cmd_vel");

    //ros::Subscriber trajectorySubscriber;
    ros::Subscriber pathSubscriber;
    ros::Subscriber odomSubscriber;
    twistPublisher = globalNode.advertise<geometry_msgs::Twist > (inputVelocityTopic, 1);
    odomSubscriber = globalNode.subscribe(inputOdometryTopic, 1, &odomCallback);
    //trajectorySubscriber = globalNode.subscribe(inputTrajectoryTopic, 1, &trajectoryCallback);
    pathSubscriber = globalNode.subscribe(inputPathTopic, 1, &pathCallback);

    controller = new OmniDrivePositionController(positionGainTranslation,
            velocityGainTranslation,
            positionGainRotation,
            velocityGainRotation,
            positionToleranceTranslation,
            velocityToleranceTranslation,
            positionToleranceRotation,
            velocityToleranceRotation);

    ros::Rate r(cycleFrequencyInHz);

    debugPath = globalNode.advertise <nav_msgs::Path> ("debugRollingWindow", 1);

    while (ros::ok()) {
        ros::spinOnce();
        controlLoop();
        r.sleep();
    }

    delete controller;

    return 0;
}
