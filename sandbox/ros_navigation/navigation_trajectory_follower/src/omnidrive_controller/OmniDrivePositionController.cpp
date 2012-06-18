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

#include <kdl/frames.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>

#include "OmniDrivePositionController.h"
#include <cmath>

#include <ros/ros.h>

//#define DEBUG

#ifdef DEBUG
#include <iostream>
#endif

OmniDrivePositionController::OmniDrivePositionController() {
    
    trajectoryComposite = new KDL::Trajectory_Composite();
    
}

OmniDrivePositionController::OmniDrivePositionController(const VelocityRamp& linearVelocityRamp, const VelocityRamp& angularVelocitRamp, const Odometry& tolerance) {

}

OmniDrivePositionController::OmniDrivePositionController(const OmniDrivePositionController& orig) {
    
}

OmniDrivePositionController::~OmniDrivePositionController() {
    delete trajectoryComposite;
}

void pose2dToKdl(const Pose2D& pose2d, KDL::Frame& pose) {
    pose = KDL::Frame(KDL::Rotation::RPY(0, 0, pose2d.getTheta()), KDL::Vector(pose2d.getX(), pose2d.getY(), 0));
}

void twist2dToKdl(const Twist2D& twist2d, KDL::Twist& twist) {
    twist = KDL::Twist(KDL::Vector(twist2d.getX(), twist2d.getY(), 0), KDL::Vector(0, 0, twist2d.getTheta()));
}

void OmniDrivePositionController::setTargetTrajectory(const std::vector <Odometry>& trajectory) {

    ROS_INFO("Got traj: %d", trajectory.size());
    
    trajectoryComposite->Destroy();
    trajectoryComposite = new KDL::Trajectory_Composite();

    std::vector <Odometry>::const_iterator it;

    Odometry odom = trajectory.front();
    KDL::Frame pose1;
    pose2dToKdl(odom.getPose2D(), pose1);
    KDL::Twist twist1;
    twist2dToKdl(odom.getTwist2D(), twist1);

    for (it = trajectory.begin() + 1; it != trajectory.end(); ++it) {
        
        odom = *it;
        KDL::Frame pose2;
        pose2dToKdl(odom.getPose2D(), pose2);
        KDL::Twist twist2;
        twist2dToKdl(odom.getTwist2D(), twist2);

        KDL::Path_Line* path = new KDL::Path_Line(pose1, pose2, new KDL::RotationalInterpolation_SingleAxis(), 0.1);
        KDL::VelocityProfile_Spline* velprof = new KDL::VelocityProfile_Spline();

        velprof->SetProfileDuration(0,
                sqrt(twist1.vel.x() * twist1.vel.x() + twist1.vel.y() * twist1.vel.y()),
                path->PathLength(),
                sqrt(twist2.vel.x() * twist2.vel.x() + twist2.vel.y() * twist2.vel.y()), 0.2);


        std::cout << pose2 << std::endl;
        
        KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(path, velprof);
        trajectoryComposite->Add(trajectorySegment);
        pose1 = pose2;
        twist1 = twist2;
         ROS_INFO("Setting trajectory, duration: %f", trajectoryComposite->Duration());
    }

    ROS_INFO("Setting trajectory, duration: %f", trajectoryComposite->Duration());
}

void OmniDrivePositionController::setTargetOdometry(const Odometry& targetOdometry) {

}

const Odometry& OmniDrivePositionController::getTargetOdometry() {
    return targetOdometry;
}

bool OmniDrivePositionController::isTargetOdometryReached() {
    return linearOdometryReached && angularOdometryReached;
}

const Odometry& OmniDrivePositionController::computeNewOdometry(const Odometry& actualOdometry, double elapsedTimeInSec) {
   
   
    computedOdometry = Odometry();
    
    if (trajectoryComposite != NULL && trajectoryComposite->Duration() > 0) {
        
        double actualTime = elapsedTimeInSec;
               
        KDL::Frame desiredPose = trajectoryComposite->Pos(actualTime);
        KDL::Twist desiredTwist = trajectoryComposite->Vel(actualTime);

        double dPosX = desiredPose.p.x();
        double dPosY = desiredPose.p.y();

        double dVelX = desiredTwist.vel.x();
        double dVelY = desiredTwist.vel.y();

        double aPosX = actualOdometry.getPose2D().getX();
        double aPosY = actualOdometry.getPose2D().getY();

        double aVelX = actualOdometry.getTwist2D().getX();
        double aVelY = actualOdometry.getTwist2D().getY();

        double positionXError = dPosX - aPosX;
        double positionYError = dPosY - aPosY;

        double velocityXError = dVelX - aVelX;
        double velocityYError = dVelY - aVelY;

        double positionError = getDistance(Pose2D(dPosX, dPosY, 0), Pose2D(aPosX, aPosY, 0));
        double velocityError = getDistance(Pose2D(dVelX, dVelY, 0), Pose2D(aVelX, aVelY, 0));

        double gain1 = 1;
        double gain2 = 1;
        double errorX = gain1 * positionXError + velocityXError;
        double errorY = gain2 * positionYError + velocityYError;
        
        ROS_INFO("error_x:%f, error_y:%f", errorX, errorY);

        computedOdometry.setTwist2D(Twist2D(errorX, errorY, 0));
    }

    return computedOdometry;
}

float OmniDrivePositionController::getShortestAngle(float goalAngle, float actualAngle) {
    return atan2(sin(goalAngle - actualAngle), cos(goalAngle - actualAngle));
}

float OmniDrivePositionController::getDistance(const Pose2D& actualPose, const Pose2D& goalPose) {
    return sqrt((actualPose.getX() - goalPose.getX()) * (actualPose.getX() - goalPose.getX()) +
            (actualPose.getY() - goalPose.getY()) * (actualPose.getY() - goalPose.getY()));

}

Twist2D OmniDrivePositionController::computeAngularVelocity(const Pose2D& actualPose, const Pose2D& initialPose, const Pose2D& goalPose) {

    return Twist2D(0, 0, 0);
}

Twist2D OmniDrivePositionController::computeLinearVelocity(const Pose2D& actualPose, const Pose2D& initialPose, const Pose2D& goalPose) {


    return Twist2D(0, 0, 0);
}

void OmniDrivePositionController::setTolerance(const Odometry& tolerance) {
    /*  this->tolerance = tolerance;*/
}

const Odometry& OmniDrivePositionController::getTolerance() {
    /* return tolerance;*/
}
