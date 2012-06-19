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

#include "OmniDrivePositionController.h"
#include "ConversionUtils.h"

#include <kdl/frames.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>
#include <cmath>

OmniDrivePositionController::OmniDrivePositionController() {
    tolerance = Odometry(Pose2D(0.1,0.1,0.1));
    trajectoryComposite = new KDL::Trajectory_Composite();
    resetFlags();
}

OmniDrivePositionController::OmniDrivePositionController(const OmniDrivePositionController& orig) : 
        trajectoryComposite(dynamic_cast<KDL::Trajectory_Composite*>(orig.trajectoryComposite->Clone())) {
    
    tolerance = orig.getTolerance();
    targetTrajectory = orig.getTargetTrajectory();
    targetOdometry = orig.getTargetOdometry();
    translationFlag = orig.translationFlag;
    rotationFlag = orig.rotationFlag;
    //TODO:: implement
}

OmniDrivePositionController::~OmniDrivePositionController() {
    delete trajectoryComposite;
}

void OmniDrivePositionController::resetFlags() {
    translationFlag = false;
    rotationFlag = false; 
}

void OmniDrivePositionController::setTargetOdometry(const Odometry& targetOdometry) {
    this->targetOdometry = targetOdometry;
}

const Odometry& OmniDrivePositionController::getTargetOdometry() const {
    return targetOdometry;
}

void OmniDrivePositionController::setTargetTrajectory(const std::vector <Odometry>& trajectory) {

    resetFlags();
    
    targetTrajectory = trajectory;

    trajectoryComposite->Destroy();
    trajectoryComposite = new KDL::Trajectory_Composite();

    if (targetTrajectory.size() > 1) { // if it has more than one setpoint 
        std::vector <Odometry>::const_iterator it;

        Odometry odom = targetTrajectory.front();
        KDL::Frame pose1;
        pose2dToFrameKdl(odom.getPose2D(), pose1);
        KDL::Twist twist1;
        twist2dToTwistKdl(odom.getTwist2D(), twist1);

        for (it = targetTrajectory.begin() + 1; it != targetTrajectory.end(); ++it) {

            odom = *it;
            KDL::Frame pose2;
            pose2dToFrameKdl(odom.getPose2D(), pose2);
            KDL::Twist twist2;
            twist2dToTwistKdl(odom.getTwist2D(), twist2);

            KDL::Path_Line* path = new KDL::Path_Line(pose1, pose2, new KDL::RotationalInterpolation_SingleAxis(), 0.1);
            KDL::VelocityProfile_Spline* velprof = new KDL::VelocityProfile_Spline();

            velprof->SetProfileDuration(0,
                    sqrt(twist1.vel.x() * twist1.vel.x() + twist1.vel.y() * twist1.vel.y()),
                    path->PathLength(),
                    sqrt(twist2.vel.x() * twist2.vel.x() + twist2.vel.y() * twist2.vel.y()), 0.2);

            KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(path, velprof);
            trajectoryComposite->Add(trajectorySegment);
            pose1 = pose2;
            twist1 = twist2;

        }
    }
}

const std::vector<Odometry>& OmniDrivePositionController::getTargetTrajectory() const {
    return targetTrajectory;
}

bool OmniDrivePositionController::isTargetReached() const {
       
    return translationFlag && rotationFlag;
}

void OmniDrivePositionController::targetReached(bool& translation, bool& rotation) {
    double duration = trajectoryComposite->Duration();
    KDL::Frame frame = trajectoryComposite->Pos(duration);
         
    double roll, pitch, yaw;
    frame.M.GetRPY(roll, pitch, yaw);
         
    Pose2D desiredPose(frame.p.x(), frame.p.x(), yaw);         
    Pose2D actualPose = actualOdometry.getPose2D();
         
    double angDist = getShortestAngle(desiredPose.getTheta(), actualPose.getTheta());
    double linDist = getDistance(desiredPose, actualPose);
         
    translation = false;
    rotation = false;
         
    if (linDist <= tolerance.getPose2D().getX() && linDist <= tolerance.getPose2D().getY())
        translation = true;
    if (angDist <= tolerance.getPose2D().getTheta())
        rotation = true;
}

const Odometry& OmniDrivePositionController::computeNewOdometry(const Odometry& actualOdometry, double elapsedTimeInSec) {

    this->actualOdometry = actualOdometry;
    computedOdometry = Odometry();
    
    if (trajectoryComposite != NULL && trajectoryComposite->Duration() > 0) {
        targetReached(translationFlag, rotationFlag);
        
        double actualTime = elapsedTimeInSec;

        KDL::Frame desiredPose = trajectoryComposite->Pos(actualTime);
        KDL::Twist desiredTwist = trajectoryComposite->Vel(actualTime);

        
        double r,p,y;
        desiredPose.M.GetRPY(r,p,y);
             
        double dPosX = desiredPose.p.x();
        double dPosY = desiredPose.p.y();
        double dPosTheta = y;
             
        double dVelX = desiredTwist.vel.x();
        double dVelY = desiredTwist.vel.y();
        double dVelTheta = desiredTwist.rot.z();
        
        double aPosX = actualOdometry.getPose2D().getX();
        double aPosY = actualOdometry.getPose2D().getY();

        double aVelX = actualOdometry.getTwist2D().getX();
        double aVelY = actualOdometry.getTwist2D().getY();

        double positionXError = dPosX - aPosX;
        double positionYError = dPosY - aPosY;

        double velocityXError = dVelX - aVelX;
        double velocityYError = dVelY - aVelY;

        double gain1 = 1;
        double gain2 = 1;
        double errorX = gain1 * positionXError + velocityXError;
        double errorY = gain2 * positionYError + velocityYError;
        double errorTheta = 0;
       

        computedOdometry.setTwist2D(Twist2D(errorX, errorY, errorTheta));
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

void OmniDrivePositionController::setTolerance(const Odometry& tolerance) {
    this->tolerance = tolerance;
}

const Odometry& OmniDrivePositionController::getTolerance() const {
    /* return tolerance;*/
    return tolerance;
}
