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

#include <ros/ros.h>

#include <kdl/frames.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>

#include "OmniDriveKinematicsModel.h"

#include <cmath>
#include <iostream>

using namespace std;

OmniDrivePositionController::OmniDrivePositionController(double positionGainTranslation,
        double velocityGainTranslation,
        double positionGainRotation,
        double velocityGainRotation,
        double positionToleranceTranslation,
        double velocityToleranceTranslation,
        double positionToleranceRotation,
        double velocityToleranceRotation) {

    actualTime = 0;
    timeOffset = 0;

    

    tolerance = Odometry(Pose2D(positionToleranceTranslation,
            positionToleranceTranslation,
            positionToleranceRotation),
            Twist2D(velocityToleranceTranslation,
            velocityToleranceTranslation,
            velocityToleranceRotation));

    gains = Odometry(Pose2D(positionGainTranslation,
            positionGainTranslation,
            positionGainRotation),
            Twist2D(velocityGainTranslation,
            velocityGainTranslation,
            velocityGainRotation));

    trajectoryComposite = new KDL::Trajectory_Composite();
    resetFlags();

}

OmniDrivePositionController::OmniDrivePositionController() {
    actualTime = 0;
    timeOffset = 0;
    tolerance = Odometry(Pose2D(0.1, 0.1, 0.05));
    trajectoryComposite = new KDL::Trajectory_Composite();
    resetFlags();
}

OmniDrivePositionController::OmniDrivePositionController(const OmniDrivePositionController& orig) :
trajectoryComposite(dynamic_cast<KDL::Trajectory_Composite*> (orig.trajectoryComposite->Clone())) {

    actualTime = 0;
    timeOffset = 0;
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

void OmniDrivePositionController::setTargetTrajectory(const KDL::Trajectory_Composite& trajectory) {
    resetFlags();

    KDL::Trajectory_Composite* newTrajectoryComposite = dynamic_cast<KDL::Trajectory_Composite*>(trajectory.Clone());

    trajectoryComposite->Destroy();
    trajectoryComposite = newTrajectoryComposite;
    
  
    
}

void OmniDrivePositionController::setTargetTrajectory(const std::vector <Odometry>& trajectory) {

    resetFlags();

    targetTrajectory = trajectory;

    //    trajectoryComposite->Destroy();

    KDL::Trajectory_Composite* newTrajectoryComposite;
    newTrajectoryComposite = new KDL::Trajectory_Composite();

    if (targetTrajectory.size() > 1) { // if it has more than one setpoint 
        std::vector <Odometry>::const_iterator it;

        Odometry odom = targetTrajectory.front();
        KDL::Frame pose1;
        pose2dToFrameKdl(odom.getPose2D(), pose1);

        for (it = targetTrajectory.begin() + 1; it != targetTrajectory.end(); ++it) {

            odom = *it;
            KDL::Frame pose2;
            pose2dToFrameKdl(odom.getPose2D(), pose2);

            KDL::Path_Line* path = new KDL::Path_Line(pose1, pose2, new KDL::RotationalInterpolation_SingleAxis(), 0.001);
            KDL::VelocityProfile_Spline* velprof = new KDL::VelocityProfile_Spline();

            velprof->SetProfileDuration(0, path->PathLength(), 0.2);

            KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(path, velprof);
            newTrajectoryComposite->Add(trajectorySegment);
            pose1 = pose2;

        }

        if (trajectoryComposite != NULL && trajectoryComposite->Duration() > 0) {
            targetReached(translationFlag, rotationFlag);

            KDL::Frame actualPoseKDL = trajectoryComposite->Pos(actualTime);
            Pose2D actualPose(actualPoseKDL.p.x(), actualPoseKDL.p.y(), 0);
            KDL::Frame newPoseKDL = newTrajectoryComposite->Pos(0);
            Pose2D newPose(newPoseKDL.p.x(), newPoseKDL.p.y(), 0);
            double min = getDistance(actualPose, newPose);
            for (float i = 0.01; i <= newTrajectoryComposite->Duration(); i = i + 0.01) {
                newPoseKDL = newTrajectoryComposite->Pos(i);
                Pose2D newPose(newPoseKDL.p.x(), newPoseKDL.p.y(), 0);
                double res = getDistance(actualPose, newPose);
                if (res > min) {
                    ROS_INFO("Start from %f msec", i);
                    timeOffset = i;
                    break;
                } else {
                    timeOffset = 0;
                    min = res;
                }
            }
        }

        trajectoryComposite->Destroy();
        trajectoryComposite = newTrajectoryComposite;
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

    Pose2D desiredPose(frame.p.x(), frame.p.y(), yaw);
    Pose2D actualPose = actualOdometry.getPose2D();

    double angDist = getShortestAngle(desiredPose.getTheta(), actualPose.getTheta());
    double linDist = getDistance(desiredPose, actualPose);

    translation = false;
    rotation = false;

    if (linDist <= tolerance.getPose2D().getX() && linDist <= tolerance.getPose2D().getY()) {
     //   ROS_INFO("Translation target reached");
        translation = true;
    }

   // ROS_INFO("Desired theta: %f ----- %f", fabs(angDist), tolerance.getPose2D().getTheta());

    if (fabs(angDist) <= tolerance.getPose2D().getTheta()) {
       // ROS_INFO("Rotation target reached");
        rotation = true;
    }
}

const Odometry& OmniDrivePositionController::computeNewOdometry(const Odometry& actualOdometry, double elapsedTimeInSec) {

    this->actualOdometry = actualOdometry;
    computedOdometry = Odometry();


    if (trajectoryComposite != NULL && trajectoryComposite->Duration() > 0 && !isTargetReached()) {//trajectoryComposite->Duration() > 0 && /*!isTargetReached()*/(elapsedTimeInSec + timeOffset) < trajectoryComposite->Duration()) {
        targetReached(translationFlag, rotationFlag);

        actualTime = elapsedTimeInSec + timeOffset;

        KDL::Frame desiredPose = trajectoryComposite->Pos(actualTime);
        KDL::Twist desiredTwist = trajectoryComposite->Vel(actualTime);
        KDL::Frame initPose = trajectoryComposite->Pos(0);

        double r, p, y;
        desiredPose.M.GetRPY(r, p, y);
       
        Odometry desiredOdometryGlobal(Pose2D(desiredPose.p.x(),
                desiredPose.p.y(),
                actualOdometry.getPose2D().getTheta()),
                Twist2D(desiredTwist.vel.x(),
                desiredTwist.vel.y(),
                desiredTwist.rot.z()));
        
        

        Odometry desiredOdometryLocal;

        Odometry actualOdometryGlobal(actualOdometry);
        Odometry actualOdometryLocal;

        OmniDriveKinematicsModel omnidrive;
        omnidrive.convertToLocalReferenceFrame(desiredOdometryGlobal, desiredOdometryLocal);
        omnidrive.convertToLocalReferenceFrame(actualOdometryGlobal, actualOdometryLocal);

        double dPosX = desiredOdometryLocal.getPose2D().getX();
        double dPosY = desiredOdometryLocal.getPose2D().getY();
        double dPosTheta = y;//desiredOdometryLocal.getPose2D().getTheta();

        double dVelX = desiredOdometryLocal.getTwist2D().getX();
        double dVelY = desiredOdometryLocal.getTwist2D().getY();
        double dVelTheta = desiredOdometryLocal.getTwist2D().getTheta();

        double aPosX = actualOdometryLocal.getPose2D().getX();
        double aPosY = actualOdometryLocal.getPose2D().getY();
        double aPosTheta = actualOdometryLocal.getPose2D().getTheta();

        //double aVelX = actualOdometryLocal.getTwist2D().getX();
        //double aVelY = actualOdometryLocal.getTwist2D().getY();
        //double aVelTheta = actualOdometryLocal.getTwist2D().getTheta();

        double positionXError = dPosX - aPosX;
        double positionYError = dPosY - aPosY;
        double positionThetaError = getShortestAngle(dPosTheta, aPosTheta);

        double gainPosX = gains.getPose2D().getX();
        double gainPosY = gains.getPose2D().getY();
        double gainPosTheta = gains.getPose2D().getTheta();

        //double velocityXError = dVelX - aVelX;
        //double velocityYError = dVelY - aVelY;
        //double velocityThetaError = dVelTheta - aVelTheta;
        double gainVelX = gains.getTwist2D().getX();
        double gainVelY = gains.getTwist2D().getY();
        double gainVelTheta = gains.getTwist2D().getTheta();

        double errorTheta = gainVelTheta * dVelTheta + gainPosTheta * positionThetaError;
        double errorX = gainVelX * dVelX + gainPosX * positionXError;
        double errorY = gainVelY * dVelY + gainPosY * positionYError;

        
     
        
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
