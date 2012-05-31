/*
 * File:   OmnidrivePositionController.cpp
 * Author: alexey
 *
 * Created on May 26, 2012, 10:27 AM
 */

#include "OmniDrivePositionController.h"
#include <cmath>
//#define DEBUG

#ifdef DEBUG
#include <iostream>
#endif

OmniDrivePositionController:: OmniDrivePositionController(const VelocityRamp& linearVelocityRamp, const VelocityRamp& angularVelocitRamp, const Odometry& tolerance) {
    this->tolerance = tolerance;
    this->linearVelocityRamp = linearVelocityRamp;
    this->angularVelocityRamp = angularVelocitRamp;
    newTargetFlag = false;
}

OmniDrivePositionController:: OmniDrivePositionController(const OmniDrivePositionController& orig) {
    this->targetOdometry = orig.targetOdometry;
    this->computedOdometry = orig.targetOdometry;
    this->linearVelocityRamp = orig.linearVelocityRamp;
    this->angularVelocityRamp = orig.angularVelocityRamp;
}

OmniDrivePositionController::~ OmniDrivePositionController() {
}

void OmniDrivePositionController::setTargetOdometry(const Odometry& targetOdometry) {
    this->targetOdometry = targetOdometry;
    angularOdometryReached = false;
    linearOdometryReached = false;
    newTargetFlag = true;
}

const Odometry&  OmniDrivePositionController::getTargetOdometry() {
    return targetOdometry;
}

bool OmniDrivePositionController::isTargetOdometryReached() {
    return linearOdometryReached && angularOdometryReached;
}

const Odometry&  OmniDrivePositionController::computeNewOdometry(const Odometry& actualOdometry) {

    if (gotNewTarget() == true) {
        initialOdometry = actualOdometry;
        newTargetFlag = false;
    }

#ifdef DEBUG
    std::cout << "actualOdometry x = " << actualOdometry.getPose2D().getX() << " y = " <<  actualOdometry.getPose2D().getY() << std::endl;
#endif


    Twist2D linear, angular;
    if (getDistance(actualOdometry.getPose2D(), targetOdometry.getPose2D()) > getDistance(tolerance.getPose2D(), Pose2D(0,0,0))) {
        linear = computeLinearVelocity(actualOdometry.getPose2D(),
                this->initialOdometry.getPose2D(),
                this->targetOdometry.getPose2D());
    } else {
        linearOdometryReached = true;
    }

    if (fabs(getShortestAngle(actualOdometry.getPose2D().getTheta(), targetOdometry.getPose2D().getTheta())) > tolerance.getPose2D().getTheta()) {
        angular = computeAngularVelocity(actualOdometry.getPose2D(),
                this->initialOdometry.getPose2D(),
                this->targetOdometry.getPose2D());
    } else {
        angularOdometryReached = true;
    }

    computedOdometry.setPose2D(actualOdometry.getPose2D());
    computedOdometry.setTwist2D(linear + angular);

    return computedOdometry;
}

float OmniDrivePositionController::getShortestAngle (float goalAngle, float actualAngle)
{
    return atan2(sin(goalAngle-actualAngle), cos(goalAngle-actualAngle));
}

float OmniDrivePositionController::getDistance(const Pose2D& actualPose, const Pose2D& goalPose)
{
    return sqrt( (actualPose.getX() - goalPose.getX()) * (actualPose.getX() - goalPose.getX()) +
                 (actualPose.getY() - goalPose.getY()) * (actualPose.getY() - goalPose.getY()) );

}


Twist2D OmniDrivePositionController::computeAngularVelocity(const Pose2D& actualPose, const Pose2D& initialPose, const Pose2D& goalPose)
{

    float actualAngle = actualPose.getTheta();
    float goalAngle = goalPose.getTheta();
    float initialAngle = initialPose.getTheta();
    float acceleration = 0.5; // zero is no acceleration, 1 is a step function

    float totalDistance  = getShortestAngle(goalAngle, initialAngle);
    float actualDistance = totalDistance - getShortestAngle(goalAngle, actualAngle);

    angularVelocityRamp.setTotalDistance(totalDistance);
    angularVelocityRamp.setAcceleration(acceleration);

    float angularVelocity = angularVelocityRamp.computeVelocity(actualDistance);

    return Twist2D(0,0,angularVelocity);
}

Twist2D OmniDrivePositionController::computeLinearVelocity(const Pose2D& actualPose, const Pose2D& initialPose, const Pose2D& goalPose)
{
    float actualAngle = actualPose.getTheta();
    float goalAngle = goalPose.getTheta();

    Twist2D twist;

    float totalDistance = getDistance(initialPose, goalPose);
    float actualDistance = totalDistance - getDistance(actualPose, goalPose);

    float acceleration = 0.5; // zero is no acceleration, 1 is a step function

    linearVelocityRamp.setTotalDistance(totalDistance);
    linearVelocityRamp.setActualDistance(actualDistance);
    linearVelocityRamp.setAcceleration(acceleration);

    float linearVelocity = linearVelocityRamp.computeVelocity(actualDistance);

    float x = (goalPose.getX() * cos(actualAngle) + goalPose.getY() * sin(actualAngle)) -
        (actualPose.getX() * cos(actualAngle) + actualPose.getY() * sin(actualAngle));
    float y = (goalPose.getY() * cos(actualAngle) - goalPose.getX() * sin(actualAngle)) -
        (actualPose.getY() * cos(actualAngle) - actualPose.getX() * sin(actualAngle));

    float absX = fabs(x);
    float absY = fabs(y);

    float norm = (absX < absY) ? absY : absX;

    if (norm != 0.0) {

        x = x / norm * linearVelocity;
        y = y / norm * linearVelocity;
        return Twist2D(x,y,0);
    }

    return Twist2D(0,0,0);
}

bool OmniDrivePositionController::gotNewTarget(){
    return newTargetFlag;
}

void OmniDrivePositionController::setTolerance(const Odometry& tolerance) {
    this->tolerance = tolerance;
}

const Odometry& OmniDrivePositionController::getTolerance() {
    return tolerance;
}
