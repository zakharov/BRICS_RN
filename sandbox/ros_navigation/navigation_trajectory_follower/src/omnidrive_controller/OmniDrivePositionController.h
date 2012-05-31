/* 
 * File:   OmnidrivePositionController.h
 * Author: alexey
 *
 * Created on May 26, 2012, 10:27 AM
 */

#ifndef OMNIDRIVEPOSITIONCONTROLLER_H
#define	OMNIDRIVEPOSITIONCONTROLLER_H

#include "PositionController.h"
#include "VelocityRamp.h"
#include "Odometry.h"

class OmniDrivePositionController : public PositionController  {
public:
    OmniDrivePositionController(const VelocityRamp& linearVelocityRamp, const VelocityRamp& angularVelocitRamp,  const Odometry& tolerance);
    OmniDrivePositionController(const OmniDrivePositionController& orig);
    virtual ~OmniDrivePositionController();
    
    void setTolerance(const Odometry& tolerance);
    const Odometry& getTolerance();
    
    void setTargetOdometry(const Odometry& targetOdometry);
    const Odometry& getTargetOdometry();
    const Odometry& computeNewOdometry(const Odometry& actualOdometry);
    bool isTargetOdometryReached();
    
private:    
    
    Twist2D computeLinearVelocity(const Pose2D& actualPose, const Pose2D& initialPose, const Pose2D& goalPose);
    Twist2D computeAngularVelocity(const Pose2D& actualPose, const Pose2D& initialPose, const Pose2D& goalPose);
    float getDistance(const Pose2D& actualPose, const Pose2D& goalPose);
    float getShortestAngle (float goalAngle, float actualAngle);
    bool  gotNewTarget();
    
private:
    Odometry tolerance;
    Odometry targetOdometry;
    Odometry computedOdometry;
    Odometry initialOdometry;
    VelocityRamp linearVelocityRamp;
    VelocityRamp angularVelocityRamp;
    bool newTargetFlag;
    bool linearOdometryReached;
    bool angularOdometryReached;
};

#endif	/* OMNIDRIVEPOSITIONCONTROLLER_H */

