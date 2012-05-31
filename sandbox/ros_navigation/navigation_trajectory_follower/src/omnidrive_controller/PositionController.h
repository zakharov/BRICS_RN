/* 
 * File:   PositionController.h
 * Author: alexey
 *
 * Created on May 26, 2012, 9:51 AM
 */

#ifndef POSITIONCONTROLLER_H
#define	POSITIONCONTROLLER_H

class Odometry;

class PositionController {
public:
    
    virtual void setTargetOdometry(const Odometry& targetOdometry) = 0;
    virtual const Odometry& getTargetOdometry() = 0;
    virtual bool isTargetOdometryReached() = 0;
    virtual void setTolerance(const Odometry& tolerance) = 0;
    virtual const Odometry& getTolerance() = 0;
    virtual const Odometry& computeNewOdometry(const Odometry& actualOdometry) = 0;

};

#endif	/* POSITIONCONTROLLER_H */

