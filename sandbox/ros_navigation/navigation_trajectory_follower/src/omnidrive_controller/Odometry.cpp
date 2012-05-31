/* 
 * File:   Odometry.cpp
 * Author: alexey
 * 
 * Created on May 26, 2012, 9:56 AM
 */

#include "Odometry.h"

Odometry::Odometry() : pose(Pose2D()), twist(Twist2D()) {

}

Odometry::Odometry(const Pose2D& pose, const Twist2D& twist) : pose(pose), twist(twist) {
    
}

Odometry::Odometry(const Pose2D& pose) : pose(pose), twist(Twist2D()) {
    
}

Odometry::Odometry(const Twist2D& twist) : pose(Pose2D()), twist(twist)  {
    
}

Odometry::Odometry(const Odometry& orig) : pose(orig.getPose2D()), twist(orig.getTwist2D()) {
       
}

const Odometry& Odometry::operator= (const Odometry& orig) {
    this->pose = orig.getPose2D();
    this->twist = orig.getTwist2D();
    return *this;
}

Odometry::~Odometry() {
    
}

const Pose2D& Odometry::getPose2D() const {
    return pose;
}

 void  Odometry::setPose2D(const Pose2D& pose) {
     this->pose = pose;
 }

const Twist2D& Odometry::getTwist2D() const {
    return twist;
}

void  Odometry::setTwist2D(const Twist2D& twist) {
     this->twist = twist;
 }
   