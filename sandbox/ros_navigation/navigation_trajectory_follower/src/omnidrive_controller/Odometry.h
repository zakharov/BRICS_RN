/* 
 * File:   Odometry.h
 * Author: alexey
 *
 * Created on May 26, 2012, 9:56 AM
 */

#ifndef ODOMETRY_H
#define	ODOMETRY_H

#include "Twist2D.h"
#include "Pose2D.h"

class Odometry {
public:
    Odometry(); // creates odometry with 0,0,0 pose and 0,0,0 twist
    Odometry(const Pose2D& pose, const Twist2D& twist);
    Odometry(const Pose2D& pose);
    Odometry(const Twist2D& twist);
    Odometry(const Odometry& orig);
  
    const Odometry& operator=(const Odometry& orig);
    
    const Pose2D& getPose2D() const;
    void setPose2D(const Pose2D& twist);
    
    const Twist2D& getTwist2D() const;
    void setTwist2D(const Twist2D& twist);
    
    virtual ~Odometry();
    
private:
    Pose2D pose;
    Twist2D twist;
};

#endif	/* ODOMETRY_H */

