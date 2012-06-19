
#ifndef CONVERSIONUTILS_H
#define	CONVERSIONUTILS_H

#include "Pose2D.h"
#include "Twist2D.h"
#include "kdl/frames.hpp"

void pose2dToFrameKdl(const Pose2D& pose2d, KDL::Frame& pose) {
    pose = KDL::Frame(KDL::Rotation::RPY(0, 0, pose2d.getTheta()), KDL::Vector(pose2d.getX(), pose2d.getY(), 0));
}

void twist2dToTwistKdl(const Twist2D& twist2d, KDL::Twist& twist) {
    twist = KDL::Twist(KDL::Vector(twist2d.getX(), twist2d.getY(), 0), KDL::Vector(0, 0, twist2d.getTheta()));
}


#endif	/* CONVERSIONUTILS_H */

