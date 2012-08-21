/* 
 * File:   OmniDriveKinematicsModel.cpp
 * Author: alexey
 * 
 * Created on August 21, 2012, 12:40 PM
 */

#include "OmniDriveKinematicsModel.h"
#include "Pose2D.h"
#include "Odometry.h"
#include <cmath>

OmniDriveKinematicsModel::OmniDriveKinematicsModel() {
}

OmniDriveKinematicsModel::OmniDriveKinematicsModel(const OmniDriveKinematicsModel& orig) {
}

OmniDriveKinematicsModel::~OmniDriveKinematicsModel() {
}

void OmniDriveKinematicsModel::convert(const double& x1, const double& y1, const double& theta, double& x2, double& y2, bool inverse) {
    
    double k = (inverse == true) ? 1:-1;
    
    x2 = (x1 * cos(theta) + y1 * sin(theta)) * k;
    y2 = (y1 * cos(theta) - x1 * sin(theta)) * k; 
}

void OmniDriveKinematicsModel::convertToLocalReferenceFrame(const Odometry& globalFrame, Odometry& localFrame) {
    
    double locPosX = 0;
    double locPosY = 0;
    
    convert(globalFrame.getPose2D().getX(), 
            globalFrame.getPose2D().getY(), 
            globalFrame.getPose2D().getTheta(), 
            locPosX, 
            locPosY, 
            true);
    
    double locVelX = 0;
    double locVelY = 0;
    
    convert(globalFrame.getTwist2D().getX(), 
            globalFrame.getTwist2D().getY(), 
            globalFrame.getPose2D().getTheta(), 
            locVelX, 
            locVelY, 
            true);
    
    localFrame = Odometry(Pose2D(locPosX, locPosY, globalFrame.getPose2D().getTheta()), 
            Twist2D(locVelX, locVelY, globalFrame.getTwist2D().getTheta()));
}

void OmniDriveKinematicsModel::convertToGlobalReferenceFrame(const Odometry& localFrame, Odometry& globalFrame) {
       
    double globPosX = 0;
    double globPosY = 0;
    
    convert(localFrame.getPose2D().getX(), 
            localFrame.getPose2D().getY(), 
            localFrame.getPose2D().getTheta(), 
            globPosX, 
            globPosY, 
            false);
    
    double globVelX = 0;
    double globVelY = 0;
    
    convert(localFrame.getTwist2D().getX(), 
            localFrame.getTwist2D().getY(), 
            localFrame.getPose2D().getTheta(), 
            globVelX, 
            globVelY, 
            false);
    
    globalFrame = Odometry(Pose2D(globPosX, globPosY, localFrame.getPose2D().getTheta()), 
            Twist2D(globVelX, globVelY, localFrame.getTwist2D().getTheta()));
}