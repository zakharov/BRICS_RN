/* 
 * File:   OmniDriveKinematicsModel.h
 * Author: alexey
 *
 * Created on August 21, 2012, 12:40 PM
 */

#ifndef OMNIDRIVEKINEMATICSMODEL_H
#define	OMNIDRIVEKINEMATICSMODEL_H

#include "KinematicsModel.h"

class Odometry;

class OmniDriveKinematicsModel : public KinematicsModel {
public:
    OmniDriveKinematicsModel();
    OmniDriveKinematicsModel(const OmniDriveKinematicsModel& orig);
    virtual ~OmniDriveKinematicsModel();
    
    void convertToLocalReferenceFrame(const Odometry& globalFrame, Odometry& localFrame);
    void convertToGlobalReferenceFrame(const Odometry& localFrame, Odometry& globalFrame);
    
private:
    
    void convert(const double& x1, const double& y1, const double& theta, double& x2, double& y2, bool inverse);

};

#endif	/* OMNIDRIVEKINEMATICSMODEL_H */

