/* 
 * File:   KinematicsModel.h
 * Author: alexey
 *
 * Created on August 21, 2012, 12:26 PM
 */

#ifndef KINEMATICSMODEL_H
#define	KINEMATICSMODEL_H

class Odometry;

class KinematicsModel {
public:
    virtual void convertToLocalReferenceFrame(const Odometry& globalFrame, Odometry& localFrame) = 0;
    virtual void convertToGlobalReferenceFrame(const Odometry& localFrame, Odometry& globalFrame) = 0;
private:

};

#endif	/* KINEMATICSMODEL_H */

