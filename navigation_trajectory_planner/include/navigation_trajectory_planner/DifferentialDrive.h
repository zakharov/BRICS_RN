/* 
 * File:   DifferentialDrive.h
 * Author: alexey
 *
 * Created on September 6, 2012, 2:08 PM
 */

#ifndef DIFFERENTIALDRIVE_H
#define	DIFFERENTIALDRIVE_H

//#include "navigation_trajectory_planner/PlatformKinematics.h"
#include <vector>

class FrameWithId;

class DifferentialDrive /*: public PlatformKinematics*/ {
public:
    DifferentialDrive();
    DifferentialDrive(const DifferentialDrive& orig);
    virtual ~DifferentialDrive();
    
    void calculateOrientation(const std::vector <FrameWithId>& position, 
            const FrameWithId& startPose, 
            const FrameWithId& goalPose,
            std::vector <FrameWithId>& pose);
    
private:
    
    void diffDrive(const std::vector <FrameWithId>& position, 
            const FrameWithId& startPose, 
            const FrameWithId& goalPose,
            std::vector <FrameWithId>& pose);
    
    void slerp(const FrameWithId& v0, 
        const FrameWithId& v1, 
        double t, 
        FrameWithId& result);
   
  
};

#endif	/* DIFFERENTIALDRIVE_H */

