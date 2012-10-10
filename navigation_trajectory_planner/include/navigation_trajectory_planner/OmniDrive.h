/* 
 * File:   OmniDrive.h
 * Author: alexey
 *
 * Created on September 6, 2012, 2:09 PM
 */

#ifndef OMNIDRIVE_H
#define	OMNIDRIVE_H

//#include "navigation_trajectory_planner/PlatformKinematics.h"
#include "navigation_trajectory_common/FrameWithId.h"
#include <vector>

class OmniDrive /*: public PlatformKinematics*/ {
public:
    OmniDrive();
    OmniDrive(const OmniDrive& orig);
    virtual ~OmniDrive();
    
    void calculateOrientation(const std::vector <FrameWithId>& position, 
            const FrameWithId& startPose, 
            const FrameWithId& goalPose,
            std::vector <FrameWithId>& pose);
    
private:

};

#endif	/* OMNIDRIVE_H */

