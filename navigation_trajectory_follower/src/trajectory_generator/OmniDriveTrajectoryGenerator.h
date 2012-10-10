/* 
 * File:   OmniDriveTrajectoryGenerator.h
 * Author: alexey
 *
 * Created on October 9, 2012, 2:53 PM
 */

#ifndef OMNIDRIVETRAJECTORYGENERATOR_H
#define	OMNIDRIVETRAJECTORYGENERATOR_H

#include "TrajectoryGenerator.h"

namespace KDL {
    class Trajectory_Composite;
}

class OmniDriveTrajectoryGenerator : public TrajectoryGenerator {
public:
    OmniDriveTrajectoryGenerator();
    OmniDriveTrajectoryGenerator(const OmniDriveTrajectoryGenerator& orig);
    virtual ~OmniDriveTrajectoryGenerator();
    
    void computeTrajectroy(const std::vector<FrameWithId>& path, KDL::Trajectory_Composite& trajectory);
    float getShortestAngle(float goalAngle, float actualAngle);
    
    void interpolateRotation(const std::vector<FrameWithId>& path, std::vector<FrameWithId>& pathWithRotation);
    void computeTrajectoryComposite(const std::vector<FrameWithId>& path, KDL::Trajectory_Composite& trajectory);
    
private:

};

#endif	/* OMNIDRIVETRAJECTORYGENERATOR_H */

