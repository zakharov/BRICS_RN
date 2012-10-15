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
    class Path_Composite;
}

class FrameWithId;

class OmniDriveTrajectoryGenerator : public TrajectoryGenerator {
public:
    OmniDriveTrajectoryGenerator();
    OmniDriveTrajectoryGenerator(const OmniDriveTrajectoryGenerator& orig);
    virtual ~OmniDriveTrajectoryGenerator();
    
   
    float getShortestAngle(float goalAngle, float actualAngle);
    void computePathComposite(const std::vector<FrameWithId>& path, KDL::Path_Composite& pathComposite);
    
    void interpolateRotation(const std::vector<FrameWithId>& path, std::vector<FrameWithId>& pathWithRotation);
    void computeTrajectroy(const KDL::Path_Composite& path, KDL::Trajectory_Composite& trajectory); 
    void computeTrajectroy(const std::vector<FrameWithId> path, KDL::Trajectory_Composite& trajectory);
    
private:

};

#endif	/* OMNIDRIVETRAJECTORYGENERATOR_H */

