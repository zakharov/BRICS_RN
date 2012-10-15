/* 
 * File:   DifferentialDriveTrajectoryGenerator.h
 * Author: alexey
 *
 * Created on October 9, 2012, 2:53 PM
 */

#ifndef DIFFERENTIALDRIVETRAJECTORYGENERATOR_H
#define	DIFFERENTIALDRIVETRAJECTORYGENERATOR_H

#include "TrajectoryGenerator.h"

class DifferentialDriveTrajectoryGenerator : public TrajectoryGenerator {
public:
    DifferentialDriveTrajectoryGenerator();
    DifferentialDriveTrajectoryGenerator(const DifferentialDriveTrajectoryGenerator& orig);
    virtual ~DifferentialDriveTrajectoryGenerator();
    
    void computeTrajectroy(const KDL::Path_Composite& path, const KDL::Trajectory_Composite& trajectory);
    
private:

};

#endif	/* DIFFERENTIALDRIVETRAJECTORYGENERATOR_H */

