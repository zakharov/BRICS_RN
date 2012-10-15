/* 
 * File:   TrajectoryGenerator.h
 * Author: alexey
 *
 * Created on October 9, 2012, 2:52 PM
 */

#ifndef TRAJECTORYGENERATOR_H
#define	TRAJECTORYGENERATOR_H

#include <vector>

namespace KDL {
    class Path_Composite;
    class Trajectory_Composite;
}

class TrajectoryGenerator {
public:
      
    virtual void computeTrajectroy(const KDL::Path_Composite& path, KDL::Trajectory_Composite& trajectory) = 0;
    
private:

};

#endif	/* TRAJECTORYGENERATOR_H */

