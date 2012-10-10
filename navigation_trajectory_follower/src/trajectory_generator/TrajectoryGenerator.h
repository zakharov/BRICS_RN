/* 
 * File:   TrajectoryGenerator.h
 * Author: alexey
 *
 * Created on October 9, 2012, 2:52 PM
 */

#ifndef TRAJECTORYGENERATOR_H
#define	TRAJECTORYGENERATOR_H

#include <vector>

class FrameWithId;
class TwistWithId;

class TrajectoryGenerator {
public:
      
    virtual void computeTrajectroy(const std::vector<FrameWithId>& path, std::vector<FrameWithId>& trajectory) = 0;
    
private:

};

#endif	/* TRAJECTORYGENERATOR_H */

