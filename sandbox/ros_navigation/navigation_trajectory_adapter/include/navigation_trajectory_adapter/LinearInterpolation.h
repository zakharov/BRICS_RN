/* 
 * File:   LinearInterpolation.h
 * Author: alexey
 *
 * Created on September 3, 2012, 11:51 AM
 */

#ifndef LINEARINTERPOLATION_H
#define	LINEARINTERPOLATION_H

#include "navigation_trajectory_adapter/PathInterpolation.h"

class LinearInterpolation : PathInterpolation {
public:
    LinearInterpolation();
    LinearInterpolation(const LinearInterpolation& orig);
    virtual ~LinearInterpolation();
    
    void interpolate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, double step);
    
private:

};

#endif	/* LINEARINTERPOLATION_H */

