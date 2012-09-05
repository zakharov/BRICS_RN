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

    size_t interpolate(const std::vector <FrameWithId>& in,
            std::vector <FrameWithId>& out,
            double step,
            size_t numberOfSteps);

private:

    size_t linearInterpolation(const std::vector <FrameWithId>& in,
            std::vector <FrameWithId>& out,
            double step,
            size_t numberOfsteps);

};

#endif	/* LINEARINTERPOLATION_H */

