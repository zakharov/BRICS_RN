/* 
 * File:   PathInterpolation.h
 * Author: alexey
 *
 * Created on September 3, 2012, 11:49 AM
 */

#ifndef PATHINTERPOLATION_H
#define	PATHINTERPOLATION_H

#include <vector>

class FrameWithId;

class PathInterpolation {
    virtual void interpolate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, double step) = 0;
};



#endif	/* PATHINTERPOLATION_H */

