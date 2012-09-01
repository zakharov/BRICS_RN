/* 
 * File:   PathApproximator.h
 * Author: alexey
 *
 * Created on August 31, 2012, 2:52 PM
 */

#ifndef PATHAPPROXIMATION_H
#define	PATHAPPROXIMATION_H

#include <vector>

class FrameWithId;

class PathApproximation {
    virtual void approximate (const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out) = 0;
};


#endif	/* PATHAPPROXIMATOR_H */

