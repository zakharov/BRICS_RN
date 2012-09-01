/* 
 * File:   CollisionChecking.h
 * Author: alexey
 *
 * Created on September 1, 2012, 2:13 PM
 */

#ifndef COLLISIONCHECKING_H
#define	COLLISIONCHECKING_H

#include <vector>

class FrameWithId;

class CollisionChecking {
public:
    virtual bool check(const std::vector <FrameWithId>& path, const FrameWithId& actualPose) = 0;
};

#endif	/* COLLISIONCHECKING_H */

