/* 
 * File:   PathPlanner.h
 * Author: alexey
 *
 * Created on August 30, 2012, 5:10 PM
 */

#ifndef PATHPLANNER_H
#define	PATHPLANNER_H

#include <vector>

class FrameWithId;

class PathPlanner {
public:
    virtual bool computePath(const FrameWithId& start, const FrameWithId& goal, std::vector <FrameWithId>& path) = 0;
private:

};

#endif	/* PATHPLANNER_H */

