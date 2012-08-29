/* 
 * File:   TrajectoryPlannerNode.h
 * Author: alexey
 *
 * Created on May 30, 2012, 4:21 PM
 */

#ifndef TRAJECTORYADAPTER_H
#define	TRAJECTORYADAPTER_H

#include <ros/ros.h>

class TrajectoryAdapterObserver;

class TrajectoryAdapter {
public: 
 
    TrajectoryAdapter();
    TrajectoryAdapter(TrajectoryAdapterObserver* observer);
    virtual ~TrajectoryAdapter();
    
    void planPath();
    void simplifyPath();
    void convertPathToTrajectory();
    void updateOdometry();
    
private:
    const TrajectoryAdapterObserver* observer;
};

#endif	/* TRAJECTORYADAPTER_H */

