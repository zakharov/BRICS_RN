/* 
 * File:   TrajectoryPlannerNode.h
 * Author: alexey
 *
 * Created on May 30, 2012, 4:21 PM
 */

#ifndef TRAJECTORYADAPTER_H
#define	TRAJECTORYADAPTER_H

#include <ros/ros.h>
#include <navigation_trajectory_adapter/FrameWithId.h>
#include <navigation_trajectory_adapter/TwistWithId.h>

class TrajectoryAdapterObserver;

class TrajectoryAdapter {
    
    struct Odometry {
        FrameWithId pose;
        TwistWithId twist;
        bool updated;
        Odometry () : pose(FrameWithId()), twist(TwistWithId()), updated(false) {};
    };
    
public: 
 
    TrajectoryAdapter();
    TrajectoryAdapter(TrajectoryAdapterObserver* observer);
    virtual ~TrajectoryAdapter();
    
    void planPath(const FrameWithId& goal, std::vector<FrameWithId>& path);
    void simplifyPath(const std::vector<FrameWithId>& path, std::vector<FrameWithId>& simplifiedPath);
    void calculateVelocityProfile(const std::vector<FrameWithId>& path, std::vector<TwistWithId>& velocityProfile);
    void updateOdometry(const FrameWithId& pose, const TwistWithId& twist);
    
private:
    const TrajectoryAdapterObserver* observer;
    Odometry actualOdometry;
};

#endif	/* TRAJECTORYADAPTER_H */

