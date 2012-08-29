/* 
 * File:   TrajectoryAdapterObserver.h
 * Author: alexey
 *
 * Created on August 29, 2012, 2:30 PM
 */

#ifndef TRAJECTORYADAPTEROBSERVER_H
#define	TRAJECTORYADAPTEROBSERVER_H

namespace KDL {
    class Trajectory_Composite;
}

class TrajectoryAdapterObserver {
public:
    virtual void trajectoryCallback (KDL::Trajectory_Composite& trajectory) = 0;
};

#endif	/* TRAJECTORYADAPTEROBSERVER_H */

