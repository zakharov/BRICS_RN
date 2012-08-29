/* 
 * File:   TrajectoryAdapterObserverDummy.h
 * Author: alexey
 *
 * Created on August 29, 2012, 2:43 PM
 */

#ifndef TRAJECTORYADAPTEROBSERVERDUMMY_H
#define	TRAJECTORYADAPTEROBSERVERDUMMY_H

#include "navigation_trajectory_adapter/TrajectoryAdapterObserver.h"

namespace KDL {
    class Trajectory_Composite;
}

class TrajectoryAdapterObserverDummy : public TrajectoryAdapterObserver {
public:
    void trajectoryCallback (KDL::Trajectory_Composite& trajectory);
};

#endif	/* TRAJECTORYADAPTEROBSERVERDUMMY_H */

