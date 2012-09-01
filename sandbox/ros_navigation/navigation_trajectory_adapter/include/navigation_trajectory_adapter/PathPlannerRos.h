/* 
 * File:   PathPlannerRos.h
 * Author: alexey
 *
 * Created on August 30, 2012, 5:17 PM
 */

#ifndef PATHPLANNERROS_H
#define	PATHPLANNERROS_H

#include "navigation_trajectory_adapter/PathPlanner.h"
#include <string>

namespace tf {
class TransformListener;
}

namespace costmap_2d {
class Costmap2DROS;    
}

namespace nav_core {
class BaseGlobalPlanner;    
}

class PathPlannerRos : public PathPlanner {
public:
    PathPlannerRos(nav_core::BaseGlobalPlanner* planner);
    
    PathPlannerRos(const PathPlannerRos& orig);
    
    virtual ~PathPlannerRos();
    virtual bool computePath(const FrameWithId& start, const FrameWithId& goal, std::vector <FrameWithId>& path);
    
private:
    std::string name; 
    nav_core::BaseGlobalPlanner* planner;
};

#endif	/* PATHPLANNERROS_H */

