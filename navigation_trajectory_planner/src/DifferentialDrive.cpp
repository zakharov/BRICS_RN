/* 
 * File:   DifferentialDrive.cpp
 * Author: alexey
 * 
 * Created on September 6, 2012, 2:08 PM
 */

#include "navigation_trajectory_planner/DifferentialDrive.h"
#include "navigation_trajectory_common/FrameWithId.h"
#include "navigation_trajectory_planner/Stopwatch.h"
#include "navigation_trajectory_planner/Logger.h"

DifferentialDrive::DifferentialDrive() {
}

DifferentialDrive::DifferentialDrive(const DifferentialDrive& orig) {
}

DifferentialDrive::~DifferentialDrive() {
}

void DifferentialDrive::calculateOrientation(const std::vector <FrameWithId>& position, 
            const FrameWithId& startPose, 
            const FrameWithId& goalPose,
            std::vector <FrameWithId>& pose) {
#ifdef DEBUG
    Stopwatch stopwatch;
    stopwatch.start();
#endif

#ifdef DEBUG  
    stopwatch.stop();
    LOG("Rotational Interpolation around single axis:");
    LOG("  - input array: %lu", position.size());
    FrameWithId s(startPose);
    FrameWithId g(goalPose);
    double r, p, y1, y2;
    s.M.GetRPY(r,p,y1);
    g.M.GetRPY(r,p,y2);
    LOG("  - start orientation: %f", y1);
    LOG("  - goal orientation:  %f", y2);
//    LOG("  - step size: %f", step);
//    LOG("  - iterations : %lu", result);
//    LOG("  - duration : %f ms", stopwatch.getElapsedTime());
#endif  
    
    
    
}

void DifferentialDrive::diffDrive(const std::vector <FrameWithId>& position, 
            const FrameWithId& startPose, 
            const FrameWithId& goalPose,
            std::vector <FrameWithId>& pose) {
    
 /*   for (size_t i = 0; i < position.size()-1; ++i) {
        KDL::Vector v1 = position[i].p;
        KDL::Vector v2 = position[i + 1].p;
        double cos_angle = KDL::dot(v1,v2);
        
        if (dot > DOT_THRESHOLD) {
                // If the inputs are too close for comfort, linearly interpolate
                // and normalize the result.

                Quaternion result = v0 + t*(v1 � v0);
                result.normalize();
                return result;
        }
    }
        
    }
  * */
    
}

void DifferentialDrive::slerp(const FrameWithId& v0, 
        const FrameWithId& v1, 
        double t, 
        FrameWithId& result) {
    
  //  double dot = v0 * v1
    
    
}

