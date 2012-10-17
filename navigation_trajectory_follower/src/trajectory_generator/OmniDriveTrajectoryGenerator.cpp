/* 
 * File:   OmniDriveTrajectoryGenerator.cpp
 * Author: alexey
 * 
 * Created on October 9, 2012, 2:53 PM
 */

#include "OmniDriveTrajectoryGenerator.h"

#include <navigation_trajectory_common/FrameWithId.h>
#include <navigation_trajectory_common/TwistWithId.h>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/path.hpp>
#include <iostream>

#include <cmath>

OmniDriveTrajectoryGenerator::OmniDriveTrajectoryGenerator() {
}

OmniDriveTrajectoryGenerator::OmniDriveTrajectoryGenerator(const OmniDriveTrajectoryGenerator& orig) {
}

OmniDriveTrajectoryGenerator::~OmniDriveTrajectoryGenerator() {
}

float OmniDriveTrajectoryGenerator::getShortestAngle(float goalAngle, float actualAngle) {
    return atan2(sin(goalAngle - actualAngle), cos(goalAngle - actualAngle));
}

void OmniDriveTrajectoryGenerator::computePathComposite(const std::vector<FrameWithId>& path, KDL::Path_Composite& pathComposite) {
    std::vector<FrameWithId>::const_iterator it;

    if (path.size() > 1) {

        FrameWithId p1 = path.front();
        FrameWithId p2;

        for (it = path.begin() + 1; it != path.end(); ++it) {
            p2 = *it;
            KDL::Frame f1, f2;
            f1 = p1.getFrame();
            f2 = p2.getFrame();

            KDL::RotationalInterpolation_SingleAxis* rot = new KDL::RotationalInterpolation_SingleAxis();
         //   rot->Vel(0.1,0.01);
         //   rot->Acc(0.1,0.01,0.001);
            KDL::Path_Line* pathLine = new KDL::Path_Line(f1, f2, rot, 0.001);
            pathComposite.Add(pathLine);

            p1 = p2;
        }
        
    }
}

void OmniDriveTrajectoryGenerator::interpolateRotation(const std::vector<FrameWithId>& path, std::vector<FrameWithId>& pathWithRotation) {

    if (path.size() <= 1)
        return;

    pathWithRotation.clear();

    double r, p, y;
    path.front().M.GetRPY(r, p, y);
    double start = y;

    path.back().M.GetRPY(r, p, y);
    double end = y;
    double step = getShortestAngle(end, start) / (path.size()-1);
    

    std::vector<FrameWithId>::const_iterator it;
    for (it = path.begin(); it != path.end(); ++it) {

        FrameWithId f;
        f.id = it->id;
        f.p = it->p;
        f.M = KDL::Rotation::RPY(0, 0, start);
        
        pathWithRotation.push_back(f);

        start = start + step;
    }

   // pathWithRotation.front() = path.front();
   // pathWithRotation.back() = path.back();
}

void OmniDriveTrajectoryGenerator::computeTrajectroy(const KDL::Path_Composite& path, KDL::Trajectory_Composite& trajectory) {

    const double maxVel = 1.0;
    const double maxAcc = 0.1;

    KDL::VelocityProfile_Trap* velocityProfile = new KDL::VelocityProfile_Trap(maxVel, maxAcc);
    KDL::Path* copyPath = const_cast<KDL::Path_Composite&> (path).Clone(); // Why KDL has no const version of Clone method?
    // They force me to do this!

    velocityProfile->SetProfile(0, copyPath->PathLength());

    KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(copyPath, velocityProfile);
    trajectory.Add(trajectorySegment);

}

void OmniDriveTrajectoryGenerator::computeTrajectroy(const std::vector<FrameWithId> path, KDL::Trajectory_Composite& trajectory) {
    std::vector<FrameWithId> pathWithRotation;
    interpolateRotation(path, pathWithRotation);
    
    double r,p,y;
    
       
    
    KDL::Path_Composite pathComposite;
    computePathComposite(pathWithRotation, pathComposite);
    computeTrajectroy(pathComposite, trajectory);
}