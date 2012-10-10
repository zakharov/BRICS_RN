/* 
 * File:   OmniDriveTrajectoryGenerator.cpp
 * Author: alexey
 * 
 * Created on October 9, 2012, 2:53 PM
 */

#include "OmniDriveTrajectoryGenerator.h"

#include <navigation_trajectory_common/FrameWithId.h>
#include <navigation_trajectory_common/TwistWithId.h>

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

void OmniDriveTrajectoryGenerator::computeTrajectroy(const std::vector<FrameWithId>& path, KDL::Trajectory_Composite& trajectory) {
    std::vector<FrameWithId> pathWithRotation;
    interpolateRotation(path, pathWithRotation);
    computeTrajectoryComposite(pathWithRotation, trajectory);
}


void OmniDriveTrajectoryGenerator::interpolateRotation(const std::vector<FrameWithId>& path, std::vector<FrameWithId>& pathWithRotation) {
       
    if (path.empty())
        return;
    
    pathWithRotation.clear();
    
    double r,p,y;
    path.front().M.GetRPY(r,p,y);
    double start = y;
    
    path.back().M.GetRPY(r,p,y);
    double end = y;
    double step = getShortestAngle(end, start) / path.size();

    std::vector<FrameWithId>::const_iterator it;
    for (it = path.begin(); it != path.end() - 1; ++it) {
        
        FrameWithId f;
        f.id = it->id;
        f.p = it->p;
        f.M = KDL::Rotation::RPY(0,0,start);
        pathWithRotation.push_back(f);
       
        start = start + step;
     
        
       // it->pose.orientation = f1.orientation;
    }

    pathWithRotation.front() = path.front();
    pathWithRotation.back() = path.back();
}

void OmniDriveTrajectoryGenerator::computeTrajectoryComposite(const std::vector<FrameWithId>& path, KDL::Trajectory_Composite& trajectory) {

    const double maxVel = 0.3;
    const double maxAcc = 0.05;

  //  ROS_INFO("Path converted to a trajectory, max velocity: %f m/s, max acceleration: %f m/s^2", maxVel, maxAcc);

    KDL::VelocityProfile_Trap* velocityProfile = new KDL::VelocityProfile_Trap(maxVel, maxAcc);
    KDL::Path* copyPath = const_cast<KDL::Path&> (path).Clone(); // Why KDL has no const version of Clone method?
    // They force me to do this!

    velocityProfile->SetProfile(0, copyPath->PathLength());
    KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(copyPath, velocityProfile);

    trajectory.Add(trajectorySegment);

/*    double dt = 1.0;
    for (double i = 0; i <= trajectory.Duration() + dt; i = i + dt) {
        cout << "pos" << ":" << trajectory.Pos(i).p << endl;
        cout << "vel" << ":" << trajectory.Vel(i).vel << endl;
        cout << "acc" << ":" << trajectory.Acc(i).vel << endl;
        cout << "---" << endl;
    }*/
    
    
    return true;
}