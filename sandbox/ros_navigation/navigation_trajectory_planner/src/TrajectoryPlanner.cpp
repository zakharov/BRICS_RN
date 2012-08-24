/******************************************************************************
 * Copyright (c) 2011
 * GPS GmbH
 *
 * Author:
 * Alexey Zakharov
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of GPS GmbH nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#include "navigation_trajectory_planner/TrajectoryPlanner.h"
#include "navigation_trajectory_planner/ConversionUtils.h"

#include <kdl/frames.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>

#include <iostream>

using namespace std;

ros::Publisher pathPublisher;
   

TrajectoryPlanner::TrajectoryPlanner(nav_core::BaseGlobalPlanner* pathPlanner, ros::NodeHandle& globalNode) {
    this->pathPlanner = pathPlanner;
    pathPublisher = globalNode.advertise<nav_msgs::Path> ("debugPath", 1);
}

TrajectoryPlanner::TrajectoryPlanner(nav_core::BaseGlobalPlanner* pathPlanner) {
    this->pathPlanner = pathPlanner;
    
}

TrajectoryPlanner::TrajectoryPlanner(const TrajectoryPlanner& orig) {
}

TrajectoryPlanner::~TrajectoryPlanner() {
}

void TrajectoryPlanner::setPathFrameId(std::string frameId) {
    this->frameId = frameId;
}

float getShortestAngle(float goalAngle, float actualAngle) {
    return atan2(sin(goalAngle - actualAngle), cos(goalAngle - actualAngle));
}

double perpendicularDistance(geometry_msgs::PoseStamped point, 
        geometry_msgs::PoseStamped linePoint1, 
        geometry_msgs::PoseStamped linePoint2) {
    
    double x0 = point.pose.position.x;
    double y0 = point.pose.position.y;

    // point.print();
    // linePoint1.print();
    // linePoint2.print();

    double x1 = linePoint1.pose.position.x;
    double y1 = linePoint1.pose.position.y;

    double x2 = linePoint2.pose.position.x;
    double y2 = linePoint2.pose.position.y;

    double a = (y2 - y1) / (x2 - x1);
    double b = y1 - a*x1;



    double dist = fabs((x2 - x1)*(y1 - y0)-(x1 - x0)*(y2 - y1)) /
            sqrt((x2 - x1)*(x2 - x1)+(y2 - y1)*(y2 - y1));
    // cout << "dist = " << dist << endl;

    return dist;
    
   
}

void douglasPeucker(const std::vector<geometry_msgs::PoseStamped>& pointList, 
        std::vector<geometry_msgs::PoseStamped>& resultList, double epsilon) {
 double dmax = 0;
    unsigned int index = 0;

    std::vector<geometry_msgs::PoseStamped> result1;
    std::vector<geometry_msgs::PoseStamped> result2;

    for (unsigned int i = 1; i < pointList.size() - 1; i++) {
        double d = perpendicularDistance(pointList[i], pointList[0],
                pointList[pointList.size() - 1]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }



    //If max distance is greater than epsilon, recursively simplify
    if (dmax >= epsilon) {
        //Recursive call
        std::vector<geometry_msgs::PoseStamped> pointSubList1(&pointList[0],
                &pointList[index]);
        std::vector<geometry_msgs::PoseStamped> pointSubList2(&pointList[index],
                &pointList[pointList.size()]);

        douglasPeucker(pointSubList1, result1, epsilon);
        douglasPeucker(pointSubList2, result2, epsilon);
        resultList.insert(resultList.begin(), result1.begin(), result1.end() - 1);
        resultList.insert(resultList.end(), result2.begin(), result2.end());



    } else {
        resultList.push_back(pointList[0]);
        resultList.push_back(pointList[pointList.size() - 1]);
    }



}


const geometry_msgs::PoseStamped& GetPoint(int i, const std::vector<geometry_msgs::PoseStamped>& Points) {
	// return 1st point
    int NUM_POINTS = Points.size();
    
	if (i<0) {
		return	Points[0];
	}
	// return last point
	if (i<NUM_POINTS)
		return Points[i];

	return Points[NUM_POINTS-1];
}

void cubicBSplineCurve (const std::vector<geometry_msgs::PoseStamped>& pointList, 
        std::vector<geometry_msgs::PoseStamped>& resultList, unsigned int LOD) {
    

 
   
    // in total i am going to draw (NUM_POINTS+1) curves. I will start
        // the curves at the imaginary index -3. Each section of the curve
	// will start on the next vertex index.
	//
    int NUM_SEGMENTS = pointList.size() + 1;

    
    for(int start_cv=-3,j=0;j!=NUM_SEGMENTS;++j,++start_cv) {
        // for each section of curve, draw LOD number of divisions
        for(unsigned int i=0;i!=LOD;++i) {
            // use the parametric time value 0 to 1 for this curve
            // segment.
            double t = (double)i/LOD;
            // the t value inverted
            
            double it = 1.0f-t;
            // calculate blending functions for cubic bspline
            double b0 = it*it*it/6.0f;
            double b1 = (3*t*t*t - 6*t*t +4)/6.0f;
            double b2 = (-3*t*t*t +3*t*t + 3*t + 1)/6.0f;
            double b3 =  t*t*t/6.0f;
            
            // calculate the x,y and z of the curve point
            double x = b0 * GetPoint( start_cv + 0, pointList ).pose.position.x + 
                b1 * GetPoint( start_cv + 1, pointList ).pose.position.x +
		b2 * GetPoint( start_cv + 2, pointList ).pose.position.x +
		b3 * GetPoint( start_cv + 3, pointList ).pose.position.x ;
            
            double y = b0 * GetPoint( start_cv + 0, pointList ).pose.position.y + 
                b1 * GetPoint( start_cv + 1, pointList ).pose.position.y + 
                b2 * GetPoint( start_cv + 2, pointList ).pose.position.y + 
                b3 * GetPoint( start_cv + 3, pointList ).pose.position.y ;

            double z = b0 * GetPoint( start_cv + 0, pointList ).pose.position.z + 
                b1 * GetPoint( start_cv + 1, pointList ).pose.position.z +
                b2 * GetPoint( start_cv + 2, pointList ).pose.position.z +
		b3 * GetPoint( start_cv + 3, pointList ).pose.position.z ;
            
            geometry_msgs::PoseStamped pose;
            pose.header = GetPoint( start_cv + 0, pointList ).header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            
            resultList.push_back(pose);
       
        }
    }
    resultList.push_back(pointList.back());
}

void chaikinCurve (const std::vector<geometry_msgs::PoseStamped>& pointList, 
        std::vector<geometry_msgs::PoseStamped>& resultList, double k) {
    
	std::vector<geometry_msgs::PoseStamped> newPoints;

	// keep the first point
	newPoints.push_back(pointList[0]);
        
	for(unsigned int i=0;i<(pointList.size()-1);++i) {
	
		// get 2 original points
		const geometry_msgs::PoseStamped& p0 = pointList[i];
		const geometry_msgs::PoseStamped& p1 = pointList[i+1];
		geometry_msgs::PoseStamped Q;
		geometry_msgs::PoseStamped R;

                
                
		// average the 2 original points to create 2 new points. For each
		// CV, another 2 verts are created.
                Q.header = p0.header;
		Q.pose.position.x = k*p0.pose.position.x + (1-k)*p1.pose.position.x;
		Q.pose.position.y = k*p0.pose.position.y + (1-k)*p1.pose.position.y;
		Q.pose.position.z = k*p0.pose.position.z + (1-k)*p1.pose.position.z;

                R.header = p0.header;
		R.pose.position.x = (1-k)*p0.pose.position.x + k*p1.pose.position.x;
		R.pose.position.y = (1-k)*p0.pose.position.y + k*p1.pose.position.y;
		R.pose.position.z = (1-k)*p0.pose.position.z + k*p1.pose.position.z;

		newPoints.push_back(Q);
		newPoints.push_back(R);
	}
	// keep the last point
	newPoints.push_back(pointList[pointList.size()-1]);

	// update the points array
	resultList = newPoints;
}

bool TrajectoryPlanner::computePath(const KDL::Frame& initial, const KDL::Frame& goal, KDL::Path_Composite& path) {

    geometry_msgs::PoseStamped initialPoseStamped;
    geometry_msgs::PoseStamped goalPoseStamped;
    ConversionUtils conversion;

    conversion.poseKdlToRos(initial, initialPoseStamped.pose);
    initialPoseStamped.header.frame_id = frameId;
    conversion.ConversionUtils::poseKdlToRos(goal, goalPoseStamped.pose);
    goalPoseStamped.header.frame_id = frameId;

    std::vector<geometry_msgs::PoseStamped> poseStampedArray;
    bool result = pathPlanner->makePlan(initialPoseStamped, goalPoseStamped, poseStampedArray);
    if (result == true) { // plan has no orientation

        std::vector<geometry_msgs::PoseStamped> result;
        douglasPeucker(poseStampedArray, result, 0.1);
        
        ROS_INFO("Found a path, which has %lu points", poseStampedArray.size());
        
        
        nav_msgs::Path path;
       
        
       /* std::vector<geometry_msgs::PoseStamped> result1;
        chaikinCurve(result, result1, 0.75);
        std::vector<geometry_msgs::PoseStamped> result2;
        chaikinCurve(result1, result2, 0.75);*/
        
        std::vector<geometry_msgs::PoseStamped> result2;
        cubicBSplineCurve(result, result2, 100);
        
        path.header.frame_id = result2[0].header.frame_id;
        path.poses = result2;
        pathPublisher.publish(path);
        
        poseStampedArray.clear();
        
        poseStampedArray = result2;
        
        ROS_INFO("Reduced array has %lu point", result.size());
        
        ROS_INFO("Initial pose origin (x,y,z): %f, %f, %f",
                poseStampedArray.front().pose.position.x,
                poseStampedArray.front().pose.position.y,
                poseStampedArray.front().pose.position.z);
        ROS_INFO("      orientation (w,x,y,z): %f, %f, %f, %f",
                poseStampedArray.front().pose.orientation.w,
                poseStampedArray.front().pose.orientation.x,
                poseStampedArray.front().pose.orientation.y,
                poseStampedArray.front().pose.orientation.z);

        ROS_INFO("Goal pose origin (x,y,z): %f, %f, %f",
                poseStampedArray.back().pose.position.x,
                poseStampedArray.back().pose.position.y,
                poseStampedArray.back().pose.position.z);
        ROS_INFO("   orientation (w,x,y,z): %f, %f, %f, %f",
                poseStampedArray.back().pose.orientation.w,
                poseStampedArray.back().pose.orientation.x,
                poseStampedArray.back().pose.orientation.y,
                poseStampedArray.back().pose.orientation.z);
        
        
        poseStampedArray.front() = initialPoseStamped;
        std::vector<geometry_msgs::PoseStamped>::iterator it;

        ConversionUtils convert;

        double start = tf::getYaw(initialPoseStamped.pose.orientation);
        double end = tf::getYaw(goalPoseStamped.pose.orientation);
        double step = getShortestAngle(end, start) / poseStampedArray.size();

        for (it = poseStampedArray.begin(); it != poseStampedArray.end() - 1; ++it) {

            KDL::Frame f(KDL::Rotation::RPY(0, 0, start), KDL::Vector(0, 0, 0));

            start = start + step;

            geometry_msgs::Pose f1;
            convert.poseKdlToRos(f, f1);

            it->pose.orientation = f1.orientation;

        }


        KDL::Frame f(KDL::Rotation::RPY(0, 0, end), KDL::Vector(0, 0, 0));

        geometry_msgs::Pose f1;
        convert.poseKdlToRos(f, f1);

        poseStampedArray.back().pose.orientation = f1.orientation;
    }

    conversion.pathRosToKdl(poseStampedArray, path);

    return true;
}

bool TrajectoryPlanner::computeTrajectory(const KDL::Path& path, KDL::Trajectory_Composite& trajectory) {

    const double maxVel = 0.3;
    const double maxAcc = 0.05;

    ROS_INFO("Path converted to a trajectory, max velocity: %f m/s, max acceleration: %f m/s^2", maxVel, maxAcc);

    KDL::VelocityProfile_Trap* velocityProfile = new KDL::VelocityProfile_Trap(maxVel, maxAcc);
    KDL::Path* copyPath = const_cast<KDL::Path&> (path).Clone(); // Why KDL has no const version of Clone method?
    // They force me to do this!

    velocityProfile->SetProfile(0, copyPath->PathLength());
    KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(copyPath, velocityProfile);

    trajectory.Add(trajectorySegment);

    double dt = 1.0;
    for (double i = 0; i <= trajectory.Duration() + dt; i = i + dt) {
        cout << "pos" << ":" << trajectory.Pos(i).p <<  endl;
        cout << "vel" << ":" << trajectory.Vel(i).vel <<  endl;
        cout << "acc" << ":" << trajectory.Acc(i).vel <<  endl;
        cout << "---" << endl;
    }
    
    
    return true;
}
