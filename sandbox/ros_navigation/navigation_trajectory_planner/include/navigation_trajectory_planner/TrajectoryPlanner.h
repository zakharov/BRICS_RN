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


#ifndef TRAJECTORYPLANNER_H
#define	TRAJECTORYPLANNER_H

#include <vector>


namespace KDL {
class Frame;
class Path;
class Path_Composite;
class Trajectory;
class Trajectory_Composite;
}

namespace nav_core {
class BaseGlobalPlanner;
}

class TrajectoryPlanner {
public:
    TrajectoryPlanner(nav_core::BaseGlobalPlanner* planner);
    TrajectoryPlanner(const TrajectoryPlanner& orig);
    virtual ~TrajectoryPlanner();
    
    bool computePath(const KDL::Frame& initial, const KDL::Frame& goal, KDL::Path_Composite& path);
    bool computeTrajectory(const KDL::Path& path, KDL::Trajectory_Composite& trajectory);
    
   
private:
    
    nav_core::BaseGlobalPlanner* pathPlanner;

/*    
    class TrajectoryPlannerNode {
public:
    TrajectoryPlannerNode(std::string name, costmap_2d::Costmap2DROS& costmap);
    TrajectoryPlannerNode(const TrajectoryPlannerNode& orig);
    virtual ~TrajectoryPlannerNode();

private:
    
    void prunePlan(const std::vector<geometry_msgs::PoseStamped>& actualPlan, std::vector<geometry_msgs::PoseStamped>& prunedPlan);
    void publishTrajectory(const geometry_msgs::PoseStamped& goal);
    
private:
    
    ros::NodeHandle node;
    
    ros::Publisher trajectoryPublisher;
    ros::Subscriber costmapSubscriber;
    ros::Subscriber goalSubscriber;
    
    std::string robotBaseFrame;
    std::string globalFrame;
    std::string nodeName;
    std::string globalTrajectoryPlanner;
    costmap_2d::Costmap2DROS& globalCostmap;
    nav_core::BaseGlobalPlanner* planner;
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgpLoader;
 */ 
    
};

#endif	/* TRAJECTORYPLANNER_H */

