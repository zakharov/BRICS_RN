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

#ifndef COLLISIONCHECKINGROS_H
#define	COLLISIONCHECKINGROS_H

#include "navigation_trajectory_planner/ICollisionChecking.h"

namespace costmap_2d {
    class Costmap2DROS;
}

class FrameWithId;

/**
 * @brief Implementation of the collision checking interface. Current implementation
 * uses a ROS costmap collision checking routine.
 */

class CollisionCheckingRos : public ICollisionChecking {
public:

    /**
     * @brief Constructor
     * @param costmap - costmap_2d::Costmap2DROS a pointer to a costmap, which contains
     * actual representation of the environment
     */
    CollisionCheckingRos(costmap_2d::Costmap2DROS* costmap);
    
    /**
     * @brief Copy constructor
     */
    CollisionCheckingRos(CollisionCheckingRos& orig);

    /**
     * @brief Destructor
     */
    virtual ~CollisionCheckingRos();

    /**
     * @brief An interface for collision checking.
     * @param[in] path - std::vector <FrameWithId> actual path.
     * @param[in] actualPose - FrameWithId actual pose.
     * @return true if there is a collision, false otherwise. 
     */
    bool check(const std::vector <FrameWithId>& path, const FrameWithId& actualPose);

private:

    /**
     * @brief Implementation of collision checking routine.
     * @param[in] path - std::vector <FrameWithId> actual path.
     * @param[in] actualPose - FrameWithId actual pose.
     * @param interpolationStep - interpolation step.
     * @param numberOfSteps - number of steps
     * @return true if there is a collision, false otherwise. 
     */
    bool collisionCheck(const std::vector <FrameWithId>& path,
            const FrameWithId& actualPose,
            double interpolationStep,
            unsigned int numberOfSteps);

    /**
     * @brief Pointer to the actual map of the environment.
     */
    costmap_2d::Costmap2DROS* costmap;

};

#endif	/* COLLISIONCHECKINGROS_H */

