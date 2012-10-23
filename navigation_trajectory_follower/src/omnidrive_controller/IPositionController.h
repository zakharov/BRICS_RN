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

#ifndef IPOSITIONCONTROLLER_H
#define	IPOSITIONCONTROLLER_H

#include <vector>

class TrajectoryWithId;
class Odometry;

/**
 * @brief Interface class for a position controller.
 */

class IPositionController {
public:

    /**
     * @brief Sets a trajectory to follow.
     * @param[in] trajectory - a target trajectory
     */
    virtual void setTargetTrajectory(const TrajectoryWithId& trajectory) = 0;

    /**
     * @brief Gets an actual trajectory
     * @return TrajectoryWithId is and aggregatin of KDL::Trajectory with frame id.
     */
    virtual const TrajectoryWithId& getTargetTrajectory() const = 0;

    /**
     * @brief Sets a goal pose to reach.
     * @param[in] targetOdometry - a goal pose
     */
    virtual void setTargetOdometry(const Odometry& targetOdometry) = 0;

    /**
     * @brief Gets an actual goal pose.
     * @return Odometry value of the actual goal pose.
     */
    virtual const Odometry& getTargetOdometry() const = 0;

    /**
     * @brief Returns true if target has been reached. otherwise false.
     */
    virtual bool isTargetReached() const = 0;

    /**
     * @brief Sets tolerance parameter for a controller.
     * @param[in] tolerance - required tolerance for positioning procedure.
     */
    virtual void setTolerance(const Odometry& tolerance) = 0;

    /**
     * @brief Get tolerance parameter.
     * @return tolerance of the positioning procedure.
     */
    virtual const Odometry& getTolerance() const = 0;

    /**
     * @brief Interface to compute desired odometry, given actual odometry and a time step.
     * @param[in] actualOdometry - actual odometry.
     * @param elapsedTimeInSec - a time step.
     */
    virtual const Odometry& computeNewOdometry(const Odometry& actualOdometry, double elapsedTimeInSec) = 0;

};

#endif	/* IPOSITIONCONTROLLER_H */

