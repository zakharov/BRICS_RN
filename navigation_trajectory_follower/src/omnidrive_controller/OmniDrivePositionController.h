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

#ifndef OMNIDRIVEPOSITIONCONTROLLER_H
#define	OMNIDRIVEPOSITIONCONTROLLER_H

#include "IPositionController.h"
#include "Odometry.h"

#include "navigation_trajectory_common/TrajectoryWithId.h"
#include <vector>

/**
 * @brief Implementation of the interface class for a position controller for omnidirectional mobile platform.
 */

class OmniDrivePositionController : public IPositionController {
public:

    /**
     * @brief Constructor.
     * @param positionGainTranslation - gain coefficient for a position feedback for linear motion.
     * @param velocityGainTranslation - gain coefficient for a velocity feedback for linear motion.
     * @param positionGainRotation - gain coefficient for a position feedback for angular motion.
     * @param velocityGainRotation - gain coefficient for a velocity feedback for angular motion.
     * @param positionToleranceTranslation - tolerance value for a position for linear motion.
     * @param velocityToleranceTranslation - tolerance value for a velocity for linear motion.
     * @param positionToleranceRotation - tolerance value for a position for linear motion.
     * @param velocityToleranceRotation - tolerance value for a velocity for linear motion.
     */
    OmniDrivePositionController(double positionGainTranslation,
            double velocityGainTranslation,
            double positionGainRotation,
            double velocityGainRotation,
            double positionToleranceTranslation,
            double velocityToleranceTranslation,
            double positionToleranceRotation,
            double velocityToleranceRotation);

    /**
     * @brief Default constructor.
     */
    OmniDrivePositionController();
    OmniDrivePositionController(const OmniDrivePositionController& orig);
    virtual ~OmniDrivePositionController();

    /**
     * @brief Sets a trajectory to follow.
     * @param[in] trajectory - a target trajectory
     */
    void setTargetTrajectory(const TrajectoryWithId& trajectory);

    /**
     * @brief Gets an actual trajectory
     * @return TrajectoryWithId is and aggregatin of KDL::Trajectory with frame id.
     */
    const TrajectoryWithId& getTargetTrajectory() const;

    /**
     * @brief Sets a goal pose to reach.
     * @param[in] targetOdometry - a goal pose
     */
    void setTargetOdometry(const Odometry& targetOdometry);

    /**
     * @brief Gets an actual goal pose.
     * @return Odometry value of the actual goal pose.
     */
    const Odometry& getTargetOdometry() const;

    /**
     * @brief Sets tolerance parameter for a controller.
     * @param[in] tolerance - required tolerance for positioning procedure.
     */
    void setTolerance(const Odometry& tolerance);

    /**
     * @brief Get tolerance parameter.
     * @return tolerance of the positioning procedure.
     */
    const Odometry& getTolerance() const;

    /**
     * @brief Computes desired odometry, given actual odometry and a time step.
     * @param[in] actualOdometry - actual odometry.
     * @param elapsedTimeInSec - a time step.
     * @return setpoint odometry, which includes updated velocities and pose
     */
    const Odometry& computeNewOdometry(const Odometry& actualOdometry, double elapsedTimeInSec);

    /**
     * @brief Returns true if target has been reached. otherwise false.
     */
    bool isTargetReached() const;

private:
    /**
     * @brief Calculates a distance between two points (for 2D space).
     * @param[in] actualPose - 1st point coordinates.
     * @param[in] goalPose - 2nd point coordinates.
     * @return distance
     */
    float getDistance(const Pose2D& actualPose, const Pose2D& goalPose);

    /**
     * @brief Calculates a shortest rotation angle (for 2D space).
     * @param[in] goalAngle - 1st angle.
     * @param[in] actualAngle - 2nd angle.
     * @return shortest angle
     */
    float getShortestAngle(float goalAngle, float actualAngle);

    /**
     * @brief Resets internal flags.
     */
    void resetFlags();

    /**
     * @brief Checks if the goal has been reached.
     * @param[out] translation - flag is set to true if target location is reached, otherwise false.
     * @param[out] rotation - flag is set to true if target rotation is reached, otherwise false.
     */
    void targetReached(bool& translation, bool& rotation);

private:

    /**
     * @brief Holds gain values.
     */
    Odometry gains;

    /**
     * @brief Holds tolerance values.
     */
    Odometry tolerance;

    /**
     * @brief Holds target trajectory.
     */
    TrajectoryWithId targetTrajectory;

    /**
     * @brief Holds goal position.
     */
    Odometry targetOdometry;

    /**
     * @brief Holds last setpoint for odometry.
     */
    Odometry computedOdometry;

    /**
     * @brief Holds odometry feedback from the sensors.
     */
    Odometry actualOdometry;

    /**
     * @brief Holds actual execution time.
     */
    double actualTime;

    /**
     * @brief Internal flags.
     */
    bool translationFlag;

    /**
     * @brief Internal flags.
     */
    bool rotationFlag;

};

#endif	/* OMNIDRIVEPOSITIONCONTROLLER_H */

