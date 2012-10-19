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

class OmniDrivePositionController : public IPositionController {
public:
    OmniDrivePositionController(double positionGainTranslation,
            double velocityGainTranslation,
            double positionGainRotation,
            double velocityGainRotation,
            double positionToleranceTranslation,
            double velocityToleranceTranslation,
            double positionToleranceRotation,
            double velocityToleranceRotation);

    OmniDrivePositionController();
    OmniDrivePositionController(const OmniDrivePositionController& orig);
    virtual ~OmniDrivePositionController();

    void setTargetTrajectory(const TrajectoryWithId& trajectory);
    const TrajectoryWithId& getTargetTrajectory() const;
    //void setTargetTrajectory(const std::vector<Odometry>& trajectory);
    //void setTargetTrajectory(const std::vector<Odometry>& trajectory);
    //const std::vector<Odometry>& getTargetTrajectory() const;

    void setTargetOdometry(const Odometry& targetOdometry);
    const Odometry& getTargetOdometry() const;

    void setTolerance(const Odometry& tolerance);
    const Odometry& getTolerance() const;

    const Odometry& computeNewOdometry(const Odometry& actualOdometry, double elapsedTimeInSec);
    bool isTargetReached() const;

private:

    float getDistance(const Pose2D& actualPose, const Pose2D& goalPose);
    float getShortestAngle(float goalAngle, float actualAngle);
    void resetFlags();
    void targetReached(bool& translation, bool& rotation);

private:

    //KDL::Trajectory_Composite* trajectoryComposite;
    Odometry gains;
    Odometry tolerance;
    TrajectoryWithId targetTrajectory;
    //std::vector<Odometry> targetTrajectory;
    Odometry targetOdometry;
    Odometry computedOdometry;
    Odometry actualOdometry;
    double actualTime;
    double timeOffset;


    bool translationFlag;
    bool rotationFlag;

};

#endif	/* OMNIDRIVEPOSITIONCONTROLLER_H */

