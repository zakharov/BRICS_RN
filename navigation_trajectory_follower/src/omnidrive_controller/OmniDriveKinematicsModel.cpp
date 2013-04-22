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

#include "OmniDriveKinematicsModel.h"
#include "navigation_trajectory_common/Pose2D.h"
#include "Odometry.h"
#include <cmath>

OmniDriveKinematicsModel::OmniDriveKinematicsModel() {
}

OmniDriveKinematicsModel::OmniDriveKinematicsModel(const OmniDriveKinematicsModel& orig) {
}

OmniDriveKinematicsModel::~OmniDriveKinematicsModel() {
}

void OmniDriveKinematicsModel::convert(const double& x1, const double& y1, const double& theta, double& x2, double& y2, bool inverse) {

    double k = (inverse == true) ? 1 : -1;

    x2 = (x1 * cos(theta) + y1 * sin(theta)) * k;
    y2 = (y1 * cos(theta) - x1 * sin(theta)) * k;
}

void OmniDriveKinematicsModel::convertToLocalReferenceFrame(const Odometry& globalFrame, Odometry& localFrame) {

    double locPosX = 0;
    double locPosY = 0;

    convert(globalFrame.getPose2D().getX(),
            globalFrame.getPose2D().getY(),
            globalFrame.getPose2D().getTheta(),
            locPosX,
            locPosY,
            true);

    double locVelX = 0;
    double locVelY = 0;

    convert(globalFrame.getTwist2D().getX(),
            globalFrame.getTwist2D().getY(),
            globalFrame.getPose2D().getTheta(),
            locVelX,
            locVelY,
            true);

    localFrame = Odometry(Pose2D(locPosX, locPosY, globalFrame.getPose2D().getTheta()),
            Twist2D(locVelX, locVelY, globalFrame.getTwist2D().getTheta()));
}

void OmniDriveKinematicsModel::convertToGlobalReferenceFrame(const Odometry& localFrame, Odometry& globalFrame) {

    double globPosX = 0;
    double globPosY = 0;

    convert(localFrame.getPose2D().getX(),
            localFrame.getPose2D().getY(),
            localFrame.getPose2D().getTheta(),
            globPosX,
            globPosY,
            false);

    double globVelX = 0;
    double globVelY = 0;

    convert(localFrame.getTwist2D().getX(),
            localFrame.getTwist2D().getY(),
            localFrame.getPose2D().getTheta(),
            globVelX,
            globVelY,
            false);

    globalFrame = Odometry(Pose2D(globPosX, globPosY, localFrame.getPose2D().getTheta()),
            Twist2D(globVelX, globVelY, localFrame.getTwist2D().getTheta()));
}
