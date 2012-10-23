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

#ifndef OMNIDRIVEKINEMATICSMODEL_H
#define	OMNIDRIVEKINEMATICSMODEL_H

#include "IKinematicsModel.h"

class Odometry;

/**
 * @brief Implementation of the interface class for computing motion in global and local reference frames
 * for omnidirectional mobile platform.
 */

class OmniDriveKinematicsModel : public IKinematicsModel {
public:

    /**
     * @brief Constructor.
     */
    OmniDriveKinematicsModel();

    /**
     * @brief Copy constructor.
     */
    OmniDriveKinematicsModel(const OmniDriveKinematicsModel& orig);

    /**
     * @brief Destructor.
     */
    virtual ~OmniDriveKinematicsModel();

    /**
     * @brief Mapping motion in the global reference frame to motion
     * in terms of the local reference frame for the omnidirectional platform.
     * @param[in] globalFrame - odometry in the global reference frame
     * @param[out]  localFrame - resulting conversion to the local reference frame
     */
    void convertToLocalReferenceFrame(const Odometry& globalFrame, Odometry& localFrame);

    /**
     * @brief Mapping motion in the local reference frame to motion
     * in terms of the global reference frame.
     * @param[in] localFrame - odometry in the local reference frame
     * @param[out]  globalFrame - resulting conversion to the global reference frame
     */
    void convertToGlobalReferenceFrame(const Odometry& localFrame, Odometry& globalFrame);

private:

    /**
     * @brief Implementation of the interface class for computing motion in global and local reference frames
     * for omnidirectional mobile platform.
     */
    void convert(const double& x1, const double& y1, const double& theta, double& x2, double& y2, bool inverse);

};

#endif	/* OMNIDRIVEKINEMATICSMODEL_H */

