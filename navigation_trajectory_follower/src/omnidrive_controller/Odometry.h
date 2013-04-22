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

#ifndef ODOMETRY_H
#define	ODOMETRY_H

#include "navigation_trajectory_common/Twist2D.h"
#include "navigation_trajectory_common/Pose2D.h"

/**
 * @brief Class represents odometry data, which is an aggregation of Twist2D and Pose2D.
 */
class Odometry {
public:

    /**
     * @brief Constructor.
     */
    Odometry(); // creates odometry with 0,0,0 pose and 0,0,0 twist

    /**
     * @brief Constructor.
     * @param[in] pose - initial 2D pose.
     * @param[in] twist - initial 2D twist.
     */
    Odometry(const Pose2D& pose, const Twist2D& twist);

    /**
     * @brief Constructor.
     * @param[in] pose - initial pose.
     */
    Odometry(const Pose2D& pose);

    /**
     * @brief Constructor.
     * @param[in] twist - initial twist.
     */
    Odometry(const Twist2D& twist);

    /**
     * @brief Copy constructor.
     */
    Odometry(const Odometry& orig);

    /**
     * @brief Assignment operator.
     */
    const Odometry& operator=(const Odometry& orig);

    /**
     * @brief Class represents an odometry data.
     */
    const Pose2D& getPose2D() const;

    /**
     * @brief Class represents an odometry data.
     */
    void setPose2D(const Pose2D& twist);

    /**
     * @brief Class represents an odometry data.
     */
    const Twist2D& getTwist2D() const;

    /**
     * @brief Class represents an odometry data.
     */
    void setTwist2D(const Twist2D& twist);

    /**
     * @brief Class represents an odometry data.
     */
    virtual ~Odometry();

private:

    /**
     * @brief Class represents an odometry data.
     */
    Pose2D pose;

    /**
     * @brief Class represents an odometry data.
     */
    Twist2D twist;
};

#endif	/* ODOMETRY_H */

