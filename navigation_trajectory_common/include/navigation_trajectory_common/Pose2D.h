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

#ifndef POSE2D_H
#define	POSE2D_H

/**
 * @brief Class represents pose 2D data.
 */

class Pose2D {
public:

    /**
     * @brief Constructor.
     */
    Pose2D();

    /**
     * @brief Constructor.
     * @param[in] x - coordinate
     * @param[in] y - coordinate
     * @param[in] theta - coordinate
     */
    Pose2D(float x, float y, float theta);

    /**
     * @brief Copy constructor.
     */
    Pose2D(const Pose2D& orig);

    /**
     * @brief Destructor.
     */
    virtual ~Pose2D();

    /**
     * @brief Summation.
     */
    friend Pose2D operator+(const Pose2D& op1, const Pose2D& op2);

    /**
     * @brief Assignment.
     */
    const Pose2D& operator=(const Pose2D& orig);

    /**
     * @brief Getting x coordinate.
     */
    float getX() const;

    /**
     * @brief Getting y coordinate.
     */
    float getY() const;

    /**
     * @brief Getting theta angle.
     */
    float getTheta() const;

private:

    /**
     * @brief x coordinate.
     */
    float x;

    /**
     * @brief y coordinate.
     */
    float y;

    /**
     * @brief theta coordinate.
     */
    float theta;

};

#endif	/* POSE2D_H */

