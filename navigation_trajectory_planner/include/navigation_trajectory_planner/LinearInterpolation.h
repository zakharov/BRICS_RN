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

#ifndef LINEARINTERPOLATION_H
#define	LINEARINTERPOLATION_H

#include "navigation_trajectory_planner/IPathInterpolation.h"

/**
 * @brief Implementation of the interface class for path interpolation algorithms, 
 * linear path interpolation algorithm.
 */

class LinearInterpolation : IPathInterpolation {
public:

    /**
     * Constructor.
     */
    LinearInterpolation();

    /**
     * @brief Copy constructor.
     */
    LinearInterpolation(const LinearInterpolation& orig);

    /**
     * @brief Destructor.
     */
    virtual ~LinearInterpolation();

    /**
     * @brief An interface for linear path approximation algorithms.
     * @param[in] in - std::vector <FrameWithId> input path.
     * @param[out] out - std::vector <FrameWithId> resulting interpolated path.
     * @param step - step size.
     * @param numberOfSteps - number of steps.
     */
    size_t interpolate(const std::vector <FrameWithId>& in,
            std::vector <FrameWithId>& out,
            double step,
            size_t numberOfSteps);

private:

    /**
     * @brief An implementation of the linear path approximation algorithms.
     * @param[in] in - std::vector <FrameWithId> input path.
     * @param[out] out - std::vector <FrameWithId> resulting interpolated path.
     * @param step - step size.
     * @param numberOfSteps - number of steps.
     */
    size_t linearInterpolation(const std::vector <FrameWithId>& in,
            std::vector <FrameWithId>& out,
            double step,
            size_t numberOfsteps);

};

#endif	/* LINEARINTERPOLATION_H */

