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

#ifndef IPATHINTERPOLATION_H
#define	IPATHINTERPOLATION_H

#include <vector>

class FrameWithId;

/**
 * @brief Interface class for path interpolation algorithms.
 * 
 * Here, path interpolation referrs to adding additional intermediate waypoints 
 * to a sparse path representation.
 */

class IPathInterpolation {
    /**
     * @brief An interface for path approximation.
     * 
     * Here, path interpolation referrs to adding additional intermediate waypoints 
     * to a sparse path representation. The exact semantics depend upon the used 
     * algorithm, see the implementation classes for details and for possible 
     * customization parameters.
     * 
     * The resolution of the interpolated path can be specified either by defining 
     * the desired cartessian distance between waypoints, or by specifying the total 
     * number of waypoints to use.  The behavior, when both are specified, depends on 
     * the actual implementation, see the path interpolation implementation classes.  
     * In general, the behavior in that case is not defined.
     * 
     * In a sense, this is the opposite operation of path approximation as performed
     * by the IPathApproximate classes. 
     * 
     * @param[in] in - std::vector <FrameWithId> input path.
     * @param[out] out - std::vector <FrameWithId> resulting interpolated path.
     * @param step - step size.  This is in base units of the used coordinate system, 
     * usually in meters. The resulting path will have a waypoint approximately every
     * @p step units.
     * 
     * @param numberOfSteps - number of steps. This is optional and computed from 
     * @p step and the path length if not given.
     * 
     * @return number of waypoints in the returned path @p out.
     * 
     */
    virtual size_t interpolate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, double step, size_t numberOfSteps) = 0;
};



#endif	/* IPATHINTERPOLATION_H */

