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

#ifndef DOUGLASPEUCKERAPPROXIMATION_H
#define	DOUGLASPEUCKERAPPROXIMATION_H

#include "navigation_trajectory_planner/IPathApproximation.h"
#include <vector>

class FrameWithId;

/**
 * @brief Implementation of path approximation algorithm. Ramer–Douglas–Peucker algorithm 
 * is an algorithm for reducing the number of points in a curve that is approximated by 
 * a series of points.
 */


class DouglasPeuckerApproximation : public IPathApproximation {
public:

    /**
     * @brief Constructor
     */
    DouglasPeuckerApproximation();

    /**
     * @brief Copy constructor
     */
    DouglasPeuckerApproximation(const DouglasPeuckerApproximation& orig);

    /**
     * @brief Destructor
     */
    virtual ~DouglasPeuckerApproximation();

    /**
     * @brief Reduces the number of points of an input path by Ramer–Douglas–Peucker algorithm.
     * 
     * The @c epsilon parameter of the algorithm defaults to 0.1.
     * 
     * @param[in] in - std::vector <FrameWithId> input path.
     * @param[out] out - std::vector <FrameWithId> resulting smoothed path.
     */
    void approximate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out);

    /**
     * @brief Reduces the number of points of an input path by Ramer–Douglas–Peucker algorithm.
     * @param[in] in - std::vector <FrameWithId> input path.
     * @param[out] out - std::vector <FrameWithId> resulting path with reduced number of points.
     * @param epsilon - minimum distance between points.
     */
    void approximate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, double epsilon);


private:

    /**
     * @brief Reduces the number of points of an input path by Ramer–Douglas–Peucker algorithm.
     * @param[in] in - std::vector <FrameWithId> input path.
     * @param[out] out - std::vector <FrameWithId> resulting path with reduced number of points.
     * @param counter - internal number of operations' counter.
     * @param epsilon - minimum distance between points.
     */
    void douglasPeucker(const std::vector <FrameWithId>& pointList,
            std::vector <FrameWithId>& resultList, int& counter, double epsilon);


};

#endif	/* DOUGLASPEUCKERAPPROXIMATION_H */

