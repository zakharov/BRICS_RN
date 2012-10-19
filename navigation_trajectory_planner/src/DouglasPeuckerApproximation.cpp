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

#include "navigation_trajectory_common/FrameWithId.h"
#include "navigation_trajectory_common/Utilities.h"
#include "navigation_trajectory_common/Logger.h"

#ifdef DEBUG
#include "navigation_trajectory_common/Stopwatch.h"
#endif

#include "navigation_trajectory_planner/DouglasPeuckerApproximation.h"

DouglasPeuckerApproximation::DouglasPeuckerApproximation() {
}

DouglasPeuckerApproximation::DouglasPeuckerApproximation(const DouglasPeuckerApproximation& orig) {
}

DouglasPeuckerApproximation::~DouglasPeuckerApproximation() {
}

void DouglasPeuckerApproximation::approximate(const std::vector <FrameWithId>& in,
        std::vector <FrameWithId>& out) {
    approximate(in, out, 0.1);
}

void DouglasPeuckerApproximation::approximate(const std::vector <FrameWithId>& in,
        std::vector <FrameWithId>& out, double epsilon) {

#ifdef DEBUG
    Stopwatch stopwatch;
    stopwatch.start();
#endif
    std::vector <FrameWithId> inputCopy;
    inputCopy = in;
    out.clear();
    int counter = 0;
    douglasPeucker(inputCopy, out, counter, epsilon);
#ifdef DEBUG  
    stopwatch.stop();
    LOG("Douglas-Peucker approximation algorithm:");
    LOG("  - input size: %lu", inputCopy.size());
    LOG("  - epsilon: %f", epsilon);
    LOG("  - output size: %lu", out.size());
    LOG("  - iterations : %d", counter);
    LOG("  - duration : %f ms", stopwatch.getElapsedTime());
#endif
}

void DouglasPeuckerApproximation::douglasPeucker(const std::vector <FrameWithId>& pointList,
        std::vector <FrameWithId>& resultList, int& numberOfIterations, double epsilon) {

    ++numberOfIterations;
    if (pointList.size() < 2) {
        resultList = pointList;
        return;
    }

    double dmax = 0;
    unsigned int index = 0;

    std::vector <FrameWithId> result1;
    std::vector <FrameWithId> result2;

    for (unsigned int i = 1; i < pointList.size() - 1; i++) {
        double d = utilities::perpendicularDistance(pointList[i], pointList[0],
                pointList[pointList.size() - 1]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }
    //If max distance is greater than epsilon, recursively simplify
    if (dmax >= epsilon) {
        //Recursive call
        std::vector <FrameWithId> pointSubList1(&pointList[0],
                &pointList[index]);
        std::vector <FrameWithId> pointSubList2(&pointList[index],
                &pointList[pointList.size()]);

        douglasPeucker(pointSubList1, result1, numberOfIterations, epsilon);
        douglasPeucker(pointSubList2, result2, numberOfIterations, epsilon);
        resultList.insert(resultList.begin(), result1.begin(), result1.end() - 1);
        resultList.insert(resultList.end(), result2.begin(), result2.end());
    } else {
        resultList.push_back(pointList[0]);
        resultList.push_back(pointList[pointList.size() - 1]);
    }
}