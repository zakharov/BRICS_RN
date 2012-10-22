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
#include "navigation_trajectory_common/Stopwatch.h"

#ifdef DEBUG
#include "navigation_trajectory_common/Logger.h"
#endif

#include "navigation_trajectory_planner/ChaikinCurveApproximation.h"

#include <kdl/frames.hpp>

ChaikinCurveApproximation::ChaikinCurveApproximation() {

}

ChaikinCurveApproximation::ChaikinCurveApproximation(const ChaikinCurveApproximation& orig) {

}

ChaikinCurveApproximation::~ChaikinCurveApproximation() {
}

void ChaikinCurveApproximation::approximate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, unsigned int lod) {
#ifdef DEBUG
    Stopwatch stopwatch;
    stopwatch.start();
#endif
    std::vector <FrameWithId> inputCopy;
    inputCopy = in;
    unsigned int counter = 0;
    for (unsigned int i = 0; i < lod; ++i) {
        out.clear();
        counter += chaikinCurve(inputCopy, out);
        inputCopy = out;
    }
#ifdef DEBUG  
    stopwatch.stop();
    LOG("Chaikin curve approximation algorithm:");
    LOG("  - input size: %lu", in.size());
    LOG("  - level of detail: %d", lod);
    LOG("  - output size: %lu", out.size());
    LOG("  - iterations : %d", counter);
    LOG("  - duration : %f ms", stopwatch.getElapsedTime());
#endif    
}

void ChaikinCurveApproximation::approximate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out) {

    approximate(in, out, 1);
}

unsigned int ChaikinCurveApproximation::chaikinCurve(const std::vector <FrameWithId>& pointList,
        std::vector <FrameWithId>& resultList) {

    if (pointList.size() == 0)
        return 0;

    unsigned int counter = 0;
    // keep the first point
    resultList.push_back(pointList[0]);

    double k = 0.75;
    for (unsigned int i = 0; i < (pointList.size() - 1); ++i) {
        ++counter;
        // get 2 original points
        const KDL::Frame& p0 = pointList[i].getFrame();
        std::string p0Id = pointList[i].id;
        const KDL::Frame& p1 = pointList[i + 1].getFrame();
        std::string p1Id = pointList[i + 1].id;
        KDL::Frame Q;
        KDL::Frame R;

        // average the 2 original points to create 2 new points. For each
        // CV, another 2 verts are created.
        Q.p.x(k * p0.p.x() + (1 - k) * p1.p.x());
        Q.p.y(k * p0.p.y() + (1 - k) * p1.p.y());
        Q.p.z(k * p0.p.z() + (1 - k) * p1.p.z());

        R.p.x((1 - k) * p0.p.x() + k * p1.p.x());
        R.p.y((1 - k) * p0.p.y() + k * p1.p.y());
        R.p.z((1 - k) * p0.p.z() + k * p1.p.z());

        resultList.push_back(FrameWithId(Q, p0Id));
        resultList.push_back(FrameWithId(R, p1Id));
    }
    // keep the last point
    resultList.push_back(pointList[pointList.size() - 1]);
    return counter;
}