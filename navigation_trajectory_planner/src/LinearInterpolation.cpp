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
#include "navigation_trajectory_common/Logger.h"

#ifdef DEBUG
#include "navigation_trajectory_common/Stopwatch.h"
#endif

#include "navigation_trajectory_planner/LinearInterpolation.h"

LinearInterpolation::LinearInterpolation() {

}

LinearInterpolation::LinearInterpolation(const LinearInterpolation& orig) {

}

LinearInterpolation::~LinearInterpolation() {

}

size_t LinearInterpolation::interpolate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, double step, size_t numberOfsteps = 0) {
#ifdef DEBUG
    Stopwatch stopwatch;
    stopwatch.start();
#endif
    size_t result = linearInterpolation(in, out, step, numberOfsteps);
#ifdef DEBUG  
    stopwatch.stop();
    LOG("Linear interpolation:");
    if (result > 0) {
        LOG("  - input end point: %f, %f", in.back().getFrame().p.x(), in.back().getFrame().p.y());
        LOG("  - output end point: %f, %f", out.back().getFrame().p.x(), out.back().getFrame().p.y());
    }
    LOG("  - step size: %f", step);
    LOG("  - iterations : %lu", result);
    LOG("  - duration : %f ms", stopwatch.getElapsedTime());

#endif  
    return result;
}

size_t LinearInterpolation::linearInterpolation(const std::vector <FrameWithId>& in,
        std::vector <FrameWithId>& out,
        double step,
        size_t numberOfSteps = 0) {

    if (in.empty())
        return 0;

    size_t counter = 0;
    out.clear();
    std::vector <FrameWithId> localCopy = in;
    double x0, y0, z0, xn, yn, zn, x, y, z, r, p;


    for (size_t i = 0; i < localCopy.size() - 1; ++i) {
        x0 = localCopy[i].getFrame().p.x();
        y0 = localCopy[i].getFrame().p.y();
        localCopy[i].getFrame().M.GetRPY(r, p, z0);

        xn = localCopy[i + 1].getFrame().p.x();
        yn = localCopy[i + 1].getFrame().p.y();
        localCopy[i + 1].getFrame().M.GetRPY(r, p, zn);

        double distance = sqrt((xn - x0)*(xn - x0) + (yn - y0)*(yn - y0));
        size_t steps = static_cast<size_t> (floor(distance / step));
        if (steps == 0) {
            LOG("Warning: step = 0, interpolation is not possible");
            return counter;
        }
        LOG("x0=%f, y0=%f, xn=%f, yn=%f", x0, y0, xn, yn);
        LOG("distance=%f, steps=%lu", distance, steps);

        for (size_t v = 0; v <= steps; ++v) {
            x = x0 + (xn - x0) / steps*v;
            y = y0 + (yn - y0) / steps*v;
            z = z0 + (zn - z0) / steps*v;

            LOG("x=%f, y=%f", x, y);

            KDL::Frame frame;
            std::string id = localCopy[i].id;
            frame.p.x(x);
            frame.p.y(y);
            frame.p.z(z);
            frame.M.DoRotZ(z);
            out.push_back(FrameWithId(frame,id));
            ++counter;
            if (counter >= (numberOfSteps - 1) && numberOfSteps != 0) {
                return counter;
            }
        }


    }



    return counter;

}