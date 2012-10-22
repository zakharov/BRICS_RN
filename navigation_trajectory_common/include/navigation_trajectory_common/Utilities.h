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

#ifndef UTILITIES_H
#define	UTILITIES_H

#include <vector>
#include "FrameWithId.h"

#include <kdl/frames.hpp>

/**
 * @brief Some commonly used routines. 
 */

namespace utilities {

    /**
     * @brief Calculates and returns perpendicular distance form the given point to the line (for 2D space)
     * @param[in] point - FrameWithId coordinates of the point.
     * @param[in] linePoint1 - FrameWithId 1st coordinate of the line.
     * @param[in] linePoint2 - FrameWithId 2nd coordinate of the line.
     */
    inline double perpendicularDistance(const FrameWithId& point,
            const FrameWithId& linePoint1, const FrameWithId& linePoint2) {

        double x0 = point.getFrame().p.x();
        double y0 = point.getFrame().p.y();

        double x1 = linePoint1.getFrame().p.x();
        double y1 = linePoint1.getFrame().p.y();

        double x2 = linePoint2.getFrame().p.x();
        double y2 = linePoint2.getFrame().p.y();

        double dist = fabs((x2 - x1)*(y1 - y0)-(x1 - x0)*(y2 - y1)) /
                sqrt((x2 - x1)*(x2 - x1)+(y2 - y1)*(y2 - y1));

        return dist;

    }

   /**
     * @brief Calculates pruned path, builds perpendiculars to the 
     * original path, takes the shortest perpendicular as a trimming line, 
     * trims the path, insert the point argument as a new starting point of 
     * the path (for 2D space). 
     * @param[in] path - std::vector<FrameWithId> original path, a vector of FrameWithId.
     * @param[in] point - FrameWithId trimming point.
     * @param[out] prunedPath - std::vector<FrameWithId> resulting pruned path.
     */
    inline void prunePath(const std::vector<FrameWithId>& path,
            const FrameWithId& point,
            std::vector<FrameWithId>& prunedPath) {

        std::vector<FrameWithId> localPath;
        localPath = path;

        double dmax = perpendicularDistance(localPath[0], localPath[1], point);
        unsigned int index = 1;

        for (unsigned int i = 1; i < localPath.size(); i++) {
            double d = perpendicularDistance(localPath[i - 1], localPath[i],
                    point);
            if (d <= dmax) {
                index = i;
                dmax = d;
            }
        }

        prunedPath.push_back(point);
        prunedPath.insert(prunedPath.end(), localPath.begin() + index, localPath.end());

    }

}

#endif	/* UTILITIES_H */

