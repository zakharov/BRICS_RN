/* 
 * File:   PathUtilities.h
 * Author: alexey
 *
 * Created on September 1, 2012, 2:46 PM
 */

#ifndef PATHUTILITIES_H
#define	PATHUTILITIES_H

#include <navigation_trajectory_adapter/FrameWithId.h>
#include <vector>

inline double perpendicularDistance(const FrameWithId& point,
        const FrameWithId& linePoint1, const FrameWithId& linePoint2) {

    double x0 = point.p.x();
    double y0 = point.p.y();

    double x1 = linePoint1.p.x();
    double y1 = linePoint1.p.y();

    double x2 = linePoint2.p.x();
    double y2 = linePoint2.p.y();

    double dist = fabs((x2 - x1)*(y1 - y0)-(x1 - x0)*(y2 - y1)) /
            sqrt((x2 - x1)*(x2 - x1)+(y2 - y1)*(y2 - y1));

    return dist;

}

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

#endif	/* PATHUTILITIES_H */

