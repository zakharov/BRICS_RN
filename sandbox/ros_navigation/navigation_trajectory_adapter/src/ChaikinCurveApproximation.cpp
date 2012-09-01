/* 
 * File:   ChaikinCurveApproximation.cpp
 * Author: alexey
 * 
 * Created on August 31, 2012, 3:23 PM
 */

#include "navigation_trajectory_adapter/ChaikinCurveApproximation.h"
#include "navigation_trajectory_adapter/FrameWithId.h"
#include "navigation_trajectory_adapter/Logger.h"
#include "navigation_trajectory_adapter/Stopwatch.h"

ChaikinCurveApproximation::ChaikinCurveApproximation() {
    
}

ChaikinCurveApproximation::ChaikinCurveApproximation(const ChaikinCurveApproximation& orig) {
    
}

ChaikinCurveApproximation::~ChaikinCurveApproximation() {
}

void ChaikinCurveApproximation::approximate (const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, unsigned int lod) {
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

void ChaikinCurveApproximation::approximate (const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out) {
    
    approximate (in, out, 1);
}

unsigned int ChaikinCurveApproximation::chaikinCurve (const std::vector <FrameWithId>& pointList, 
        std::vector <FrameWithId>& resultList) {
    
    if (pointList.size() == 0)
        return 0;
    
    unsigned int counter = 0;
	// keep the first point
	resultList.push_back(pointList[0]);
        
        double k = 0.75;
	for(unsigned int i=0; i<(pointList.size()-1); ++i) {
            ++counter;
		// get 2 original points
		const FrameWithId& p0 = pointList[i];
		const FrameWithId& p1 = pointList[i+1];
		FrameWithId Q;
		FrameWithId R;
                
		// average the 2 original points to create 2 new points. For each
		// CV, another 2 verts are created.
                Q.id = p0.id;
                Q.p.x(k*p0.p.x() + (1-k)*p1.p.x());
		Q.p.y(k*p0.p.y() + (1-k)*p1.p.y());
		Q.p.z(k*p0.p.z() + (1-k)*p1.p.z());

                R.id = p0.id;
		R.p.x((1-k)*p0.p.x() + k*p1.p.x());
		R.p.y((1-k)*p0.p.y() + k*p1.p.y());
		R.p.z((1-k)*p0.p.z() + k*p1.p.z());

		resultList.push_back(Q);
		resultList.push_back(R);
	}
	// keep the last point
	resultList.push_back(pointList[pointList.size()-1]);
        return counter;
}