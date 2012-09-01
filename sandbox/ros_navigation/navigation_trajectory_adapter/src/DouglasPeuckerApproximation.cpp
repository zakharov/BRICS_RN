/* 
 * File:   DouglasPeuckerApproximation.cpp
 * Author: alexey
 * 
 * Created on August 31, 2012, 2:55 PM
 */

#include "navigation_trajectory_adapter/DouglasPeuckerApproximation.h"
#include "navigation_trajectory_adapter/FrameWithId.h"
#include "navigation_trajectory_adapter/PathUtilities.h"
#include "navigation_trajectory_adapter/Logger.h"

#ifdef DEBUG
#include "navigation_trajectory_adapter/Stopwatch.h"
#endif


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
        std::vector <FrameWithId>& resultList,  int& numberOfIterations, double epsilon) {

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
        double d = perpendicularDistance(pointList[i], pointList[0],
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

        douglasPeucker(pointSubList1, result1,numberOfIterations, epsilon);
        douglasPeucker(pointSubList2, result2,numberOfIterations, epsilon);
        resultList.insert(resultList.begin(), result1.begin(), result1.end() - 1);
        resultList.insert(resultList.end(), result2.begin(), result2.end());
    } else {
        resultList.push_back(pointList[0]);
        resultList.push_back(pointList[pointList.size() - 1]);
    }
}