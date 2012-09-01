/* 
 * File:   Stopwatch.cpp
 * Author: alexey
 * 
 * Created on August 31, 2012, 4:57 PM
 */

#include "navigation_trajectory_adapter/Stopwatch.h"

Stopwatch::Stopwatch() {
    elapsedTime = 0;
    isPause = false;
}

Stopwatch::Stopwatch(const Stopwatch& orig) {
    isPause = orig.isPause;
    elapsedTime = orig.elapsedTime;
    startTime = orig.startTime;
    stopTime = orig.stopTime;
}

Stopwatch::~Stopwatch() {
}

void Stopwatch::start() {
    isPause = false;
    clock_gettime(CLOCK_MONOTONIC, &startTime); 
}

void Stopwatch::stop() {
    isPause = false;
    elapsedTime = 0;
    clock_gettime(CLOCK_MONOTONIC, &stopTime); 
}
    
void Stopwatch::pause() {
    if (!isPause) {
        isPause = true;
        clock_gettime(CLOCK_MONOTONIC, &stopTime); 
        elapsedTime = getElapsedTime();
    }
}

double Stopwatch::convertToMs(const timespec& time) const {
    return (time.tv_sec * 1000000000 + time.tv_nsec) / 1000000.0;
}
    
double Stopwatch::getStartTime() const {
    return convertToMs(startTime);
}

double Stopwatch::getStopTime() const {
    return convertToMs(stopTime);
}

double Stopwatch::getElapsedTime() const {
    return elapsedTime + convertToMs(stopTime) - convertToMs(startTime);
}