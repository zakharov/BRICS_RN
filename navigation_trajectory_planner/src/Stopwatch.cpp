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

#include "navigation_trajectory_planner/Stopwatch.h"

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