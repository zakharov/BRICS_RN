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

#include "VelocityRamp.h"
#include <cmath>

VelocityRamp::VelocityRamp() {
    initialVelocity = 0.0; // m/s
    actualVelocity = 0.0;
    finalVelocity = 0.0;

    minNegativeVelocityLimit = -0.1; // m/s
    minPositiveVelocityLimit = 0.1;

    maxNegativeVelocityLimit = -1.0;
    maxPositiveVelocityLimit = 1.0;

    acceleration = 0.5; // m/s^2
    deceleration = 0.5;

    totalDistance = 0.0; // m
    actualDistance = 0.0;
}

VelocityRamp::VelocityRamp(const VelocityRamp& orig) {
    initialVelocity = orig.initialVelocity; // m/s
    actualVelocity = orig.actualVelocity;
    finalVelocity = orig.finalVelocity;

    minNegativeVelocityLimit = orig.minNegativeVelocityLimit; // m/s
    minPositiveVelocityLimit = orig.minPositiveVelocityLimit;

    maxNegativeVelocityLimit = orig.maxNegativeVelocityLimit;
    maxPositiveVelocityLimit = orig.maxPositiveVelocityLimit;

    acceleration = orig.acceleration; // m/s^2
    deceleration = orig.deceleration;

    totalDistance = orig.totalDistance; // m
    actualDistance = orig.actualDistance;
}

VelocityRamp::~VelocityRamp() {
}

void VelocityRamp::setTotalDistance(float totalDistance) {
    this->totalDistance = totalDistance;
}

float VelocityRamp::getTotalDistance() {
    return totalDistance;
}

void VelocityRamp::setActualDistance(float actualDistance) {
    this->actualDistance = actualDistance;
}

float VelocityRamp::getActualDistance() {
    return actualDistance;
}

void VelocityRamp::setActualVelocity(float actualVelocity) {
    this->actualVelocity = actualVelocity;
}

float VelocityRamp::getActualVelocity() {
    return actualVelocity;
}

void VelocityRamp::setAcceleration(float acceleration) {
    this->acceleration = 1.0 / acceleration;
}

float VelocityRamp::getAcceleration() {
    return acceleration;
}

void VelocityRamp::setDeceleration(float deceleration) {
    this->deceleration = deceleration;
}

float VelocityRamp::getDeceleration() {
    return deceleration;
}

void VelocityRamp::setMaxVelocityLimits(float negative, float positive) {
    this->maxNegativeVelocityLimit = negative;
    this->maxPositiveVelocityLimit = positive;
}

void VelocityRamp::getMaxVelocityLimits(float& negative, float& positive) {
    negative = this->maxNegativeVelocityLimit;
    positive = this->maxPositiveVelocityLimit;
}

void VelocityRamp::setMinVelocityLimits(float negative, float positive) {
    this->minNegativeVelocityLimit = negative;
    this->minPositiveVelocityLimit = positive;
}

void VelocityRamp::getMinVelocityLimits(float& negative, float& positive) {
    negative = this->minNegativeVelocityLimit;
    positive = this->minPositiveVelocityLimit;
}

void VelocityRamp::setInitialVelocity(float initialVelocity) {
    this->initialVelocity = initialVelocity;
}

float VelocityRamp::getInitialVelocity() {
    return initialVelocity;
}

void VelocityRamp::setFinalVelocity(float finalVelocity) {
    this->finalVelocity = finalVelocity;
}

float VelocityRamp::getFinalVelocity() {
    return finalVelocity;
}

float VelocityRamp::computeVelocity(float actualDistance) {

    this->actualDistance = actualDistance;


    if (totalDistance == 0) {
        actualVelocity = 0;
        return actualVelocity;
    }


    float velocity = 0;
    // float phi = acceleration * M_PI / 2.0;
    float halfTotalDistance = (totalDistance) / 2.0;

    if (halfTotalDistance != 0.0) {
        float velocityLimitAcceleration = halfTotalDistance * this->acceleration;
        float velocityLimitDeceleration = halfTotalDistance * this->deceleration;

        if (fabs(actualDistance) <= fabs(halfTotalDistance)) {
            velocity = ((velocityLimitAcceleration - initialVelocity) / halfTotalDistance) * actualDistance + initialVelocity;
        } else if (fabs(actualDistance) > fabs(halfTotalDistance)) {
            velocity = ((finalVelocity - velocityLimitDeceleration/*velocityLimit_*/) /
                    halfTotalDistance) * actualDistance -
                    finalVelocity + 2 * /*velocityLimit_*/velocityLimitDeceleration;
        }

        if ((actualDistance < 0) && (halfTotalDistance > 0)) {

            //velocity = tan(acceleration) * actualDistance - 2 * initialVelocity;
            velocity = +0.1;
            //      initialTwist = actualTwist;
            //        ROS_INFO("HOLY CRAP! braking=%f initialVel=%f actualDistance=%f", velocity, initialVelocity, actualDistance);
        }


        if ((actualDistance > 0) && (halfTotalDistance < 0)) {
            //  velocity = tan(acceleration) * actualDistance - 2 * initialVelocity;
            velocity = -0.1;
            //        initialTwist = actualTwist;
            //      ROS_INFO("HOLY CRAP! braking=%f initialVel=%f actualDistance=%f", velocity, initialVelocity, actualDistance);
        }

        if ((velocity > minNegativeVelocityLimit) && (velocity < minPositiveVelocityLimit)) {
            if (halfTotalDistance > 0)
                velocity = minPositiveVelocityLimit;
            else if (halfTotalDistance < 0)
                velocity = minNegativeVelocityLimit;
        }

        if (velocity > maxPositiveVelocityLimit)
            velocity = maxPositiveVelocityLimit;

        if (velocity < maxNegativeVelocityLimit)
            velocity = maxNegativeVelocityLimit;
    }

    actualVelocity = velocity;

    return actualVelocity;
}
