/*
 * File:   VelocityRamp.cpp
 * Author: alexey
 *
 * Created on May 26, 2012, 9:50 AM
 */

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
