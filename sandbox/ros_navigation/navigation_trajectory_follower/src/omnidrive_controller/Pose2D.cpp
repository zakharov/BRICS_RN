/* 
 * File:   Pose2D.cpp
 * Author: alexey
 * 
 * Created on May 26, 2012, 9:53 AM
 */

#include "Pose2D.h"

Pose2D::Pose2D() : x(0), y(0), theta(0) {

}

Pose2D::Pose2D(float x, float y, float theta) : x(x), y(y), theta(theta) {
}

Pose2D::Pose2D(const Pose2D& orig) : x(orig.getX()), y(orig.getY()), theta(orig.getTheta()) {
}

Pose2D::~Pose2D() {
}

const Pose2D& Pose2D::operator=(const Pose2D& orig) {
    this->x = orig.getX();
    this->y = orig.getY();
    this->theta = orig.getTheta();
    return *this;
}

float Pose2D::getX() const {
    return this->x;
}

float Pose2D::getY() const {
    return this->y;
}
    
float Pose2D::getTheta() const {
    return this->theta;
}

Pose2D operator+(const Pose2D& op1, const Pose2D& op2) {
    Pose2D pose(op1.getX() + op2.getX(), op1.getY() + op2.getY(), op1.getTheta() + op2.getTheta());
    
    return pose;
}
