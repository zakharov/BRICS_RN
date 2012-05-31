/* 
 * File:   Twist2D.cpp
 * Author: alexey
 * 
 * Created on May 26, 2012, 9:53 AM
 */

#include "Twist2D.h"

Twist2D::Twist2D() : x(0), y(0), theta(0) {

}

Twist2D::Twist2D(float x, float y, float theta) : x(x), y(y), theta(theta) {
}

Twist2D::Twist2D(const Twist2D& orig) : x(orig.getX()), y(orig.getY()), theta(orig.getTheta()) {
}

Twist2D::~Twist2D() {
}

const Twist2D& Twist2D::operator=(const Twist2D& orig) {
    this->x = orig.getX();
    this->y = orig.getY();
    this->theta = orig.getTheta();
    return *this;
}


float Twist2D::getX() const {
    return this->x;
}

void  Twist2D::setX(float x) {
    
}

float Twist2D::getY() const {
    return this->y;
}

void  Twist2D::setY(float y) {
    
}
    
float Twist2D::getTheta() const {
    return this->theta;
}

void  Twist2D::setTheta(float theta) {
    this->theta = theta;
}

Twist2D operator+(const Twist2D& op1, const Twist2D& op2) {
    Twist2D twist(op1.getX() + op2.getX(), op1.getY() + op2.getY(), op1.getTheta() + op2.getTheta());
    
    return twist;
}