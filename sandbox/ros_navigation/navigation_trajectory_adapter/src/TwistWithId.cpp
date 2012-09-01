/* 
 * File:   TwistWithId.cpp
 * Author: alexey
 * 
 * Created on August 30, 2012, 1:24 PM
 */

#include "navigation_trajectory_adapter/TwistWithId.h"

using namespace KDL;

TwistWithId::TwistWithId() : Twist(), id("") {
}

TwistWithId::TwistWithId(const std::string& id) : Twist(), id(id) {
    
}

TwistWithId::TwistWithId(const TwistWithId& orig) : Twist(orig), id(orig.id) {
    
}

TwistWithId::TwistWithId(const Vector& _vel,const Vector& _rot, const std::string& id) : Twist(_vel, _rot), id(id) {

};

TwistWithId::~TwistWithId() {
}

