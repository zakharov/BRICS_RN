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

#include "navigation_trajectory_common/TwistWithId.h"

#include <kdl/frames.hpp>

using namespace KDL;

TwistWithId::TwistWithId() {
    id = "";
    twist = new Twist();
}

TwistWithId::TwistWithId(const std::string& id) {
    this->id = id;
    this->twist = new Twist(); 
}

TwistWithId::TwistWithId(const KDL::Twist& twist) {
    this->id = "";
    this->twist = new Twist(twist); 
}

TwistWithId::TwistWithId(const KDL::Twist& twist, const std::string& id) {
    this->id = id;
    this->twist = new Twist(twist);
}

TwistWithId::TwistWithId(const TwistWithId& orig) {
    id = orig.id;
    twist = new Twist(orig.getTwist());
}

const TwistWithId& TwistWithId::operator=(const TwistWithId& orig) {
    id = orig.id;
    delete twist;
    twist = new Twist(orig.getTwist());
    
    return *this;
}

void TwistWithId::setTwist(const KDL::Twist& orig) {
    delete this->twist;
    this->twist = new Twist(orig);
}

const KDL::Twist& TwistWithId::getTwist() const {
    return *twist;
}
    
KDL::Twist& TwistWithId::getTwist() {
    return *twist;
}

TwistWithId::~TwistWithId() {
    delete twist;
}

