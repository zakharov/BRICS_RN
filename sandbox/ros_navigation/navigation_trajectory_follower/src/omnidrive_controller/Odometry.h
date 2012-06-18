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

#ifndef ODOMETRY_H
#define	ODOMETRY_H

#include "Twist2D.h"
#include "Pose2D.h"

class Odometry {
public:
    Odometry(); // creates odometry with 0,0,0 pose and 0,0,0 twist
    Odometry(const Pose2D& pose, const Twist2D& twist);
    Odometry(const Pose2D& pose);
    Odometry(const Twist2D& twist);
    Odometry(const Odometry& orig);
  
    const Odometry& operator=(const Odometry& orig);
    
    const Pose2D& getPose2D() const;
    void setPose2D(const Pose2D& twist);
    
    const Twist2D& getTwist2D() const;
    void setTwist2D(const Twist2D& twist);

    virtual ~Odometry();
    
private:
    
    Pose2D pose;
    Twist2D twist;
};

#endif	/* ODOMETRY_H */

