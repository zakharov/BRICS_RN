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

#ifndef FRAMEWITHID_H
#define	FRAMEWITHID_H

#include <kdl/frames.hpp>
#include <string>

class FrameWithId : public KDL::Frame {
public:
    std::string id;

public:

    FrameWithId(const KDL::Rotation& R, const KDL::Vector& V, const std::string& id);

    FrameWithId(const KDL::Vector& V, const std::string& id);

    FrameWithId(const KDL::Rotation& R, const std::string& id);

    FrameWithId(const std::string& id);

    FrameWithId();

    FrameWithId(const FrameWithId& arg);
    
    KDL::Frame getFrame();

    virtual ~FrameWithId();
};


#endif	/* FRAMEWITHID_H */

