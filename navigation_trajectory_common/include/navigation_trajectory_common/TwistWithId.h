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

#ifndef TWISTWITHID_H
#define	TWISTWITHID_H

#include <string>

namespace KDL {
    class Twist;
}

/**
 * @brief TwistWithId is an aggregation of KDL::Twist class with a std::string as frame id. 
 * The embedded KDL::Frame will be always heap-allocated.
 */

class TwistWithId {
public:
    /**
     * @brief String identifier of the Twist.
     */
    std::string id;

public:

    /**
     * @brief Default Constructor.
     */
    TwistWithId();

    /**
     * @brief Constructor.
     * 
     * The embedded id is set to the empty string.
     * 
     * @param[in] twist - initial value of KDL::Twist.
     */
    TwistWithId(const KDL::Twist& twist);

    /**
     * @brief Constructor.
     * 
     * The embedded KDL::Twist in initialized by its default constructor.
     * 
     * @param[in] is - initial value of twist identifier.
     */
    TwistWithId(const std::string& id);

    /**
     * @brief Constructor.
     * @param[in] twist - initial value of KDL::Twist.
     * @param[in] is - initial value of twist identifier.
     */
    TwistWithId(const KDL::Twist& twist, const std::string& id);

    /**
     * @brief Copy constructor.
     * @param[in] orig - reference to the original object
     */
    TwistWithId(const TwistWithId& orig);

    /**
     * @brief Operation assignment
     * @param[in] orig - reference to the original object
     */
    const TwistWithId& operator=(const TwistWithId& orig);
    
    /**
     * @brief Destructor.
     */
    virtual ~TwistWithId();

    /**
     * @brief Sets a new value of the KDL::Twist.
     * 
     * The supplied KDL::Twist @p twist is copied and can be delete or modified 
     * without affecting the TwistWithId instance.
     * The @c id member is not modified.
     * 
     * @param[in] twist - new value of KDL::Twist.
     */
    void setTwist(const KDL::Twist& twist);

    /**
     * @brief Get an underlying KDL twist as a const reference or a copy.
     * 
     * Modifications on the TwistWithId, except assignment or setTwist(), will be 
     * reflected by the returned reference.
     * 
     * Assigning or setting a new twist invalidates all references obtained from 
     * const KDL::Twist& getTwist() const.
     * 
     */
    const KDL::Twist& getTwist() const;

    /**
     * @brief Get an underlying KDL trajectory as a reference
     * 
     * Modifications on the TwistWithId, except assignment or setTwist(), will be 
     * reflected by the returned reference.
     * 
     * Assigning or setting a new twist invalidates all references obtained from 
     * const KDL::Twist& getTwist() const.
     * 
     */
    KDL::Twist& getTwist();

private:

    /**
     * @brief Aggregation of the KDL::Twist
     */
    KDL::Twist* twist;

};

#endif	/* TWISTWITHID_H */

