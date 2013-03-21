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

#ifndef PATHITERATOR_H
#define	PATHITERATOR_H

#include <vector>

class FrameWithId;
class IPathInterpolation;

/**
 * @brief A path iterator class.
 */

class PathIterator {
public:

    /**
     * @brief A path iterator constructor
     * 
     * This does @b not create a copy of the path. It is the users responsibility, to 
     * keep the path object @p path valid until the path iterator itself is destroyed.
     * 
     * The iterator is initialized such that it returns the first element of the path 
     * at the first call to next().
     * 
     * @param[in] path - an input path to iterate.
     */
    PathIterator(const std::vector <FrameWithId>& path);

    /**
     * @brief Copy constructor.
     * 
     * This creates a shared copy.  The new path iterator shares the embedded path 
     * with the @p orig path iterator and any other copy made from it. Modifying the 
     * original path will affect all iterators referencing that path.
     * 
     * The position with in the path is of course @b not shared.
     */
    PathIterator(const PathIterator& orig);

    /**
     * @brief Destructor
     */
    virtual ~PathIterator();

    /**
     * @brief return true if iterator has a next point, otherwise false.
     */
    bool hasNext() const;

    /**
     * @brief return the current point and move the cursor to the following point, if one exists.
     */
    const FrameWithId& next() const;

private:
    /**
     * @brief An input path.
     */
    const std::vector <FrameWithId>& path;

    /**
     * @brief Actual position of the cursor.
     * 
     * This is zero based, value 0 corresponds to the first element in the path.
     */
    mutable size_t cursor;

    /**
     * @brief Move cursor to the next point and return it.
     */
    static const FrameWithId& dummy;
};

#endif	/* PATHITERATOR_H */

