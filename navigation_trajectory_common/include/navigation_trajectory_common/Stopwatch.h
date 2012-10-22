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

#ifndef STOPWATCH_H
#define	STOPWATCH_H

#include <ctime>

/**
 * @brief Implementation of a stopwatch for profiling of the algorithms.
 */

class Stopwatch {
public:
    /**
     * @brief Default constructor.
     */
    Stopwatch();

    /**
     * @brief Copy constructor.
     */
    Stopwatch(const Stopwatch& orig);

    /**
     * @brief Destructor.
     */
    virtual ~Stopwatch();

    /**
     * @brief Start a stop watch
     */
    void start();

    /**
     * @brief Stop and reset a stop watch
     */
    void stop();

    /**
     * @brief Pause a stopwatch without resetting
     */
    void pause();

    /**
     * @brief Return a start time in milliseconds
     */
    double getStartTime() const;

    /**
     * @brief Return a top time in milliseconds
     */
    double getStopTime() const;

    /**
     * @brief return elapsed time in milliseconds 
     */
    double getElapsedTime() const;

private:

    double convertToMs(const timespec& time) const;
    timespec startTime;
    timespec stopTime;
    double elapsedTime;
    bool isPause;
};

#endif	/* STOPWATCH_H */

