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

#ifndef TRAJECTORYWITHID_H
#define	TRAJECTORYWITHID_H

#include <string>

namespace KDL {
    class Trajectory;
}

/**
 * @brief TrajectoryWithId is an aggregation of KDL::Trajectory class with frame id. 
 */


class TrajectoryWithId {
public:
    /**
     * @brief String identifier of the trajectory.
     */
    std::string id;

    /**
     * @brief Default constructor.
     */
    TrajectoryWithId();

    /**
     * @brief Constructor.
     * @param[in] id - string identifier of the trajectory.
     */
    TrajectoryWithId(const std::string& id);

    /**
     * @brief Constructor
     * @param[in] trajectory - KDL trajectory.
     */
    TrajectoryWithId(const KDL::Trajectory& trajectory);

    /**
     * @brief Constructor.
     * @param[in] trajectory - KDL trajectory.
     * @param[in] id - string identifier of the trajectory.
     */
    TrajectoryWithId(const KDL::Trajectory& trajectory, const std::string& id);

    /**
     * @brief Copy constructor
     * @param[in] orig - reference to the original object
     */
    TrajectoryWithId(const TrajectoryWithId& orig);

    /**
     * @brief Operation assignment
     * @param[in] orig - reference to the original object
     */
    const TrajectoryWithId& operator=(const TrajectoryWithId& orig);
    
    /**
     * @brief Destructor
     */
    virtual ~TrajectoryWithId();

    /**
     * @brief Sets a trajectory
     * @param[in] trajectory - KDL::Trajectory
     */
    void setTrajectory(const KDL::Trajectory& trajectory);

    /**
     * @brief Get an underlying KDL trajectory as a reference
     */
    KDL::Trajectory& getTrajectory();

    /**
     * @brief Gets an underlying KDL trajectory as const reference or a copy
     */
    const KDL::Trajectory& getTrajectory() const;

public:

    /**
     * @brief Aggregated KDL trajectory
     */
    KDL::Trajectory* trajectory;
};

#endif	/* TRAJECTORYWITHID_H */

