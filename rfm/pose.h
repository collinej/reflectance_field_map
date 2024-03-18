/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     pose.h
 * \author   Collin Johnson
 *
 * Declaration of pose_t, pose_distribution_t, and associated helper functions. The pose type
 * used by all of the Vulcan code.
 */

#ifndef RFM_POSE_H
#define RFM_POSE_H

#include "angle_functions.h"
#include <cstdint>
#include <iosfwd>
#include <numbers>

namespace rfm
{

struct pose_distribution_t;

/**
 * pose_t represents the robot pose in Cartesian coordinates in the current global
 * frame of reference.
 */
struct pose_t
{
    int64_t timestamp;

    float x;
    float y;
    float theta;

    pose_t() : timestamp(0), x(0), y(0), theta(0) { }

    pose_t(float x, float y, float theta) : timestamp(0), x(x), y(y), theta(theta) { }

    explicit pose_t(const Point<float>& position, float theta = 0.0f) : timestamp(0), x(position.x), y(position.y), theta(theta)
    {
    }

    pose_t(int64_t timestamp, float x, float y, float theta) : timestamp(timestamp), x(x), y(y), theta(theta) { }

    // Some helper functions for converting reference frames, etc.
    /**
     * toPoint converts the pose to a point. Simply chop off the theta.
     */
    Point<float> toPoint() const { return Point<float>(x, y); }
};

// Various operator overloads
bool operator==(const pose_t& lhs, const pose_t& rhs);
bool operator!=(const pose_t& lhs, const pose_t& rhs);
std::ostream& operator<<(std::ostream& out, const pose_t& pose);

// Serialization support -- both types are just PODs
template <class Archive>
void serialize(Archive& ar, pose_t& pose)
{
    ar(pose.timestamp, pose.x, pose.y, pose.theta);
}

}   // namespace rfm

#endif   // RFM_POSE_H
