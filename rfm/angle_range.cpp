/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     angle_range.cpp
 * \author   Collin Johnson
 *
 * Definition of angle_range_t for representing a continuous range of angles as a start and extent.
 */

#include "angle_range.h"
#include <iostream>

namespace rfm
{

std::ostream& operator<<(std::ostream& out, const angle_range_t& range)
{
    out << '(' << range.start << ',' << range.extent << ')';
    return out;
}

}   // namespace rfm
