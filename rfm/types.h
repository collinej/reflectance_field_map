/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     types.h
 * \author   Collin Johnson
 *
 * Definition of types that are common to multiple levels of the RFM:
 *
 * Grid types:
 *   - cell_t  : representation of a cell position
 *
 * Topological types:
 *   - AreaType      : enumeration of all possible area types
 *   - TopoDirection : direction of motion when moving along a path or crossing a transition.
 */

#ifndef RFM_TYPES_H
#define RFM_TYPES_H

#include "point.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rfm
{

/**
 * PointHash is a simple hash function for a Point that uses the following hash function:
 *
 *   h(point) = point.x*xSpan + point.y
 */
template <typename T>
struct PointHash
{
  int xSpan = 1000000;

  std::size_t operator()(const Point<T>& point) const { return point.x * xSpan + point.y; }
};

//////////////////// Grid types ////////////////////////
using cell_idx_t = int16_t;
using cell_t = Point<cell_idx_t>;

using CellHash = PointHash<cell_idx_t>;

template <typename T>
using CellToTypeMap = std::unordered_map<cell_t, T, CellHash>;

using CellSet = std::unordered_set<cell_t, CellHash>;

using CellToIntMap = CellToTypeMap<int>;
using CellVector = std::vector<cell_t>;

//////////     Cell classifications     //////////
using cell_type_t = uint8_t;

const cell_type_t kUnknownOccGridCell = 0x01;
const cell_type_t kFreeOccGridCell = 0x02;
const cell_type_t kOccupiedOccGridCell = 0x04;
const cell_type_t kDynamicOccGridCell = 0x08;
const cell_type_t kLimitedVisibilityOccGridCell = 0x40;

const cell_type_t kHitCellTypes = kOccupiedOccGridCell | kDynamicOccGridCell | kLimitedVisibilityOccGridCell;

}   // namespace rfm

#endif   // RFM_TYPES_H
