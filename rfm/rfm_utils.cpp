/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     glass_map_utils.cpp
 * \author   Collin Johnson
 *
 * Definition of utility functions for interacting with the RFM:
 *
 *   - is_cell_in_region
 *   - active_region_in_flat_map
 */

#include "rfm_utils.h"

namespace rfm
{

bool is_cell_in_region(Point<int> cell, const Rectangle<int>& region)
{
    return (cell.x >= region.bottomLeft.x) && (cell.x < region.topRight.x) && (cell.y >= region.bottomLeft.y)
      && (cell.y < region.topRight.y);
}


Rectangle<int> active_region_in_flat_map(const ReflectanceFieldMap& map)
{
    auto activeRegion = map.activeRegionInCells();
    auto offset = map.glassToFlatMapOffset();
    Rectangle<int> flatActiveRegion(activeRegion.bottomLeft + offset, activeRegion.topRight + offset);
    flatActiveRegion.bottomLeft.x =
      std::clamp(flatActiveRegion.bottomLeft.x, 0, static_cast<int>(map.flattenedMapCosts().getWidthInCells()));
    flatActiveRegion.bottomLeft.y =
      std::clamp(flatActiveRegion.bottomLeft.y, 0, static_cast<int>(map.flattenedMapCosts().getHeightInCells()));
    flatActiveRegion.topRight.x = std::clamp(flatActiveRegion.topRight.x,
                                        flatActiveRegion.bottomLeft.x,
                                        static_cast<int>(map.flattenedMapCosts().getWidthInCells()));
    flatActiveRegion.topRight.y = std::clamp(flatActiveRegion.topRight.y,
                                        flatActiveRegion.bottomLeft.y,
                                        static_cast<int>(map.flattenedMapCosts().getHeightInCells()));

    return flatActiveRegion;
}


angle_range_t angle_bins_to_angle_range(int x, int y, const ReflectanceFieldMap& map)
{
    if (!map.isCellInActiveRegion(x, y)) {
        return angle_range_t();
    }

    double binAngleWidth = 2 * M_PI / map.numAngleBins();
    auto bin = map.beginBin(x, y);

    angle_range_t range;
    bool haveHit = false;

    for (int n = 0; n < map.numAngleBins(); ++n, ++bin) {
        if (*bin > 0) {
            double angle = map.binToAngle(n);

            // If the last bin wasn't a hit, then this is the start of a new angle range
            if (!haveHit) {
                range = angle_range_t(angle);
                range.expand(angle + binAngleWidth);
            }
            // Otherwise, we are continuing to expand an existing range
            else {
                // Add the whole bin
                range.expand(angle + binAngleWidth);
            }

            haveHit = true;
        }
    }

    return range;
}


double glass_cell_normal(int x, int y, const ReflectanceFieldMap& map)
{
    // Make sure the cell is active in order to process the normal
    if (!map.isCellInActiveRegion(x, y)) {
        return 0.0;
    }

    return weighted_angle_sum(map.beginBin(x, y), map.endBin(x, y));
}

}   // namespace rfm
