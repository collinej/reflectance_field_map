/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     glass_map.cpp
 * \author   Collin Johnson
 *
 * Definition of ReflectanceFieldMap.
 */

#include "reflectance_field_map.h"
#include "rfm_utils.h"
#include "timestamp.h"

// #define DEBUG_REGION_FILTER

namespace rfm
{

const int kNumAngleBins = 1440;
const double kDefaultMaxRange = 3.0;
const int8_t kMaxHit = 127;
const int8_t kMaxMiss = -127;

constexpr cell_intensity_t kDefaultIntensity{0, 0.0f};

inline int round_to_multiple_of_16(int value)
{
    // Integer division will toss out any remainder, then multiply by 16 gets back to nearest multiple of 16
    return (value / 16) * 16;
}


ReflectanceFieldMap::ReflectanceFieldMap()
: numAngleBins_(kNumAngleBins)
, hitGrid_("hit", 0.314, kDefaultMaxRange, numAngleBins_)
, costGrid_(100, 100, 0.314, Point<float>(0.0f, 0.0f))
, typeGrid_(100, 100, 0.314, Point<float>(0.0f, 0.0f))
, counts_(100, 100, 0.314, Point<float>(0.0f, 0.0f))
, intensityGrid_(100, 100, 0.314, Point<float>(0.0f, 0.0f), kDefaultIntensity)
{
    assert(kNumAngleBins == round_to_multiple_of_16(kNumAngleBins));
    setGridOffset();
}


ReflectanceFieldMap::ReflectanceFieldMap(float maxLaserRange,
                   int numAngleBins,
                   int8_t flattenHitThreshold,
                   int8_t flattenMissThreshold,
                   std::size_t gridWidth,
                   std::size_t gridHeight,
                   float gridScale,
                   const Point<float>& globalCenter)
: numAngleBins_(round_to_multiple_of_16(numAngleBins))
, hitThreshold_(std::max(int8_t(1), flattenHitThreshold))
, missThreshold_(std::min(int8_t(-1), flattenMissThreshold))
, hitGrid_(gridWidth, gridHeight, gridScale, globalCenter, maxLaserRange, numAngleBins_, "hit")
, costGrid_(gridWidth, gridHeight, gridScale, globalCenter)
, typeGrid_(gridWidth, gridHeight, gridScale, globalCenter)
, counts_(gridWidth, gridHeight, gridScale, globalCenter)
, intensityGrid_(gridWidth, gridHeight, gridScale, globalCenter, kDefaultIntensity)
{
    std::cout << "Created Glass Map with hit threshold " << static_cast<int>(flattenHitThreshold)
              << " and miss threshold " << static_cast<int>(flattenMissThreshold) << '\n';

    setGridOffset();
}


void ReflectanceFieldMap::setGridSizeInCells(std::size_t width, std::size_t height)
{
    costGrid_.setGridSizeInCells(width, height);
    typeGrid_.setGridSizeInCells(width, height);
    counts_.setGridSizeInCells(width, height);
    intensityGrid_.setGridSizeInCells(width, height);

    setGridOffset();
}


void ReflectanceFieldMap::setMetersPerCell(float gridScale)
{
    costGrid_.setMetersPerCell(gridScale);
    typeGrid_.setMetersPerCell(gridScale);
    counts_.setMetersPerCell(gridScale);
    intensityGrid_.setMetersPerCell(gridScale);
}


void ReflectanceFieldMap::setBottomLeft(const Point<float>& bottomLeft)
{
    costGrid_.setBottomLeft(bottomLeft);
    typeGrid_.setBottomLeft(bottomLeft);
    counts_.setBottomLeft(bottomLeft);
    intensityGrid_.setBottomLeft(bottomLeft);

    setGridOffset();
}


void ReflectanceFieldMap::changeBoundary(const Rectangle<float>& newBoundary)
{
    costGrid_.changeBoundary(newBoundary);
    typeGrid_.changeBoundary(newBoundary);
    counts_.changeBoundary(newBoundary);
    intensityGrid_.changeBoundary(newBoundary, kDefaultIntensity);
    hitGrid_.changeBoundary(newBoundary);
    setGridOffset();
}


bool ReflectanceFieldMap::needToRecenterActiveRegion(const Point<float>& position) const
{
    return hitGrid_.needToRecenterActiveRegion(position);
}


bool ReflectanceFieldMap::recenterActiveRegion(const Point<float>& position)
{
    return hitGrid_.recenterActiveRegion(position);
}


void ReflectanceFieldMap::reset()
{
    costGrid_.reset();
    typeGrid_.reset();
    counts_.reset();
    hitGrid_.reset();
    resetIntensity();
}


void ReflectanceFieldMap::resetIntensity()
{
    intensityGrid_.reset(kDefaultIntensity);
}


bool ReflectanceFieldMap::haveOverlappingAngles(int x1, int y1, int x2, int y2) const
{
    // When searching, looking for matches in a slightly wider region than cell-to-cell +/-1 cell will make the match
    // more robust to localization noise
    auto firstIt = beginBin(x1, y1);
    auto firstEnd = endBin(x1, y1);
    auto secondIt = beginBin(x2, y2);
    auto secondEnd = endBin(x2, y2);

    if ((isHit(*firstIt) && isHit(*(secondEnd - 1))) || (isHit(*(firstEnd - 1)) && isHit(*secondIt))) {
        return true;
    }

    for (; (firstIt != firstEnd) && (secondIt != secondEnd); ++firstIt, ++secondIt) {
        if (isHit(*firstIt) && isHit(*secondIt)) {
            return true;
        }
    }

    firstIt = beginBin(x1, y1);
    firstEnd = endBin(x1, y1);
    secondIt = beginBin(x2, y2);
    secondEnd = endBin(x2, y2);
    ++secondIt;

    for (; (firstIt != firstEnd) && (secondIt != secondEnd); ++firstIt, ++secondIt) {
        if (isHit(*firstIt) && isHit(*secondIt)) {
            return true;
        }
    }

    firstIt = beginBin(x1, y1);
    firstEnd = endBin(x1, y1);
    secondIt = beginBin(x2, y2);
    secondEnd = endBin(x2, y2);
    ++firstIt;

    for (; (firstIt != firstEnd) && (secondIt != secondEnd); ++firstIt, ++secondIt) {
        if (isHit(*firstIt) && isHit(*secondIt)) {
            return true;
        }
    }

    return false;
}


int ReflectanceFieldMap::angleToBin(double angle) const
{
    assert(angle >= 0.0);
    if (angle >= 2 * M_PI) {
        angle -= 2 * M_PI;
    }
    assert(angle < 2 * M_PI);

    constexpr double kOneOverTwoPi = 1.0 / (2 * M_PI);
    return angle * numAngleBins_ * kOneOverTwoPi;
}


double ReflectanceFieldMap::binToAngle(int bin) const
{
    assert(bin >= 0);
    assert(bin < static_cast<int>(numAngleBins_));

    const double kBinAngleWidth = 2 * M_PI / numAngleBins_;
    return bin * kBinAngleWidth;
}


bool ReflectanceFieldMap::observedCell(int x, int y, int angleBin, uint16_t intensity, float range)
{
    auto flatCell = glassToFlatMap(x, y);
    if (intensityGrid_.isCellInGrid(flatCell)) {
        auto& maxIntensity = intensityGrid_(flatCell.x, flatCell.y);
        if (maxIntensity.intensity < intensity) {
            maxIntensity.intensity = intensity;
            maxIntensity.range = range;
        }
    }

    haveIntensity_ |= intensity > 0;   // any intensity measurement clamps to using intensity for the map

    if (hitGrid_.isCellInActiveRegion(x, y) && (hitGrid_(x, y, angleBin) < kMaxHit)) {
        // Once a miss, always a miss. Otherwise, if not below that threshold yet, see what we can see.
        if (hitGrid_(x, y, angleBin) > kMaxMiss) {
            ++hitGrid_(x, y, angleBin);
            if (counts_.isCellInGrid(flatCell)) {
                // If the threshold is hit then we've switched to occupied
                if (hitGrid_(x, y, angleBin) == hitThreshold_) {
                    counts_(flatCell.x, flatCell.y).hits++;
                    return true;
                }
                // Had been a miss, but now has switched to unknown
                else if (hitGrid_(x, y, angleBin) == (missThreshold_ + 1)) {
                    counts_(flatCell.x, flatCell.y).misses--;
                }
            }
        }
    }

    return false;
}


bool ReflectanceFieldMap::missedCell(int x, int y, int angleBin)
{
    if (hitGrid_.isCellInActiveRegion(x, y) && (hitGrid_(x, y, angleBin) > kMaxMiss)) {
        --hitGrid_(x, y, angleBin);
        // If the value slips one below then it has changed from occupied to free
        auto flatCell = glassToFlatMap(x, y);
        if (counts_.isCellInGrid(flatCell)) {
            if (hitGrid_(x, y, angleBin) == (hitThreshold_ - 1)) {
                counts_(flatCell.x, flatCell.y).hits--;
                return true;
            } else if (hitGrid_(x, y, angleBin) == missThreshold_) {
                counts_(flatCell.x, flatCell.y).misses++;
            }
        }
    }

    return false;
}


void ReflectanceFieldMap::reflectedCell(int x, int y, int angleBin)
{
    if (hitGrid_.isCellInActiveRegion(x, y) && (hitGrid_(x, y, angleBin) > kMaxMiss)) {
        // If the value was a hit and if we're resetting it, then reduce the hit count
        auto flatCell = glassToFlatMap(x, y);
        if (counts_.isCellInGrid(flatCell)) {
            if (hitGrid_(x, y, angleBin) >= hitThreshold_) {
                counts_(flatCell.x, flatCell.y).hits--;
            } else if (hitGrid_(x, y, angleBin) <= missThreshold_) {
                counts_(flatCell.x, flatCell.y).misses--;
            }
        }
        hitGrid_(x, y, angleBin) = 0;
    }
}


void ReflectanceFieldMap::flattenActiveRegion(int minVisibleRange)
{
    // Get the active region in which changes might have occurred and flatten just those
    flattenRegion(active_region_in_flat_map(*this), minVisibleRange);
}


void ReflectanceFieldMap::flattenFullMap(int minVisibleRange)
{
    Rectangle<int> fullMapRegion(Point<int>(0, 0),
                                       Point<int>(counts_.getWidthInCells(), counts_.getHeightInCells()));
    flattenRegion(fullMapRegion, minVisibleRange);
}


void ReflectanceFieldMap::printSize() const
{
    const auto numCells = costGrid_.getHeightInCells() * costGrid_.getWidthInCells();

    const auto inMemory = hitGrid_.inMemoryFootprint();
    const auto onDisk = hitGrid_.onDiskFootprint();
    const auto flattened = numCells * sizeof(cell_type_t) * 2;
    const auto hitCount = numCells * sizeof(int16_t);
    const auto missCount = numCells * sizeof(int16_t);
    const auto intensity = numCells * sizeof(uint16_t);

    const auto totalSize = inMemory + flattened + hitCount + missCount + intensity;

    std::cout << "//////////     GLASS MAP SIZE     //////////\n"
              << "Hit Grid (Memory): " << inMemory << '\n'
              << "Hit Grid (Disk): " << onDisk << '\n'
              << "Flattened: " << flattened << '\n'
              << "Hit/Miss: " << hitCount << '\n'
              << "Intensity: " << intensity << '\n'
              << "Total (bytes): " << totalSize << '\n'
              << "Total (MB): " << totalSize / static_cast<double>(1 << 20) << '\n';
}


void ReflectanceFieldMap::setGridOffset()
{
    // The glass map and flattened grid likely don't have the exact same boundary, since one
    // snaps to tiles not cells. This offset will align the flattened and glass cells for storage.
    auto glassToFlatOffset = hitGrid_.getBottomLeft() - costGrid_.getBottomLeft();
    glassToFlatCellOffset_.x = std::lrint(glassToFlatOffset.x * costGrid_.cellsPerMeter());
    glassToFlatCellOffset_.y = std::lrint(glassToFlatOffset.y * costGrid_.cellsPerMeter());
}


Point<int> ReflectanceFieldMap::glassToFlatMap(int x, int y)
{
    return Point<int>(x + glassToFlatCellOffset_.x, y + glassToFlatCellOffset_.y);
}


Point<int> ReflectanceFieldMap::glassToFlatMap(Point<int> glass)
{
    return glassToFlatMap(glass.x, glass.y);
}


Point<int> ReflectanceFieldMap::flatToRFM(int x, int y)
{
    return Point<int>(x - glassToFlatCellOffset_.x, y - glassToFlatCellOffset_.y);
}


Point<int> ReflectanceFieldMap::flatToRFM(Point<int> flat)
{
    return flatToRFM(flat.x, flat.y);
}


void ReflectanceFieldMap::flattenRegion(Rectangle<int> boundaryInCells, int minVisibleRange)
{
    minVisibleRange = std::max(minVisibleRange, 1);   // must be visible from at least one angle

    int64_t computationStartTimeUs = system_time_us();

    constexpr uint8_t kUnknownCost = 127;
    constexpr int kHitIncrement = 128 / 5;

    for (int y = boundaryInCells.bottomLeft.y; y < boundaryInCells.topRight.y; ++y) {
        for (int x = boundaryInCells.bottomLeft.x; x < boundaryInCells.topRight.x; ++x) {
            // If there's a hit, then mark full odds.
            // If there's a miss, then mark no odds.
            // Otherwise, leave it as unknown.
            uint8_t cost = kUnknownCost;

            if (counts_(x, y).hits >= minVisibleRange) {
                cost = (counts_(x, y).hits * kHitIncrement > 128) ? 255 : kUnknownCost + counts_(x, y).hits * kHitIncrement;
            } else if (counts_(x, y).misses >= minVisibleRange) {
                cost = 0;
            }

            uint8_t prevCost = costGrid_(x, y);
            // If the cost is unchanged, then no need to do anything
            if (prevCost != cost) {
                cell_type_t type = typeGrid_(x, y);
                costGrid_(x, y) = cost;

                // Keep the type for occupied cells the same so as not to erase the results of the dynamic object filter
                if (cost > kUnknownCost) {
                    type =
                      (type & kHitCellTypes) ? type : kOccupiedOccGridCell;   // if type not set, just go with occupied
                    typeGrid_(x, y) = type;
                }
                // If the cost has gone to unknown, then set to unknown. Otherwise, removed reflections get marked dynamic
                else if (cost == kUnknownCost) {
                    typeGrid_(x, y) = kUnknownOccGridCell;
                } else   // if(cost < kUnknownCost)
                {
                    typeGrid_(x, y) = kFreeOccGridCell;
                }

                assert((cost <= kUnknownCost) || (typeGrid_(x, y) & kHitCellTypes));
            }
        }
    }

    int64_t computationTimeUs = system_time_us() - computationStartTimeUs;
    totalFlatteningTime_ += computationTimeUs;
    maxFlatteningTime_ = std::max(maxFlatteningTime_, computationTimeUs);
    ++numFlattenings_;
    std::cout << "Flattening time: " << computationTimeUs << "us Mean: " << totalFlatteningTime_ / numFlattenings_
        << "us Max: " << maxFlatteningTime_ << "us\n";
}


void ReflectanceFieldMap::flushCacheToDisk() const
{
    hitGrid_.flush();
}

}   // namespace rfm
