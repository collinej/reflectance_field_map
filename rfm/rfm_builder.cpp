/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     glass_map_builder.cpp
 * \author   Collin Johnson
 *
 * Definition of RFMBuilder adapted from Paul's reference implementation of the glass mapping.
 */

#include "rfm_builder.h"

#include "algorithm_ext.h"
#include "cell_grid.h"
#include "point_cloud_2d.h"
#include "pose.h"
#include "ray_tracing.h"
#include "rfm_dynamic_filter.h"
#include "rfm_utils.h"
#include "timestamp.h"
#include "types.h"
#include <array>
#include <stack>

namespace rfm
{

RFMBuilder::RFMBuilder(const rfm_map_builder_params_t& params, const Point<float>& center)
: map_(params.maxLaserRange,
       params.numAngleBins,
       params.hitThreshold,
       params.missThreshold,
       params.width,
       params.height,
       params.scale,
       center)
, scanHits_(params.numAngleBins)
, angleBinPlusPi_(params.numAngleBins)
, kMaxLaserRange_(params.maxLaserRange)
, kShouldAntiAliasHits_(params.shouldAntiAlias)
, kShouldFilterDynamic_(params.shouldFilterDynamic)
, kShouldFilterReflections_(params.shouldFilterReflections)
, dynamicFilter_(std::make_unique<RFMDynamicFilter>())
{
    assert(params.numAngleBins > 0);

    map_.reset();   // clear out any previous glass information when initializing

    // Store the bin on the opposite side (+180 degrees)
    for (int n = 0; n < params.numAngleBins; ++n) {
        angleBinPlusPi_[n] = (n + (params.numAngleBins / 2)) % params.numAngleBins;
    }

    double radPerBin = 2.0 * M_PI / params.numAngleBins;
    kMinVisibleBins_ = std::max(lrint(params.minVisibleOccupiedRange / radPerBin), 1l);
//    kMinHighlyVisibleBins_ = std::max(lrint(params.minHighlyVisibleRange / radPerBin), 1l);
    kMinHighlyVisibleBins_ = params.minHighlyVisibleRange;

    std::cout << "INFO: RFMBuilder: Min visible bins:" << kMinVisibleBins_
              << " Min highly-visible bins:" << kMinHighlyVisibleBins_ << '\n';
}


RFMBuilder::~RFMBuilder() = default;


void RFMBuilder::filterDynamicObjects()
{
    dynamicFilter_->filterDynamicObjectsFromActiveRegion(map_, kMinHighlyVisibleBins_, kMinGlassIntensity_);
}


ReflectedLaserScan RFMBuilder::detectReflections(const PointCloud2D& scan)
{
    int64_t startTimeUs = system_time_us();

    CellVector rayCells;
    std::vector<reflected_ray_t> reflectedRays;
    const auto& costMap = map_.flattenedMapCosts();

    auto activeRegion = map_.activeRegionInMeters();

    CellToTypeMap<angle_range_t> ranges;

    PointCloud2D activeScans;

    for (auto& ray : scan) {
        // Ignore invalid rays
        if (ray.range < 0.0f) {
            continue;
        }

        // Is either end in the active region? If not, then there's no need to process this scan
        if (!activeRegion.contains(ray.origin) && !activeRegion.contains(ray.endpoint)) {
            continue;
        }

        activeScans.addRay(ray);

        rayCells.clear();
        // Find the cells along the ray that will be checked for reflections
        float range = std::min(ray.range, kMaxLaserRange_);   // only consider up to the maximum range
        Point<double> rangeLimitedEnd(ray.origin.x + (range * std::cos(ray.angle)),
                                      ray.origin.y + (range * std::sin(ray.angle)));
        find_cells_along_line(Line<double>(global_point_to_grid_point(ray.origin, map_),
                                                  global_point_to_grid_point(rangeLimitedEnd, map_)),
                                     map_,
                                     std::back_inserter(rayCells));

        reflected_ray_t reflected;
        reflected.origin = ray.origin;
        reflected.range = ray.range;
        reflected.angle = ray.angle;
        reflected.isReflected = false;
        reflected.distToReflection = ray.range;

        int angleBin = map_.angleToBin(wrap_to_2pi(ray.angle));
        bool confirmedReflection = false;

        // Search along the cells the ray passes through. If a cell is occupied, then it is the potential source of a
        // reflection. Until a free cell is encountered, we keep update the reflection hypothesis because the walls in
        // the map aren't perfectly one cell thick and thus, we don't want false positives where it's just a two-cell
        // thick bit of wall.
        for (auto cell : rayCells) {
            // The two grids don't have the same reference frame, so cajole them together here.
            auto global = grid_point_to_global_point(cell, map_);
            auto flatCell = global_point_to_grid_cell_round(global, costMap);
            auto cellType = costMap.getValue(flatCell.x, flatCell.y);

            if (!confirmedReflection   // once a reflection is confirmed, all hits thereafter are reflections
                && (cellType & (kLimitedVisibilityOccGridCell | kOccupiedOccGridCell)))   // if we encounter an occupied cell,
                                                                             // it might be the source of a reflection
            {
                auto angleRangeIt = ranges.find(cell);
                if (angleRangeIt == ranges.end()) {
                    auto angleRange = angle_bins_to_angle_range(cell.x, cell.y, map_);
                    auto result = ranges.insert(std::make_pair(cell, angleRange));
                    angleRangeIt = result.first;
                }

                // A reflection has occurred the ray passes through an occupied cell outside its visible range

                if (!angleRangeIt->second.contains(ray.angle)) {
                    reflected.isReflected = true;
                    reflected.distToReflection = distance_between_points(reflected.origin, global);
                }
            }
            // If there was a reflection detected and now have hit a free cell, then we definitely passed through
            // an object
            else if (reflected.isReflected && (cellType & (kFreeOccGridCell | kUnknownOccGridCell))) {
                // For all reflections, mark this angle bin as being the result of a reflection
                confirmedReflection = true;
            }

            if (confirmedReflection && kShouldFilterReflections_) {
                map_.reflectedCell(cell.x, cell.y, angleBin);
            }
        }

//        reflectedRays.push_back(reflected);
    }

    allMappingScans_ = activeScans;

    int64_t elapsedTimeUs = system_time_us() - startTimeUs;
    ++numReflectionDetections_;
    totalReflectionTimeUs_ += elapsedTimeUs;
    maxReflectionTimeUs_ = std::max(elapsedTimeUs, maxReflectionTimeUs_);

    std::cout << "Reflection time: " << elapsedTimeUs << "us Mean: " << (totalReflectionTimeUs_ / numReflectionDetections_)
              << "us Max: " << maxReflectionTimeUs_ << "us Num rays: " << activeScans.size()
              << " Time per ray: " << (elapsedTimeUs / static_cast<double>(activeScans.size())) << "us\n";


    return ReflectedLaserScan(0, scan.timestamp(), reflectedRays);
}


void RFMBuilder::reset()
{
    allMappingScans_.clear();
}


void RFMBuilder::changeMapBoundary(const Rectangle<float>& boundary)
{
    map_.changeBoundary(boundary);
}


void RFMBuilder::updateMap(const PointCloud2D& scan, const Point<float>& scanOrigin)
{
    int64_t startTimeUs = system_time_us();

    // Keep the map centered around the robot
    if (map_.needToRecenterActiveRegion(scanOrigin)) {
        if (kShouldFilterReflections_) {
            detectReflections(allMappingScans_);
        }
        map_.recenterActiveRegion(scanOrigin);
    }

    // Process the laser scan once the map is centered
    // Don't use the scan position for centering the map because the laser positions are separated by enough that the
    // glass map will thrash back and forth between blocks, significantly slowing the system down.
    processScanPoints(scan);

    if (kShouldFilterDynamic_) {
        filterDynamicObjects();
    }

    allMappingScans_.addRays(scan.begin(), scan.end());

    int64_t endTimeUs = system_time_us();

    totalTimeUs_ += endTimeUs - startTimeUs;
    maxUpdateTimeUs_ = std::max(maxUpdateTimeUs_, endTimeUs - startTimeUs);
    ++numUpdates_;

    std::cout << "Glass Update Time: This: " << (endTimeUs - startTimeUs) << "us. Mean: "
        << (totalTimeUs_ / numUpdates_) << "us Max: " << maxUpdateTimeUs_ << "us\n";

    map_.printSize();
}


void RFMBuilder::processScanPoints(const PointCloud2D& scan)
{
    const int64_t scanStartUs = system_time_us();

    findScanBins(scan);

    for (std::size_t n = 0; n < scanBins_.size(); ++n) {
        angle_bin_range_t binRange = findAngleBinRange(n);

        scanHits_.clear();
        markHitCellsInRange(binRange, scan);
        markFreeCellsInRange(binRange, scan);

        // Jump to the end of the current bin range
        n = binRange.endIndex - 1;
        // now the ++n will increment to the start of the next bin
    }

    const int64_t scanEndUs = system_time_us();
    const int64_t scanTimeUs = scanEndUs - scanStartUs;

    numScans_ += scan.size();
    totalScanTimeUs_ += scanTimeUs;
    maxScanTimeUs_ = std::max(maxScanTimeUs_, scanTimeUs);

    std::cout << "Scan time: " << scanTimeUs << "us Mean: " << (totalScanTimeUs_ / (numScans_ / 1081))
              << "us Max: " << maxScanTimeUs_ << "us\n";

    map_.flattenActiveRegion(kMinVisibleBins_);
}


void RFMBuilder::findScanBins(const PointCloud2D& scan)
{
    scanBins_.clear();

    for (std::size_t n = 0; n < scan.size(); ++n) {
        const auto& ray = scan[n];

        // Skip invalid scans
        if (ray.range < 0.0f) {
            continue;
        }

        ray_bin_t rayBin;
        rayBin.bearing = wrap_to_2pi(ray.angle);
        rayBin.angleBin = map_.angleToBin(rayBin.bearing);
        rayBin.scanIndex = n;

        scanBins_.push_back(rayBin);
    }

    // Sort by bins so that multiple lasers or measurements being contained in the same PointCloud2D will
    // be correctly handled for the logic of not overwriting hits with misses
    std::sort(scanBins_.begin(), scanBins_.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.angleBin < rhs.angleBin;
    });
}


RFMBuilder::angle_bin_range_t RFMBuilder::findAngleBinRange(std::size_t beginIndex)
{
    const int angleBin = scanBins_[beginIndex].angleBin;

    angle_bin_range_t range = {
      .beginIndex = beginIndex,
      .endIndex = scanBins_.size()   // for the last bin, won't trigger the break condition
                                     // so start with the end including the rest of the scan
    };

    // Find all rays that fall into the same angle bin within the scan
    for (std::size_t n = beginIndex + 1; n < scanBins_.size(); ++n) {
        if (scanBins_[n].angleBin != angleBin) {
            range.endIndex = n;
            break;
        }
    }

    return range;
}


void RFMBuilder::markHitCellsInRange(angle_bin_range_t range, const PointCloud2D& scan)
{
    // Mark all hit cells in the angle bin range
    for (std::size_t n = range.beginIndex; n < range.endIndex; ++n) {
        const auto& ray = scan[scanBins_[n].scanIndex];

        // Mark the hit if it was within range
        auto hitCell = global_point_to_grid_point(ray.endpoint, map_);
        if (ray.range < kMaxLaserRange_) {
            map_.observedCell(hitCell.x, hitCell.y, scanBins_[n].angleBin, ray.intensity, ray.range);

            if (kShouldAntiAliasHits_) {
                if (std::floor(hitCell.x + 0.5) != hitCell.x) {
                    map_.observedCell(hitCell.x + 0.5, hitCell.y, scanBins_[n].angleBin, ray.intensity, ray.range);
                }

                if (std::floor(hitCell.x - 0.5) != hitCell.x) {
                    map_.observedCell(hitCell.x - 0.5, hitCell.y, scanBins_[n].angleBin, ray.intensity, ray.range);
                }

                if (std::floor(hitCell.y + 0.5) != hitCell.y) {
                    map_.observedCell(hitCell.x, hitCell.y + 0.5, scanBins_[n].angleBin, ray.intensity, ray.range);
                }

                if (std::floor(hitCell.y - 0.5) != hitCell.y) {
                    map_.observedCell(hitCell.x, hitCell.y - 0.5, scanBins_[n].angleBin, ray.intensity, ray.range);
                }
            }

            scanHits_.push_back(hitCell);
        }
    }
}

void RFMBuilder::markFreeCellsInRange(angle_bin_range_t range,
                                           const PointCloud2D& scan)
{
    for (std::size_t n = range.beginIndex; n < range.endIndex; ++n) {
        markFreeCellsAlongRay(scanBins_[n], scan);
    }
}


void RFMBuilder::markFreeCellsAlongRay(const ray_bin_t& ray,
                                            const PointCloud2D& scan)
{
    // Trace the free up to the endpoint or the edge of the max range
    auto laserCell = global_point_to_grid_point(scan[ray.scanIndex].origin, map_);

    float range = std::min(scan[ray.scanIndex].range, kMaxLaserRange_);
    int numSteps = range * map_.cellsPerMeter();
    double x = 0.0;
    double y = 0.0;
    const double deltaX = std::cos(ray.bearing);
    const double deltaY = std::sin(ray.bearing);

    cell_t cell;

    for (int n = 0; n <= numSteps; ++n) {
        cell.x = static_cast<cell_idx_t>(laserCell.x + x);
        cell.y = static_cast<cell_idx_t>(laserCell.y + y);

        // Only mark misses if there wasn't a hit in the same cell/bin on this update
        if (!contains(scanHits_, cell)) {
            map_.missedCell(cell.x, cell.y, ray.angleBin);
        }

        x += deltaX;
        y += deltaY;
    }
}

}   // namespace rfm
