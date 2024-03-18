/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     glass_map_builder.h
 * \author   Collin Johnson
 *
 * Declaration of RFMBuilder, the HSSH-specific version of Paul's glass mapping.
 */

#ifndef RFM_RFM_BUILDER_H
#define RFM_RFM_BUILDER_H

#include "cell_grid.h"
#include "discretized_angle_grid.h"
#include "point_cloud_2d.h"
#include "reflectance_field_map.h"
#include "reflected_laser_scan.h"
#include "types.h"
#include <map>

namespace rfm
{

class RFMDynamicFilter;

struct lpm_params_t
{

  uint8_t maxCellCost;
  uint8_t occupiedCellCost;
  uint8_t freeCellCost;
};

struct rfm_map_builder_params_t
{
    int width;
    int height;
    float scale;

    float maxLaserRange;
    int numAngleBins;
    bool shouldFilterDynamic;        // flag indicating if the dynamic object filter should be run
    bool shouldFilterReflections;    // flag indicating if reflections should be removed from the laser scan
    bool shouldAntiAlias;   // flag indicating if hits in the glass map should be anti-aliased to increase signal and
                                     // help solidify the walls more

    int hitThreshold;    // number of hits in a bin for it to get marked as a hit while building the map
    int missThreshold;   // number of misses in a bin for it to get marked as a miss while building the map

    double minVisibleOccupiedRange;   // [radians] minimum visible range for a cell to be considered occupied in the
                                          // flattened map
    double minHighlyVisibleRange;     // [radians] minimum visible range for a cell to be highly visible in the dynamic
                                          // object filter
    uint16_t minHighlyVisibleIntensity;   // [sensor-specific counts] minimum intensity reading for a cell to be
                                          // considered highly-visible glass
};

/**
 * RFMBuilder
 */
class RFMBuilder
{
public:
    /**
     * Constructor for RFMBuilder.
     *
     * \param    params         Parameters for configuring the RFM-building algorithm
     * \param    currentPose    Current pose of the robot, around which the map will initially be centered
     */
    RFMBuilder(const rfm_map_builder_params_t& params, const Point<float>& center);

    /**
     * Destroy for RFMBuilder.
     */
    ~RFMBuilder();

    void reset();

    /**
     * changeMapBoundary shifts the boundary of the map to contain only cell within the
     * new boundary. Any shifted cells not inside the boundary are discarded. Cells not previously
     * in the boundary will be initialized to their default value
     *
     * \param    boundary        Boundary in meters defining the portion of the map
     */
    void changeMapBoundary(const Rectangle<float>& boundary);

    /**
     * updateMap updates the map with new sensor information. See the map_update_data_t description above
     * to see the exact contents of the provided data.
     *
     * After updateMap completes, the fresh map can be accessed via getMap(). updateMap will increment
     * the mapId.
     *
     * \param    data            Data for doing the map update
     */
    void updateMap(const PointCloud2D& scan, const Point<float>& scanOrigin);

    /**
     * Filter dynamic obstacles in the environment using a breadth-first search that begins from highly-visible
     * cells.
     */
    void filterDynamicObjects();

    /**
     * detectReflections searches through the provided laser scan for reflections in the map. Any cells that a reflected
     * ray passes through are returned to the default unknown state, which has the useful effect of erasing mysterious
     * walls that can appear as a consequence of reflections.
     *
     * \param    scan        Scan in which to find reflections
     * \return   Information on the reflections in the scan. Every valid ray (range > 0) in scan will be represented
     *   here. Rays in the scan with no reflections are simply marked as having no reflections, but are included in the
     *   returned scan.
     */
    ReflectedLaserScan detectReflections(const PointCloud2D& scan);

    //////////     Accessors     //////////
    ReflectanceFieldMap& getReflectanceFieldMap() { return map_; }
    const ReflectanceFieldMap& getReflectanceFieldMap() const { return map_; }

    /**
     * getCostMap retrieves the usable flattened map that represents the condensed version of the full 3D glass
     * map into a simpler 2D occupancy grid. The
     *
     * \return   Cost map of the environment
     */
    const CostGrid& getCostMap() const { return map_.flattenedMapCosts(); }

    /**
     * getTypeMap retrieves the usable flattened map that classifies each cell in the map based on information
     * inferred from the full 3D glass map.
     */
     const TypeGrid& getTypeMap() const { return map_.flattenedMapTypes(); }

    /**
     * Retrieve the intensity map of the environment. The intensity map encodes the max intensity and
     * the range at which it was measured.
     */
    const IntensityGrid& getIntensityMap() const { return map_.intensityMap(); }

private:
    // ray_bin_t is the angle bin for a given ray in the laser scan
    struct ray_bin_t
    {
        double bearing;          // bearing of the ray
        int angleBin;            // angle bin the ray falls in
        std::size_t scanIndex;   // index of the ray in the scan
    };

    // Range is [beginIndex, endIndex)
    struct angle_bin_range_t
    {
        std::size_t beginIndex;
        std::size_t endIndex;
    };

    ReflectanceFieldMap map_;
    std::vector<ray_bin_t> scanBins_;
    CellVector scanHits_;
    std::vector<int> angleBinPlusPi_;
    float kMaxLaserRange_;
    bool kShouldAntiAliasHits_;
    bool kShouldFilterDynamic_;
    bool kShouldFilterReflections_;
    int kMinVisibleBins_;
    int kMinHighlyVisibleBins_;
    uint16_t kMinGlassIntensity_;

    std::unique_ptr<RFMDynamicFilter> dynamicFilter_;

    PointCloud2D allMappingScans_;

    int numUpdates_ = 0;
    int64_t totalTimeUs_ = 0;
    int64_t maxUpdateTimeUs_ = 0;

    int64_t numReflectionDetections_ = 0;
    int64_t totalReflectionTimeUs_ = 0;
    int64_t maxReflectionTimeUs_ = 0;

    int64_t numScans_ = 0;
    int64_t totalScanTimeUs_ = 0;
    int64_t maxScanTimeUs_ = 0;

    // Internal helper methods for constructing the glass map
    void processScanPoints(const PointCloud2D& scan);
    void findScanBins(const PointCloud2D& scan);
    angle_bin_range_t findAngleBinRange(std::size_t beginIndex);
    void markHitCellsInRange(angle_bin_range_t range, const PointCloud2D& scan);
    void markFreeCellsInRange(angle_bin_range_t range, const PointCloud2D& scan);
    void markFreeCellsAlongRay(const ray_bin_t& ray, const PointCloud2D& scan);
};

}   // namespace rfm

#endif   // RFM_RFM_BUILDER_H
