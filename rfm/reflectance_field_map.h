/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     reflectance_field_map.h
 * \author   Collin Johnson
 *
 * Declaration of ReflectanceFieldMap.
 */

#ifndef RFM_REFLECTANCE_FIELD_MAP_H
#define RFM_REFLECTANCE_FIELD_MAP_H

#include "angle_range.h"
#include "cell_grid.h"
#include "rectangle.h"
#include "discretized_angle_grid.h"
#include "types.h"
#include <cereal/access.hpp>

namespace rfm
{

struct cell_intensity_t
{
    uint16_t intensity;
    float range;
};

using CostGrid = CellGrid<uint8_t>;
using TypeGrid = CellGrid<uint8_t>;
using IntensityGrid = CellGrid<cell_intensity_t>;

/**
 * ReflectanceFieldMap is a grid map that maintains both an occupancy estimate and an estimate of the visible range for
 * each cell in the grid.
 */
class ReflectanceFieldMap
{
public:
    // Iterator for accessing the individual bins associated with a particular cell
    using BinConstIter = DiscretizedAngleGrid::BinConstIter;

    static constexpr cell_type_t OccupiedMask =
      kOccupiedOccGridCell | kDynamicOccGridCell | kLimitedVisibilityOccGridCell;

    ReflectanceFieldMap();

    /**
     * Constructor to create a new ReflectanceFieldMap.
     *
     * \param    maxLaserRange           Maximum range the laser will be used for
     * \param    numAngleBins            Number of angle bins to use per cell
     * \param    flattenHitThreshold     Number of hits in a bin for it to be considered a hit when flattening
     * \param    flattenMissThreshold    Number of misses in a bin for it to be consider a miss when flattening
     * \param    gridWidth               Width of the grid in cells
     * \param    gridHeight              Height of the grid in cells
     * \param    gridScale               Meters per grid cell
     * \param    globalCenter            Global center of the map
     */
    ReflectanceFieldMap(float maxLaserRange,
             int numAngleBins,
             int8_t flattenHitThreshold,
             int8_t flattenMissThreshold,
             std::size_t gridWidth,
             std::size_t gridHeight,
             float gridScale,
             const Point<float>& globalCenter);

    ReflectanceFieldMap(const ReflectanceFieldMap& toCopy) = default;
    ReflectanceFieldMap(ReflectanceFieldMap&& toMove) noexcept = default;
    ReflectanceFieldMap& operator=(const ReflectanceFieldMap& rhs) = default;
    ReflectanceFieldMap& operator=(ReflectanceFieldMap&& rhs) = default;

    // Methods for accessing grid parameters
    int64_t getTimestamp() const { return timestamp_; }
    int32_t getId() const { return id_; }

    std::size_t getWidthInCells() const { return hitGrid_.getWidthInCells(); }
    float getWidthInMeters() const { return hitGrid_.getWidthInMeters(); }

    std::size_t getHeightInCells() const { return hitGrid_.getHeightInCells(); }
    float getHeightInMeters() const { return hitGrid_.getHeightInMeters(); }

    int numAngleBins() const { return numAngleBins_; }

    float metersPerCell() const { return hitGrid_.metersPerCell(); }
    float cellsPerMeter() const { return hitGrid_.cellsPerMeter(); }

    Point<float> getGlobalCenter() const { return hitGrid_.getGlobalCenter(); }
    Point<float> getBottomLeft() const { return hitGrid_.getBottomLeft(); }

    // Methods for mutating the grid

    void setTimestamp(int64_t newTime) { timestamp_ = newTime; }
    void setId(int32_t mapId) { id_ = mapId; }
    void setGridSizeInCells(std::size_t width, std::size_t height);
    void setMetersPerCell(float gridScale);
    void setBottomLeft(const Point<float>& bottomLeft);

    // Methods for modifying position of the grid
    /**
     * changeBoundary changes the shape and boundary of the ReflectanceFieldMap. The reshaped area is specified as a new metric
     * boundary for the ReflectanceFieldMap. The parts of the current grid that fall into the reshaped area will be copied
     * into the appropriate location, and the rest of the cells will be set to the default value.
     *
     * \param    newBoundary         Reshaped boundary for the ReflectanceFieldMap
     */
    void changeBoundary(const Rectangle<float>& newBoundary);

    /**
     * Check to see if the current robot position has drifted sufficiently far from the active region
     * that it needs to shift in order to ensure that all rays remain in the map.
     */
     bool needToRecenterActiveRegion(const Point<float>& position) const;

    /**
     * recenterActiveRegion centers the active region of the map around the provided global position.
     *
     * \param    position                New center position for the active region of the glass map (global coords)
     * \return  True if the active region is recentered.
     */
    bool recenterActiveRegion(const Point<float>& position);

    /**
     * reset resets all cells into the grid to value
     */
    void reset();

    /**
     * Reset the intensity data back to 0.
     */
    void resetIntensity();

    bool isCellInGrid(const Point<int>& cell) const { return hitGrid_.isCellInGrid(cell.x, cell.y); }
    bool isCellInGrid(int x, int y) const { return hitGrid_.isCellInGrid(x, y); }

    /**
     * isCellInActiveRegion checks if a cell is in the active region. If it isn't, then the active region can be
     * recentered via recenterActiveRegion.
     */
    bool isCellInActiveRegion(int x, int y) const { return hitGrid_.isCellInActiveRegion(x, y); }

    /**
     * activeRegionInCells retrieves the boundary of the active region in cells.
     */
    Rectangle<int> activeRegionInCells() const { return hitGrid_.getActiveRegionInCells(); }

    /**
     * activeRegionInMeters retrieves the boundary of the active region in the map in meters/global coordinates.
     */
    Rectangle<float> activeRegionInMeters() const { return hitGrid_.getActiveRegion(); }

    /**
     * beginBin retrieves the start iterator for a cell's bins in the ReflectanceFieldMap. If the cell isn't in the active region,
     * then beginBin == endBin.
     */
    BinConstIter beginBin(int x, int y) const { return hitGrid_.beginBin(x, y); }

    /**
     * endBin retrieves the end iterator for a cell's bins in the ReflectanceFieldMap. If the cell isn't in the active region,
     * then beginBin == endBin.
     */
    BinConstIter endBin(int x, int y) const { return hitGrid_.endBin(x, y); }

    /**
     * haveOverlappingAngles checks if two cells are adjacent in both position and angle. The adjacency check requires
     * that the cells are 8-way connected within the grid. Given these 8-way connected cells, then the two cells
     * are considered overlapping if the following holds:
     *
     *   There exists some n in [0, numAngleBins) s.t.
     *           (beginBin(x1, y1) + n) > hitThreshold && (beginBin(x2, y2) + n) > hitThreshold
     *
     * \param    x1      x-coordinate of first cell
     * \param    y1      y-coordinate of first cell
     * \param    x2      x-coordinate of second cell
     * \param    y2      y-coordinate of second cell
     * \return   True if the two cells have overlapping angle ranges.
     */
    bool haveOverlappingAngles(int x1, int y1, int x2, int y2) const;

    /**
     * angleToBin converts an angle to a bin. The angle must be in the range [0, 2pi). This bin should be used when
     * marking an observed or missed cell.
     */
    int angleToBin(double angle) const;

    /**
     * binToAngle converts a bin to an angle. The angle returned will be in the range [0, 2pi).
     *
     * \param    bin         Bin to convert
     * \pre  bin < numAngleBins && bin >= 0
     */
    double binToAngle(int bin) const;

    /**
     * observedCell indicates an angle from which a cell was observed.
     *
     * \param    x           x-coordinate of cell
     * \param    y           y-coordinate of cell
     * \param    angleBin    Angle in of the cell
     * \param    intensity   Measured intensity (optional, default = 0)
     * \param    range       Range of measurement producing the intensity
     * \return   True if the state of the particular cell switches from free to occupied.
     */
    bool observedCell(int x, int y, int angleBin, uint16_t intensity = 0, float range = 0.0f);

    /**
     * missedCell indicates the angle from which a cell was seen through.
     *
     * \return   True if the state of the cell switches from occupied to free.
     */
    bool missedCell(int x, int y, int angleBin);

    /**
     * reflectedCell marks a cell as being identified due to a reflection. A reflected resets all knowledge of the
     * particular bin to 0.
     */
    void reflectedCell(int x, int y, int angleBin);

    /**
     * flattenActiveRegion flattens the angle bins in the active region into a single occupancy grid value -- occupied
     * or free.
     *
     * \param    minVisibleRange         A parameter controlling the number of angle bins in which a cell is visible
     *   in order for it to be considered occupied (optional, default = 1)
     */
    void flattenActiveRegion(int minVisibleRange = 1);

    /**
     * flattenFullMap flattens the entire glass map into an occupancy grid.
     *
     * \param    minVisibleRange         A parameter controlling the number of angle bins in which a cell is visible
     *   in order for it to be considered occupied (optional, default = 1)
     */
    void flattenFullMap(int minVisibleRange = 1);

    /**
     * flattenedMapCosts retrieves the cost map in which dynamic objects have been filtered out and the angle bins have been
     * flattened into a single occupancy value -- yay or nay.
     */
    const CostGrid& flattenedMapCosts() const { return costGrid_; }

    /**
     * flattenedMapTypes retrieves the type map in which the cells have been classified based on whether they are
     * occupied, limited visibility, free, unknown, etc.
     */
    const TypeGrid& flattenedMapTypes() const { return typeGrid_; }

    /**
     * intensityMap retrieves the map containing the maximum intensity measurement for each cell in the grid. This map
     * only contains valid data if intensity information was available in the laser scan.
     */
    const IntensityGrid& intensityMap() const { return intensityGrid_; }

    /**
     * glassToFlatMapOffset retrieves the cell offset between the glass map and the flattened map. The origin of the
     * glass map is located at the returned offset point in the flat map. This offset allows easy and exact conversion
     * between the glass and flattened maps.
     *
     *   glassCell + glassToFlatMapOffset = flatCell
     *   flatCell - glassToFlatMapOffset = glassCell
     */
    Point<int> glassToFlatMapOffset() const { return glassToFlatCellOffset_; }

    /**
     * Print the full amount of memory the map currently requires.
     */
    void printSize() const;

    /**
     * Apply some Func f to the active region. The function is applied per position (x,y),
     * not per cell (x,y,theta). The glass map instance is provided as the first member of
     * func so that you can
     *
     * The signature of f is:
     *
     *  f(glass_map, x, y)
     */
    template <typename Func>
    void mapFuncActive(Func f) const
    {
        auto activeRegion = activeRegionInCells();
        for (int y = activeRegion.bottomLeft.y; y < activeRegion.topRight.y; ++y) {
            for (int x = activeRegion.bottomLeft.x; x < activeRegion.topRight.x; ++x) {
                f(*this, x, y);
            }
        }
    }

    /**
     * Apply some Func f to the entire map.
     *
     * The active region will be moved around in the map
     * The original active region will be saved and restored at the end of the operation.
     */
    template <typename Func>
    void mapFuncFull(Func f) const
    {
        for (std::size_t y = 0; y < hitGrid_.getHeightInCells(); ++y) {
            for (std::size_t x = 0; x < hitGrid_.getWidthInCells(); ++x) {
                if (!isCellInActiveRegion(x, y)) {
                    auto posInGlobal = grid_point_to_global_point(Point<int>(x, y), hitGrid_);
                    hitGrid_.recenterActiveRegion(posInGlobal);
                }

                f(*this, x, y);
            }
        }
    }

private:

    friend class RFMDynamicFilter;

    struct CellCount
    {
        int16_t hits = 0;
        int16_t misses = 0;

        int total() const { return hits + misses; }

        template <class Archive>
        void serialize(Archive& ar)
        {
          ar (hits, misses);
        }
    };

    int64_t timestamp_;
    int32_t id_;

    int numAngleBins_;
    int8_t hitThreshold_;
    int8_t missThreshold_;
    mutable DiscretizedAngleGrid hitGrid_;
    CostGrid costGrid_;
    TypeGrid typeGrid_;
    CellGrid<CellCount> counts_;
    IntensityGrid intensityGrid_;

    bool haveIntensity_ = false;   // flag indicating if there is valid, non-zero intensity data in the scan
    Point<int> glassToFlatCellOffset_;

    int64_t totalFlatteningTime_ = 0;
    int64_t maxFlatteningTime_ = 0;
    int64_t numFlattenings_ = 0;

    void setGridOffset();
    Point<int> glassToFlatMap(int x, int y);
    Point<int> glassToFlatMap(Point<int> glass);
    Point<int> flatToRFM(int x, int y);
    Point<int> flatToRFM(Point<int> flat);

    // boundary in cell is for hitCount_/flattenedGrid_
    void flattenRegion(Rectangle<int> boundaryInCells, int minVisibleRange);
    void flushCacheToDisk() const;

    bool isHit(int8_t value) const { return value >= hitThreshold_; }
    bool isMiss(int8_t value) const { return value < 0; }

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void load(Archive& ar, const unsigned int version)
    {
        ar(timestamp_,
           id_,
           numAngleBins_,
           hitThreshold_,
           missThreshold_,
           hitGrid_,
           costGrid_,
           typeGrid_,
           counts_,
           intensityGrid_,
           haveIntensity_,
           glassToFlatCellOffset_);
    }

    template <class Archive>
    void save(Archive& ar, const unsigned int version) const
    {
        flushCacheToDisk();

        ar(timestamp_,
           id_,
           numAngleBins_,
           hitThreshold_,
           missThreshold_,
           hitGrid_,
           costGrid_,
           typeGrid_,
           counts_,
           intensityGrid_,
           haveIntensity_,
           glassToFlatCellOffset_);
    }
};

}   // namespace rfm

#endif   // RFM_REFLECTANCE_FIELD_MAP_H
