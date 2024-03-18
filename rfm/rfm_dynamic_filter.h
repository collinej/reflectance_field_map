#ifndef RFM_RFM_DYNAMIC_FILTER_H
#define RFM_RFM_DYNAMIC_FILTER_H

#include "reflectance_field_map.h"

#include "cell_grid.h"
#include "rectangle.h"

#include <boost/dynamic_bitset.hpp>
#include <vector>

namespace rfm
{

class RFMDynamicFilter
{
public:

    /**
     * filterDynamicObjects filters dynamic objects out of the flattened map. The filtering process starts from
     * highly-visible cells and performs a breadth-first search through (x, y, theta) space. Any occupied cell
     * not encountered by the search becomes a dynamic cell.
     *
     * The filter only deals with the flattened map, so if the flattened map is stale, i.e. flattenMap() hasn't been
     * called recently, then the filter may not work as desired.
     *
     * The filter for dynamic objects occurs only within the active region in the map, as it requires angle data
     * which may or may not be loaded at the time.
     *
     * \param    minHighlyVisibleRange           Minimum visible range for a highly-visible cell
     *   (optional, default = 20)
     * \param    minGlassIntensity               Minimum intensity for a cell to be considered high-confidence glass,
     *   (optional, default = 8000)
     *
     * NOTE: The intensity threshold is sensor dependent. Every model of lidar will need to have a different threshold.
     * The default threshold here is for a Hokuyo UTM-30LX
     */
    void filterDynamicObjectsFromActiveRegion(ReflectanceFieldMap& map, int minHighlyVisibleRange = 20, uint16_t minGlassIntensity = 8000,
                                              int maxUnobservedForConnectivity = 2);

    /**
     * filterDynamicObjectsFromFullMap filters dynamic objects from the entire map. It shifts the active region as
     * needed to pass the filter over the full map. The active region is move by half its width/height, so the final
     * results of the filter might not be exactly the same as the incremental filtering performed during SLAM.
     *
     * At the end of the filtering, the active region will be returned to its original configuration.
     *
     * \param    minHighlyVisibleRange           Minimum visible range for a highly-visible cell
     *   (optional, default = 20)
     * \param    minGlassIntensity               Minimum intensity for a cell to be considered high-confidence glass,
     *   (optional, default = 8000)
     *
     * NOTE: The intensity threshold is sensor dependent. Every model of lidar will need to have a different threshold.
     * The default threshold here is for a Hokuyo UTM-30LX
     */
    void filterDynamicObjectsFromFullMap(ReflectanceFieldMap& map, int minHighlyVisibleRange = 20, uint16_t minGlassIntensity = 8000);

private:

    using VisitedBits = boost::dynamic_bitset<>;

    // This is a circular range, so size can wrap around to the beginning of the range depending on the
    // circumstances
    struct IndexRange
    {
        int startIdx = 0;
        int size = 0;

        IndexRange() = default;
        IndexRange(int startIdx, int size)
        : startIdx(startIdx)
        , size(size)
        {
        }
    };

    struct GlassBinRange
    {
        Point<int> glassCell;
        Point<int> flatCell;
        IndexRange range;
    };

    // VisitedSet is a grid containing a bitset
    class VisitedSet
    {
    public:

        void initialize(const Rectangle<int>& glassActiveRegion, int numAngleBins)
        {
            if ((glassActiveRegion.width() != visitedGrid_.getWidthInCells())
                || (glassActiveRegion.height() != visitedGrid_.getHeightInCells())) {
                visitedGrid_.setGridSizeInCells(glassActiveRegion.width(), glassActiveRegion.height());
                visitedGrid_.reset(VisitedBits(numAngleBins));
            } else {
                for (std::size_t y = 0; y < visitedGrid_.getHeightInCells(); ++y) {
                    for (std::size_t x = 0; x < visitedGrid_.getWidthInCells(); ++x) {
                        visitedGrid_(x, y).reset();
                    }
                }
            }

            glassToGridOffset_ = glassActiveRegion.bottomLeft;
        }

        bool contains(int x, int y, int theta) const
        {
            return bits(x, y).test(theta);
        }

        void add(int x, int y, int theta)
        {
            bits(x, y).set(theta);
        }

        VisitedBits& bits(int x, int y)
        {
            const auto [cellX, cellY] = glassToGridCell(x, y);
            return visitedGrid_(cellX, cellY);
        }

        const VisitedBits& bits(int x, int y) const
        {
            const auto [cellX, cellY] = glassToGridCell(x, y);
            return visitedGrid_(cellX, cellY);
        }

    private:

        CellGrid<VisitedBits> visitedGrid_;
        Point<int> glassToGridOffset_;

        std::pair<int, int> glassToGridCell(int x, int y) const
        {
            return std::make_pair(x - glassToGridOffset_.x, y - glassToGridOffset_.y);
        }
    };

    using GlassBinQueue = std::deque<GlassBinRange>;

    GlassBinQueue searchQueue_;
    VisitedSet visitedList_;

    int maxUnobservedForConnectivity_;

    int64_t totalFilteringTime_ = 0;
    int64_t maxFilteringTime_ = 0;
    int64_t numFilterings_ = 0;

    void enqueueAllHitRanges(ReflectanceFieldMap& map, const Point<int>& glassCell);
    void enqueueAllAdjacent(ReflectanceFieldMap& map, const Point<int>& glassCell, const IndexRange& range);
    void enqueueOverlappingRanges(ReflectanceFieldMap& map, const Point<int>& glassCell, const IndexRange& range);
    int enqueueHitRange(ReflectanceFieldMap& map,
                         const Point<int>& glassCell,
                         int startIdx,
                         VisitedBits& visited);
    IndexRange growHitRange(ReflectanceFieldMap& map, const Point<int>& glassCell, int startIdx);
    int findFirstMiss(ReflectanceFieldMap& map, const Point<int>& glassCell,
                      int startIdx,
                      ReflectanceFieldMap::BinConstIter end,
                      int direction) const;
};

} // namespace rfm

#endif // RFM_RFM_DYNAMIC_FILTER_H
