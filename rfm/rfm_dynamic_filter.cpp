#include "rfm_dynamic_filter.h"

#include "rfm_utils.h"

#include "timestamp.h"

namespace rfm
{

void RFMDynamicFilter::filterDynamicObjectsFromActiveRegion(ReflectanceFieldMap& map, int minHighlyVisibleRange, uint16_t /*minGlassIntensity*/, int maxUnobservedForConnectivity)
{
    // To filter, change all occupied cells to dynamic. Flip cells back to occupied after they confirmed to be
    // attached to a highly visible cell
    minHighlyVisibleRange = std::max(minHighlyVisibleRange, 1);   // must be visible from at least one angle

    maxUnobservedForConnectivity_ = maxUnobservedForConnectivity;

    int64_t computationStartTimeUs = system_time_us();

    auto glassActiveRegion = map.activeRegionInCells();
    auto flatActiveRegion = active_region_in_flat_map(map);

    //    std::deque<Point<int>> searchQueue;
    //
    // Set all occupied and dynamic to be dynamic, which allows easily knowing if a particular cell has been visited
    // or not
    for (int y = flatActiveRegion.bottomLeft.y; y < flatActiveRegion.topRight.y; ++y) {
        for (int x = flatActiveRegion.bottomLeft.x; x < flatActiveRegion.topRight.x; ++x) {
            if (map.typeGrid_(x, y) & kHitCellTypes) {
                map.typeGrid_(x, y) = kDynamicOccGridCell;
            }
        }
    }
    //
    //    // For all highly visible cells, add them to the initial search queue and flip them back to being occupied
    //    // because the search starts here
    //    for (int y = flatActiveRegion.bottomLeft.y; y < flatActiveRegion.topRight.y; ++y) {
    //        for (int x = flatActiveRegion.bottomLeft.x; x < flatActiveRegion.topRight.x; ++x) {
    //            // If the intensity is greater than the min glass intensity, definitely glass
    //            // If the hit count is higher than the min visible range, definitely highly visible
    //            // If there's been a hit and no misses, then the hit count might be highly visible, so mark as occupied
    //            int totalObservedBins = counts_(x, y).hits + counts_(x, y).misses;
    //            float probVisible = (totalObservedBins > 0) ? (counts_(x, y).hits / static_cast<float>(totalObservedBins)) : 0.0f;
    //            if (std::lrint(probVisible * 100) > minHighlyVisibleRange) {
    //                searchQueue.push_back(Point<int>(x, y));
    //                flattenedGrid_.setTypeNoCheck(Point<int>(x, y), kOccupiedOccGridCell);
    //            }
    //        }
    //    }
    //
    //    // Use an 8-way search
    //    const int xDeltas[8] = {1, -1, 0, 0, -1, -1, 1, 1};
    //    const int yDeltas[8] = {0, 0, 1, -1, -1, 1, -1, 1};
    //
    //    // Queue up cells that sit right outside the active region. If any of them are occupied and are adjacent in (x,y)
    //    // to a cell in the active region, then add that active region cell to the queue. This change causes the walls saved
    //    // during a previous dynamic search to still exist even when they fall outside the active region
    //    for (int y = flatActiveRegion.bottomLeft.y - 1; y <= flatActiveRegion.topRight.y; ++y) {
    //        for (int x = flatActiveRegion.bottomLeft.x - 1; x <= flatActiveRegion.topRight.x; ++x) {
    //            // Ignore cells that...
    //            if (is_cell_in_region(Point<int>(x, y), flatActiveRegion)   // are in the active region
    //                || !flattenedGrid_.isCellInGrid(Point<int>(x, y))       // aren't in the flat map
    //                || !(flattenedGrid_.getCellTypeNoCheck(x, y)
    //                     & (kOccupiedOccGridCell | kLimitedVisibilityOccGridCell)))   // aren't occupied
    //            {
    //                continue;
    //            }
    //
    //            for (int n = 0; n < 8; ++n) {
    //                Point<int> adjacentCell(x + xDeltas[n], y + yDeltas[n]);
    //                // Ensure the cell is in the active region and it is dynamic -- if so, then it put it on the queue
    //                if (is_cell_in_region(adjacentCell, flatActiveRegion)
    //                    && flattenedGrid_.getCellType(adjacentCell) == kDynamicOccGridCell) {
    //                    flattenedGrid_.setTypeNoCheck(adjacentCell, kOccupiedOccGridCell);
    //                    searchQueue.push_back(adjacentCell);
    //                }
    //            }
    //        }
    //    }
    //
    //    while (!searchQueue.empty()) {
    //        auto cell = searchQueue.front();
    //        searchQueue.pop_front();
    //
    //        auto angleCell = cell - glassToFlatCellOffset_;
    //
    //        for (int n = 0; n < 8; ++n) {
    //            Point<int> adjacentCell(cell.x + xDeltas[n], cell.y + yDeltas[n]);
    //            auto adjacentAngleCell = adjacentCell - glassToFlatCellOffset_;
    //            // Ensure the cell is in the map
    //            if (!is_cell_in_region(adjacentCell, flatActiveRegion)
    //                || !is_cell_in_region(adjacentAngleCell, activeRegion)) {
    //                continue;
    //            }
    //
    //            // Skip anything that isn't dynamic because it is either free or already be identified in the search.
    //            if (flattenedGrid_.getCellTypeNoCheck(adjacentCell) != kDynamicOccGridCell) {
    //                continue;
    //            }
    //
    //            // If the cells are also adjacent across angles, then add the cell to the queue and mark as part of
    //            // the occupied portion of the environment.
    //            if (haveOverlappingAngles(angleCell.x, angleCell.y, adjacentAngleCell.x, adjacentAngleCell.y)) {
    //                flattenedGrid_.setTypeNoCheck(adjacentCell, kLimitedVisibilityOccGridCell);
    //                searchQueue.push_back(adjacentCell);
    //            }
    //        }
    //    }

    /*
     * The search space is quite large, so there's the potential for it to be very slow.
     * To minimize the search space, we only do the search through cells that we aren't
     * certain to be highly-visible.
     *
     * To do this, we perform the search in the following way:
     *  - Find all cells that are highly visible, marked as occupied.
     *  - Find cells on the boundary of the active region, and ensure all cells in the
     *    active region get added to the search.
     *  - Start the BFS through limited-visibility space. When a cell is encountered, then
     *    mark directly as limited visibility. This allows us to treat occupied cells as
     *    visited already, and therefore we don't enqueue or spend effort enqueueing the many
     *    visible angle bins.
     *      - We also mark all the highly visible cells first, and then do the enqueuing to ensure
     *        that no highly visible cells end up in the x,y,theta queue.
     *      - To get further efficiency, we treat the neighbors of a highly-visible cell with a
     *        special function that queries the limited cell against the highly visible cell because
     *        that results in fewer queries since there are typically only one or two connected
     *        components among the limited visibility cells
     *
     * Thus, we maintain one search queue through the limited visibility cells. We have two
     * data structures that serve as visited lists:
     *   1) The traditional set of limited visibility cells that are in the search queue
     *   2) The highly-visible cells, which we know are visited because they are marked as occupied
     */

    // Search queue is storing values in the glass map
    searchQueue_.clear();
    visitedList_.initialize(glassActiveRegion, map.numAngleBins());

    int numHighlyVisible = 0;

    // For all highly visible cells, add them to the initial search queue and flip them back to being occupied
    // because the search starts here
    for (int y = flatActiveRegion.bottomLeft.y; y < flatActiveRegion.topRight.y; ++y) {
        for (int x = flatActiveRegion.bottomLeft.x; x < flatActiveRegion.topRight.x; ++x) {
            // If the intensity is greater than the min glass intensity, definitely glass
            // If the hit count is higher than the min visible range, definitely highly visible
            // If there's been a hit and no misses, then the hit count might be highly visible, so mark as occupied
            int totalObservedBins = map.counts_(x, y).total();
            float probVisible =
              (totalObservedBins > 0) ? (map.counts_(x, y).hits / static_cast<float>(totalObservedBins)) : 0.0f;
            if (std::lrint(probVisible * 100) > minHighlyVisibleRange) {
                map.typeGrid_(x, y) = kOccupiedOccGridCell;
                ++numHighlyVisible;
            }
        }
    }

    // INVARIANT: At this point, all highly visible cells have been marked as Occupied and all
    // other hits cells are marked as Dynamic
    // Indices to use for the adjacency search
    // Ordering here is to try and improve locality of memory access. Grid is stored in row-major order
    // so perform the search in the same row-order fashion
    const std::array<Point<int>, 8> deltas = {
        Point<int>{-1,-1},
        Point<int>{0, -1},
        Point<int>{1, -1},
        Point<int>{-1, 0},
        Point<int>{1,  0},
        Point<int>{-1, 1},
        Point<int>{0,  1},
        Point<int>{1,  1}
    };

    for (int y = flatActiveRegion.bottomLeft.y; y < flatActiveRegion.topRight.y; ++y) {
        for (int x = flatActiveRegion.bottomLeft.x; x < flatActiveRegion.topRight.x; ++x) {
            if (map.typeGrid_(x, y) != kOccupiedOccGridCell) {
                continue;
            }

            auto glassCell = map.flatToRFM(x, y);

            // For highly-visible cells, we enqueue the entire range, so any cell with any
            // hit overlap is added
            const IndexRange fullRange(0, map.numAngleBins());

            for (const auto& delta : deltas) {
                const Point<int> adjacentCell(x + delta.x, y + delta.y);
                const Point<int> adjacentGlass = glassCell + delta;

                // Ensure the cell is in the active region, and it is dynamic
                // If so, then it put it on the queue because it might be limited visibility
                if (is_cell_in_region(adjacentGlass, glassActiveRegion)
                    && is_cell_in_region(adjacentCell, flatActiveRegion)
                    && map.typeGrid_(adjacentCell.x, adjacentCell.y) == kDynamicOccGridCell) {
                    enqueueOverlappingRanges(map, adjacentGlass, fullRange);
                }
            }
        }
    }

    // Queue up cells that are that sit right outside the active region. If any of them are occupied
    // and are adjacent in (x,y) to a cell in the active region, then add that active region cell to the queue.
    // This change causes the walls saved during a previous dynamic search to still exist even when they fall
    // outside the active region
    for (int y = flatActiveRegion.bottomLeft.y - 1; y <= flatActiveRegion.topRight.y; ++y) {
        for (int x = flatActiveRegion.bottomLeft.x - 1; x <= flatActiveRegion.topRight.x; ++x) {
            // Ignore cells that...
            if (!map.typeGrid_.isCellInGrid(Point<int>(x, y))             // aren't in the flat map
                || is_cell_in_region(Point<int>(x, y), flatActiveRegion)   // are in the active region
                || !(map.typeGrid_(x, y)
                     & (kOccupiedOccGridCell | kLimitedVisibilityOccGridCell)))   // aren't occupied
            {
                continue;
            }

            // Enqueue any cells overlapping a previous visible cell because we've lost the
            // information about exactly which angles we would be able to view them from
            const IndexRange fullRange(0, map.numAngleBins());

            for (const auto& delta : deltas) {
                const Point<int> adjacentCell(x + delta.x, y + delta.y);
                const auto adjacentGlass = map.flatToRFM(adjacentCell);
                // Ensure the cell is in the active region, and it is dynamic
                // If so, then it put it on the queue because it might be limited visibility
                if (is_cell_in_region(adjacentGlass, glassActiveRegion)
                    && map.typeGrid_.getValue(adjacentCell.x, adjacentCell.y) == kDynamicOccGridCell) {
                    enqueueOverlappingRanges(map, adjacentGlass, fullRange);
                }
            }
        }
    }

    std::cout << "\n\nINITIAL: \nHighly visible cells: " << numHighlyVisible
              << "\nAdjacent visible bins: " << searchQueue_.size() << "\n\n";

    auto numVisited = searchQueue_.size();
    std::size_t maxQueueSize = 0;

    while (!searchQueue_.empty()) {
        maxQueueSize = std::max(maxQueueSize, searchQueue_.size());
        ++numVisited;

        const auto glassBin = searchQueue_.front();
        searchQueue_.pop_front();

        map.typeGrid_(glassBin.flatCell.x, glassBin.flatCell.y) = kLimitedVisibilityOccGridCell;

        for (const auto& delta : deltas) {
            const Point<int> adjacentGlassCell = glassBin.glassCell + delta;
            const Point<int> adjacentFlatCell = glassBin.flatCell + delta;

            // Ensure the cell is in the map
            if (!is_cell_in_region(adjacentGlassCell, glassActiveRegion) // ...is in the glass map
                || !is_cell_in_region(adjacentFlatCell, flatActiveRegion)) {  // ...is in the flat map
                continue;
            }

            // If this isn't part of a potential glass region, then we don't need to
            // continue the search
            const auto cellType = map.typeGrid_(adjacentFlatCell.x, adjacentFlatCell.y);
            if ((cellType != kDynamicOccGridCell) && (cellType != kLimitedVisibilityOccGridCell)) {
                continue;
            }

            enqueueOverlappingRanges(map, adjacentGlassCell, glassBin.range);
        }
    }

    int64_t computationTimeUs = system_time_us() - computationStartTimeUs;

    totalFilteringTime_ += computationTimeUs;
    maxFilteringTime_ = std::max(maxFilteringTime_, computationTimeUs);
    ++numFilterings_;

    std::cout << "Filtering time: " << computationTimeUs << "us Mean: " << (totalFilteringTime_ / numFilterings_)
              << "us Max: " << maxFilteringTime_ << "us Queue max: " << maxQueueSize << " Visited: " << numVisited
              << " Time per visited: " << computationTimeUs / static_cast<double>(numVisited) << "us\n";
}


void RFMDynamicFilter::filterDynamicObjectsFromFullMap(ReflectanceFieldMap& map, int minHighlyVisibleRange, uint16_t minGlassIntensity)
{
    // Find the number of x and y increments in the active regions to fit the whole map
    // If the radius is larger than the grid, make the increment just the center of the grid
    const double xRegionIncrement = (map.hitGrid_.activeRegionRadius() < map.hitGrid_.getWidthInMeters())
      ? map.hitGrid_.activeRegionRadius()
      : map.hitGrid_.getWidthInMeters() / 2.0;
    // Always at least one increment
    const int numXIncrements = static_cast<int>(map.hitGrid_.getWidthInMeters() / map.hitGrid_.activeRegionRadius()) + 1;

    // If the radius is larger than the grid, make the increment just the center of the grid
    const double yRegionIncrement = (map.hitGrid_.activeRegionRadius() < map.hitGrid_.getHeightInMeters())
      ? map.hitGrid_.activeRegionRadius()
      : map.hitGrid_.getHeightInMeters() / 2.0;
    // Always at least one increment
    const int numYIncrements = static_cast<int>(map.hitGrid_.getHeightInMeters() / map.hitGrid_.activeRegionRadius()) + 1;

    // Before beginning to shift, save the original active center, so it can be restored at the end
    auto originalRegionCenter = map.hitGrid_.activeRegionCenter();

#ifdef DEBUG_REGION_FILTER
    std::cout << "DEBUG: RFM::filterDynamicObjectsFromFullMap: Num increments (x,y): (" << numXIncrements << ','
              << numYIncrements << ") Increment size: (" << xRegionIncrement << ',' << yRegionIncrement << ")\n";
#endif

    // Create all the regions and filter one-by-one
    for (int y = 0; y < numYIncrements; ++y) {
        Point<float> activeCenter = map.hitGrid_.getBottomLeft();
        activeCenter.y += (y + 1) * yRegionIncrement;   // add one to account for shifting from bottom left to center

        for (int x = 0; x < numXIncrements; ++x) {
            activeCenter.x += xRegionIncrement;
            map.hitGrid_.recenterActiveRegion(activeCenter);

#ifdef DEBUG_REGION_FILTER
            std::cout << "Filtering region: Center:" << activeCenter << " Boundary:" << hitGrid_.getActiveRegion()
                      << '\n';
#endif

            filterDynamicObjectsFromActiveRegion(map, minHighlyVisibleRange, minGlassIntensity);
        }
    }

    // Restore the original active region
    map.hitGrid_.recenterActiveRegion(originalRegionCenter);
}


void RFMDynamicFilter::enqueueOverlappingRanges(ReflectanceFieldMap& map,
                                                     const Point<int>& glassCell,
                                                     const IndexRange& range)
{
    // Only get the bits once to avoid repeated cell frame conversions
    auto& bits = visitedList_.bits(glassCell.x, glassCell.y);

    for (int i = range.startIdx; i < map.numAngleBins(); ++i) {
        // Skip any visited cells. As we go through this loop, bits will continue to get set, but we'll perform
        // the check anyway for simplicity of implementation. This is once place we could get a little speedup
        // by returning the max index from a range that gets expanded
        if (bits.test(i)) {
            continue;
        }

        i += enqueueHitRange(map, glassCell, i, bits);
    }

    const int numWrappedBins = range.startIdx + range.size - map.numAngleBins();
    for (int i = 0; i < numWrappedBins; ++i) {
        if (bits.test(i)) {
            continue;
        }

        i += enqueueHitRange(map, glassCell, i, bits);
    }
}


int RFMDynamicFilter::enqueueHitRange(ReflectanceFieldMap& map,
                                           const Point<int>& glassCell,
                                           int startIdx,
                                           VisitedBits& visited)
{
    auto range = growHitRange(map, glassCell, startIdx);

    if (range.size == 0) {
        return 0;
    }

    // Mark all the indices as visited
    for (int i = range.startIdx; i < map.numAngleBins(); ++i) {
        visited.set(i);
    }

    const int numWrappedBins = range.startIdx + range.size - map.numAngleBins();
    for (int i = 0; i < numWrappedBins; ++i) {
        visited.set(i);
    }

    searchQueue_.emplace_back(glassCell, map.glassToFlatMap(glassCell), range);

    return range.size;
}

RFMDynamicFilter::IndexRange
  RFMDynamicFilter::growHitRange(ReflectanceFieldMap& map,
                                                                      const Point<int>& glassCell,
                                                                      int startIdx)
{
    // Starting from theta, search forward and backward and if a cell is a hit, then enqueue.
    // If the cell is a miss, then the search in that direction stops since we're just worried about
    // the connected component here.
    // If we hit a visited cell, we can also exit early because we always add all connected cells to the
    // queue at the same time
    int forwardEnd = findFirstMiss(map, glassCell, startIdx, map.endBin(glassCell.x, glassCell.y), 1);

    // Did we wrap around? Then we need to keep going
    if (forwardEnd == map.numAngleBins()) {
        forwardEnd = findFirstMiss(map, glassCell, 0, map.beginBin(glassCell.x, glassCell.y) + startIdx, 1);
    }

    // If the forward search hits the startIdx, then the entire range is visible
    if (forwardEnd == startIdx) {
        return { 0, map.numAngleBins() };
    }

    int reverseEnd = findFirstMiss(map, glassCell, startIdx, map.beginBin(glassCell.x, glassCell.y) - 1, -1);

    // Did we wrap around? Then need to keep going!
    if (reverseEnd == -1) {
        reverseEnd = findFirstMiss(map,
                           glassCell, map.numAngleBins() - 1, map.beginBin(glassCell.x, glassCell.y) + startIdx, -1);
    }

    // The reverseEnd is always the start, so we don't want the first-miss, but the last-hit.
    IndexRange hitRange;
    hitRange.startIdx = reverseEnd + 1;

    // If the reverse search wraps or the forward search wraps, then the startIdx will be greater than the forward
    // search end (we've already handled the full range being visible). Thus, the size needs to account for the
    // wraparound that occurred
    if (hitRange.startIdx > forwardEnd) {
        hitRange.size = forwardEnd + (map.numAngleBins() - hitRange.startIdx);
    } else {
        // Otherwise, the start is the reverse end and the size is the distance to the forward end
        hitRange.size = forwardEnd - hitRange.startIdx;
    }

    return hitRange;
}


int RFMDynamicFilter::findFirstMiss(ReflectanceFieldMap& map, const Point<int>& glassCell,
                       int startIdx,
                                         ReflectanceFieldMap::BinConstIter end,
                       int direction) const
{
    int numSinceLastHit = 0;
    int firstMissIdx = startIdx;

    int binIdx = startIdx;
    for (auto binIt = map.beginBin(glassCell.x, glassCell.y) + startIdx, endIt = end; binIt != endIt;
         binIt += direction, binIdx += direction) {
        // If this isn't a hit, we still have the search expand through unobserved or less confident
        // angular regions
        if (!map.isHit(*binIt)) {
            // Set the miss index only after the first non-hit so that we don't extend the range in the
            // event that there are no misses, but too many unobserved
            if (numSinceLastHit == 0) {
                firstMissIdx = binIdx;
            }

//            if (map.isMiss(*binIt) || (numSinceLastHit > maxUnobservedForConnectivity_)) {
            if (numSinceLastHit > maxUnobservedForConnectivity_) {
                break;
            } else {
                ++numSinceLastHit;
            }
        } else {
            numSinceLastHit = 0;
        }
    }

    return firstMissIdx;
}

} // namespace rfm
