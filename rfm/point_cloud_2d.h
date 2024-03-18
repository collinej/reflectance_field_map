#ifndef RFM_POINT_CLOUD_2D_H
#define RFM_POINT_CLOUD_2D_H

#include "line.h"
#include "point.h"
#include <cereal/access.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/vector.hpp>
#include <cstdint>
#include <type_traits>

/**
 * @file
 * @author  Collin Johnson
 *
 * PointCloud2D is a collection of lidar rays in 2D. The rays are not necessarily all taken from the same
 * sensor, but will contain all values gathered within a given block of time. Rays may optionally be
 * associated with a line in the environment. This line is extracted from the raw laser data and provides
 * an estimate of the surface (and its normal) that was measured by the lidar.
 */

namespace rfm
{

struct pose_t;

struct ray_2d_t
{
    int64_t timestamp;        ///< Time this ray was gathered
    Point<float> origin;      ///< Position of sensor generating this ray in the cloud's reference frame
    Point<float> endpoint;    ///< Endpoint of the ray in the cloud's reference frame
    float range;              ///< Range measured by the ray (< 0 = error)
    float angle;              ///< Angle of ray in cloud's reference frame
    uint16_t intensity = 0;   ///< Measured intensity of the ray (default is 0, not all logs have intensity)

    ray_2d_t() = default;

    ray_2d_t(int64_t timestamp, const Point<float>& origin, const Point<float>& endpoint, uint16_t intensity = 0)
    : timestamp(timestamp)
    , origin(origin)
    , endpoint(endpoint)
    , range(distance_between_points(origin, endpoint))
    , angle(angle_to_point(origin, endpoint))
    , intensity(intensity)
    {
    }
};

template <class Archive>
void serialize(Archive& ar, ray_2d_t& ray)
{
    ar(ray.timestamp, ray.origin, ray.endpoint, ray.range, ray.angle, ray.intensity);
}

/**
 * PointCloud2D holds laser rays collected by one or more lidars on the robot.
 *
 * All rays added to the point cloud are assumed to be in the same reference frame.
 *
 * Each ray contains enough information to be used individually.
 */
class PointCloud2D
{
public:
    using RayContainer = std::vector<ray_2d_t>;
    using const_iterator = RayContainer::const_iterator;

    PointCloud2D() = default;

    /**
     * Constructor a new PointCloud2D from a sequence of rays.
     */
    template <class Iter>
    PointCloud2D(Iter begin, Iter end) : rays_(begin, end)
    {
        static_assert(std::is_same_v<typename Iter::value_type, ray_2d_t>);
    }

    ~PointCloud2D() = default;

    /**
     * Add a sequence of rays to the point cloud.
     *
     * @tparam  Iter    iterator over container of type <ray_2d_t>
     * @param   begin   Start of ray range
     * @param   end     End of ray range
     */
    template <class Iter>
    void addRays(Iter begin, Iter end)
    {
        static_assert(std::is_same_v<typename Iter::value_type, ray_2d_t>);
        std::copy(begin, end, std::back_inserter(rays_));
    }

    void addRay(const ray_2d_t &ray)
    {
        rays_.push_back(ray);
    }

    /**
     * Decimate the point cloud by the given factor. A factor of 1 will just return a copy of the point cloud.
     * Decimation factors > 1 reduce point cloud to size() / factor.
     */
    PointCloud2D decimate(int factor) const;

    /**
     * Retrieve timestamp of the point cloud.
     *
     * The timestamp is the time of the earliest measurement in the point cloud. If no returns, the
     * timestamp is just 0.
     */
    int64_t timestamp() const { return rays_.empty() ? INT64_C(0) : rays_.front().timestamp; }

    // Iterate through the scan
    std::size_t size() const { return rays_.size(); }
    bool empty() const { return rays_.empty(); }

    const_iterator begin() const { return rays_.begin(); }
    const_iterator end() const { return rays_.end(); }

    const ray_2d_t& front() const { return rays_.front(); }
    const ray_2d_t& back() const { return rays_.back(); }

    const ray_2d_t& operator[](int n) const { return rays_[n]; }

    /**
     * Empty out the point cloud.
     */
    void clear() { rays_.clear(); }

private:
    std::vector<ray_2d_t> rays_;

    //////////     Serialization support     //////////
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(rays_);
    }
};

}   // namespace rfm

#endif   // RFM_POINT_CLOUD_2D_H
