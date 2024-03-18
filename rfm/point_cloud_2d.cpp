#include "point_cloud_2d.h"

namespace rfm
{

PointCloud2D PointCloud2D::decimate(int factor) const
{
    PointCloud2D decimatedCloud;

    for (std::size_t i = 0; i < size(); i += factor) {
        decimatedCloud.rays_.push_back(rays_[i]);
    }

    return decimatedCloud;
}

}   // namespace rfm
