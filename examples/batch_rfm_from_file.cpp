#include "rfm/rfm_builder.h"

#include <fstream>
#include <iostream>

using namespace rfm;

std::vector<PointCloud2D> load_point_clouds(const std::string& filename)
{
    std::ifstream cloudFile(filename);

    std::vector<PointCloud2D> clouds;

    // TODO: Read the point clouds from a simple text file format

    return clouds;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: rfm-from-file <name of file with laser data>\n";
        return 0;
    }

    const auto clouds = load_point_clouds(argv[1]);

    // TODO: Build the RFM by iterating through the clouds and passing them in for
    // construction one at a time.

    return 0;
}
