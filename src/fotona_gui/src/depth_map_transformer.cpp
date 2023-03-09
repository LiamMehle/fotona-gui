#include "depth_map_transformer.hpp"

// sets up any properties when loaded
void CustomDepthMapTransformer::createProperties(rviz::Property* parent_property, uint32_t mask, QList<rviz::Property*>& out_props) {}
// preference/priority. Should return 0 for flat color, but for custom embeded applications, it returns 255.
uint8_t CustomDepthMapTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud)    { return ~0; }
uint8_t CustomDepthMapTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud) { return ~0;}

// ignores `mask`, assume little-endian
bool CustomDepthMapTransformer::transform(sensor_msgs::PointCloud2ConstPtr const& cloud_pointer, uint32_t mask, const Ogre::Matrix4& transform, rviz::V_PointCloudPoint& out) {
    auto const& cloud  = *(cloud_pointer.get());
    // todo: check that the layout is:
    // all float32: x, y, z, noise, intensity, gray
    struct point {
        float x, y, z, noise, intensity, gray;
    };
    // hacky stuff for convenience: telling the compiler that this is actually a bunch of structured sigle-precision floats
    auto const  input_points = reinterpret_cast<point const* const __restrict__>(cloud.data.data());
    auto       output_points = reinterpret_cast<point      * const __restrict__>(out.data());
    auto highest_z = input_points[0].z;
    auto  lowest_z = input_points[0].z;
    for (int i=1; i<cloud.data.size(); i++) {
        highest_z = std::max(highest_z, input_points[i].z);
            lowest_z = std::min( lowest_z, input_points[i].z);
    }
    for (int i=0; i<cloud.data.size(); i++) {
        auto point = input_points[i];
        point.z -= lowest_z;
        point.gray = point.z / (highest_z - lowest_z);
        output_points[i] = point;
    }
    return true;
}
