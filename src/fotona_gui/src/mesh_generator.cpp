#include "mesh_generator.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include <vector>
#include <array>

char const* _source_topic = nullptr;
ros::Publisher publisher;

template<typename T, size_t N0, size_t N1>
constexpr
auto concat_arrays(std::array<T, N0> a, std::array<T, N1> b) -> std::array<T, N0+N1> {
	std::array<T, N0+N1> output;
	for (int i=0; i<N0; i++)
		output[i] = a[i];
	for (int i=0; i<N1; i++)
		output[N0+i] = b[i];
	return output;
};

void process_mesh(const sensor_msgs::PointCloud2& pointcloud) {
	#pragma pack(1)
	struct point_t {
		float x, y, z, noise;
		int16_t intensity;
		int8_t gray;
	};
	auto const point_array = reinterpret_cast<point_t const* const>(pointcloud.data.data());
	auto const point_count = pointcloud.data.size()/sizeof(point_t);
	size_t const n = pointcloud.height;
	size_t const m = pointcloud.width;
	assert(n*m == point_count);
	// assuming point [0, 0] is top left
	// for every point not on the top or left edge, there are 2 tris
	size_t const tri_count = 2*(n-1)*(m-1);
	size_t const vertex_count = tri_count*3;
	visualization_msgs::Marker mesh;  // will be published as output
	mesh.type = visualization_msgs::Marker::TRIANGLE_LIST;
	mesh.header = pointcloud.header;
	mesh.points.resize(vertex_count);     // resize instead of reserve to populate in parallel
	mesh.ns = "mesh";
	mesh.id = 0;
	mesh.action = visualization_msgs::Marker::ADD;
	// mesh.pose.orientation;
	// mesh.pose.position;
	// mesh.scale
	mesh.color.a = 1;
	auto const extract_coords = [](point_t const p) {
		return std::array<float, 3>{p.x, p.y, p.z};
	};
	auto const extract_color = [](point_t const p) {
		std_msgs::ColorRGBA color;
		color.a = 1;
		color.r = p.intensity;
		color.g = p.intensity;
		color.b = p.intensity;
		return color;
	};
	auto const array_to_point = [](std::array<float, 3> const a) {
		geometry_msgs::Point point;
		point.x = a[0];
		point.y = a[1];
		point.z = a[2];
		return point;
	};

	#pragma omp parallel for num_threads(2) collapse(2) schedule(static)
	for (int i=0; i<m; i++) {
		for (int j=0; j<n; j++) {
			// for point [i,j], there are 2 tris
			auto const p0 = extract_coords(point_array[(i-0)*m + (j-0)]);  // root
			auto const p1 = extract_coords(point_array[(i-1)*m + (j-0)]);  // up
			auto const p2 = extract_coords(point_array[(i-0)*m + (j-1)]);  // left
			auto const p3 = extract_coords(point_array[(i-1)*m + (j-1)]);  // up-left
			auto const c0 = extract_color( point_array[(i-0)*m + (j-0)]);  // root
			auto const c1 = extract_color( point_array[(i-1)*m + (j-0)]);  // up
			auto const c2 = extract_color( point_array[(i-0)*m + (j-1)]);  // left
			auto const c3 = extract_color( point_array[(i-1)*m + (j-1)]);  // up-left
			std::array const tri0    = {p0, p1, p3};
			std::array const tri1    = {p0, p2, p3};
			std::array const colors0 = {c0, c1, c3};
			std::array const colors1 = {c0, c2, c3};
			std::array const points = concat_arrays(tri0,    tri1);
			std::array const colors = concat_arrays(colors0, colors1);
			auto const initial_offset = i*m + j;
			for (int k=0; k<points.size(); k++) {
				mesh.points[initial_offset+k] = array_to_point(points[k]);
				mesh.colors[initial_offset+k] = colors[k];
			}
		}
	}
	publisher.publish(mesh);
}

void setup_mesh_generator(ros::NodeHandle n, char const* const destination_topic, char const* const source_topic) {
	_source_topic = source_topic;
	n.subscribe(source_topic, 1024, process_mesh);
	publisher = n.advertise<visualization_msgs::Marker>(destination_topic, 1024);
}