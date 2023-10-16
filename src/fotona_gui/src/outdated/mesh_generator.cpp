#include "mesh_generator.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include <vector>
#include <array>

char const* _source_topic = nullptr;
ros::Publisher publisher;
ros::Subscriber subscriber;

template<typename T, size_t N0, size_t N1>
constexpr
static
auto concat_arrays(std::array<T, N0> a, std::array<T, N1> b) -> std::array<T, N0+N1> {
	std::array<T, N0+N1> output;
	for (int i=0; i<N0; i++)
		output[i] = a[i];
	for (int i=0; i<N1; i++)
		output[N0+i] = b[i];
	return output;
};

static
void process_mesh(const sensor_msgs::PointCloud2& pointcloud) {
	#pragma pack(1)
	struct point_t {
		float x, y, z, noise;
		int16_t intensity;
		int8_t gray;
	};
	auto const point_array = reinterpret_cast<point_t const* const>(pointcloud.data.data());
	auto const point_count = pointcloud.data.size()/sizeof(point_t);
	size_t const m = pointcloud.height;
	size_t const n = pointcloud.width;
	// assuming point [0, 0] is top left
	// for every point not on the top or left edge, there are 2 tris
	size_t const tri_count = 2*(n-1)*(m-1);
	size_t const vertex_count = tri_count*3;
	visualization_msgs::Marker mesh;  // will be published as output
	mesh.type = visualization_msgs::Marker::TRIANGLE_LIST;
	// mesh.header = pointcloud.header;
	mesh.header.frame_id = pointcloud.header.frame_id;
	mesh.header.seq = pointcloud.header.seq;
	mesh.header.stamp = pointcloud.header.stamp;

	mesh.points.resize(vertex_count);     // resize instead of reserve to populate in parallel
	mesh.colors.resize(vertex_count);     // resize instead of reserve to populate in parallel
	mesh.ns = "mesh";
	mesh.id = 0;
	mesh.action = visualization_msgs::Marker::ADD;
	mesh.pose.orientation.x = 1;
	mesh.pose.orientation.y = 0;
	mesh.pose.orientation.z = 0;
	mesh.pose.orientation.w = 0;
	mesh.pose.position.x    = 0;
	mesh.pose.position.y    = 0;
	mesh.pose.position.z    = 0;
	mesh.scale.x            = 1;
	mesh.scale.y            = 1;
	mesh.scale.z            = 1;
	mesh.color.a            = 1;
	auto constexpr extract_coords = [](point_t const p) {
		return std::array<float, 3>{p.x, p.y, p.z};
	};
	auto constexpr extract_color = [](point_t const p) {
		std_msgs::ColorRGBA color;
		color.a = 1;
		color.r = static_cast<float>(p.intensity)/static_cast<float>(2048);
		color.g = static_cast<float>(p.intensity)/static_cast<float>(2048);
		color.b = static_cast<float>(p.intensity)/static_cast<float>(2048);
		return color;
	};
	auto constexpr array_to_point = [](std::array<float, 3> const a) {
		geometry_msgs::Point point;
		point.x = a[0];
		point.y = a[1];
		point.z = a[2];
		return point;
	};

	auto* mesh_points = mesh.points.data();
	auto* mesh_colors = mesh.colors.data();

	// #pragma omp parallel for num_threads(4) collapse(2) schedule(static)
	for (int i=1; i<m; i++) {
		for (int j=1; j<n; j++) {
			// for point [i,j], there are 2 tris
			auto const p00 = extract_coords(point_array[(i-0)*n + (j-0)]);  // root
			auto const p10 = extract_coords(point_array[(i-1)*n + (j-0)]);  // up
			auto const p01 = extract_coords(point_array[(i-0)*n + (j-1)]);  // left
			auto const p11 = extract_coords(point_array[(i-1)*n + (j-1)]);  // up-left
			auto const c00 = extract_color( point_array[(i-0)*n + (j-0)]);  // root
			auto const c10 = extract_color( point_array[(i-1)*n + (j-0)]);  // up
			auto const c01 = extract_color( point_array[(i-0)*n + (j-1)]);  // left
			auto const c11 = extract_color( point_array[(i-1)*n + (j-1)]);  // up-left
			std::array const tri0    = {p00, p11, p10};
			std::array const tri1    = {p00, p01, p11};
			std::array const colors0 = {c00, c11, c10};
			std::array const colors1 = {c00, c01, c11};
			std::array const points = concat_arrays(tri0,    tri1);
			std::array const colors = concat_arrays(colors0, colors1);
			auto const initial_offset = (i*m + j)*points.size();
			for (int k=0; k<points.size(); k++) {
				mesh_points[initial_offset+k] = array_to_point(points[k]);
				mesh_colors[initial_offset+k] = colors[k];
			}
		}
	}
	publisher.publish(mesh);
}

void setup_mesh_generator(ros::NodeHandle& n, char const* const destination_topic, char const* const source_topic) {
	_source_topic = source_topic;
	subscriber = n.subscribe(source_topic, 1024, process_mesh);
	publisher = n.advertise<visualization_msgs::Marker>(destination_topic, 1024);
}