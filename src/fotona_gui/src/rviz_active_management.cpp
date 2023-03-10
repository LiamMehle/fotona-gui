
#include "rviz_active_management.hpp"

extern std::unique_ptr<MainWindow> w;
void update_view_matrix(const sensor_msgs::PointCloud2& cloud) {
	#pragma pack(1)
	struct point_t {
		float x, y, z, noise;
		int16_t intensity;
		int8_t gray;
	};
	auto const point_array = reinterpret_cast<point_t const* const>(cloud.data.data());
	auto const point_count = cloud.data.size()/sizeof(point_t);
	std::vector<float> distances;
	distances.reserve(point_count);
	for (int i=0; i<point_count; i++) {
		auto const point     = point_array[i];
		auto const point_x   = std::abs(point.x);
		auto const point_y   = std::abs(point.y);
		auto const max_coord = std::max(point_x, point_y);
		distances[i] = max_coord;
	}
	// keep at least 80% of the points in view
	size_t const ignored_point_count = point_count * 2 / 10;
	auto const ignored_point_iterator = distances.begin()+ignored_point_count;
	std::nth_element(distances.begin(), ignored_point_iterator, distances.end(), [](float const a, float const b) {return a > b;});
	auto const x_max_measured = *(ignored_point_iterator + 1);
	auto const x_max = x_max_measured * 4;
	auto const y_max =  x_max;

	auto const x_min = -x_max;
	auto const y_min = -y_max;

	auto const z_min = 0.001f;
	auto const z_max = 1000.f;
	// x is back
	// y is right
	// z is up
	// from top-down perspective
	// x is down
	// y is right
	// z is towards camera
#ifndef NODISPLAY
	w->set_view_matrix(calculate_projection_matrix(y_min, y_max, x_min, x_max, z_min, z_max));
#endif
#ifdef LOG
	printf("x: [%.4f|%.4f]\t y: [%.4f|%.4f]\n",
		x_min,
		x_max,
		y_min,
		y_max);
#endif
}