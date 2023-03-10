#include "main_window.hpp"
#include <QApplication>
#include <cstdio>
#include <exception>
#include <cstdint>
#include "sensor_msgs/PointCloud2.h"
#include <algorithm>
#include <thread>

#include "parameter_server.hpp"
#include "optional_ext.hpp"

std::unique_ptr<MainWindow> w;

// #define NODISPLAY
#define LOG

void update_view_matrix(const sensor_msgs::PointCloud2& cloud) {
	#pragma pack(1)
	struct point_t {
		float x, y, z, noise;
		int16_t intensity;
		int8_t gray;
	};
	auto const point_array = reinterpret_cast<point_t const* const>(cloud.data.data());
	float x_max = 0;
	#pragma omp order(concurrent) simd safelen(512) reduction(max:x_max)
	for (int i=0; i<cloud.data.size()/sizeof(point_t); i++) {
		auto const point     = point_array[i];
		auto const point_x   = std::abs(point.x);
		auto const point_y   = std::abs(point.y);
		auto const max_coord = std::max(point_x, point_y);

		x_max = std::max(x_max, max_coord);  // update max so far
	}
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

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "fotona_gui"); // small overhead
	ros::NodeHandle n;                   // likely best done on the main thread
	try {
#ifndef NODISPLAY
		QApplication a(argc, argv);          // Qt stuff, must be on the main thread
		w = std::unique_ptr<MainWindow>(new MainWindow());
#endif
		auto event_loop = std::thread([&n]() {
			auto const subscriber = n.subscribe("/pico_flexx/points", 128, update_view_matrix);
			auto rate = ros::Rate(1);
			auto alpha_parameter_key = std::string{"Alpha"};
			auto size_parameter_key = std::string{"Size"};
			n.param(alpha_parameter_key,   0.5f);
			n.param(size_parameter_key,    0.003f);
			while (ros::ok()) {
				ros::spinOnce();
				auto const alpha = get_parameter<float>(n, alpha_parameter_key);
				auto const size  = get_parameter<float>(n, size_parameter_key);

				map(alpha, [&](auto const alpha){ w->set_pointcloud_alpha(alpha); });
				map(size,  [&](auto const size ){ w->set_pointcloud_size(size);   });

				rate.sleep();
			}
		}); 
#ifndef NODISPLAY
		w->show();
		return a.exec();
#else
		event_loop.join();
#endif
	} catch (ros::InvalidNodeNameException) {
		std::printf("[err]: Please reinstall fotona_gui, it is likely broken beyond repair.");
	} catch (std::bad_alloc) {
		std::printf("[err]: Failed to allocate enough memory.");
	} catch (std::runtime_error e) {
		std::printf("[err]: %s", e.what());
	}
}