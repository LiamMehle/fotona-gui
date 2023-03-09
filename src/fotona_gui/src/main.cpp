#include "main_window.hpp"
#include <QApplication>
#include <cstdio>
#include <exception>
#include <cstdint>
#include "sensor_msgs/PointCloud2.h"
#include <algorithm>
#include <thread>

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
	float x_min = 0,
	      x_max = 0,
	      y_min = 0,
	      y_max = 0;
	#pragma omp order(concurrent) simd safelen(512) reduction(max:x_max)
	for (int i=0; i<cloud.data.size()/sizeof(point_t); i++) {
		auto const point = point_array[i];
		x_max = std::max(std::abs(point.y),std::max(std::abs(point.x), x_max));
	}
	x_max =  x_max;
	y_max =  x_max;

	x_min = -x_max;
	y_min = -y_max;
	// x is back
	// y is right
	// z is up
	// from top-down perspective
	// x is down
	// y is right
	// z is towards camera
#ifndef NODISPLAY
	w->set_view_matrix(calculate_projection_matrix(y_min, y_max, x_min, x_max, 0.001, 10000.f));
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
			ros::TransportHints hints;
			hints.tcpNoDelay(true);
			auto const subscriber = n.subscribe("/pico_flexx/points", 1, update_view_matrix, hints);
			auto rate = ros::Rate(33);
			ros::spin();
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