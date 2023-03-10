#include <cstdio>
#include <exception>
#include <cstdint>
#include <algorithm>
#include <thread>

#include "ros/ros.h"
#include <QApplication>
#include "sensor_msgs/PointCloud2.h"

#include "main_window.hpp"
#include "rviz_active_management.hpp"


std::unique_ptr<MainWindow> w;

// #define NODISPLAY
#define LOG

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