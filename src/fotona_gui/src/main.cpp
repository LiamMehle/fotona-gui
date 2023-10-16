#include <cstdio>
#include <exception>
#include <cstdint>
#include <algorithm>
#include <thread>

#include "ros/ros.h"
#include <QApplication>
#include "sensor_msgs/PointCloud2.h"

#include "main_window.hpp"
// #include "ros_event_loop.hpp"

std::unique_ptr<MainWindow> w;

#define LOG

int main(int argc, char *argv[]) {
	// auto event_loop = std::thread([&n, &a]() {
	// 	// ros_event_loop(n, *w);
	// 	a.exit(0);
	// });
	try {
		puts("init");
		ros::init(argc, argv, "fotona_gui");
		ros::NodeHandle n;
		QApplication a(argc, argv);
		w = std::make_unique<MainWindow>();

		return a.exec();
	} catch (ros::InvalidNodeNameException) {
		std::printf("[err]: Please reinstall fotona_gui, it is likely broken beyond repair.");
	} catch (std::bad_alloc) {
		std::printf("[err]: Failed to allocate enough memory.");
	} catch (std::runtime_error e) {
		std::printf("[err]: %s", e.what());
	}
}