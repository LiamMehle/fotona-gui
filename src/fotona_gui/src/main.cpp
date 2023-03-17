#include <cstdio>
#include <exception>
#include <cstdint>
#include <algorithm>
#include <thread>

#include "ros/ros.h"
#include <QApplication>
#include "sensor_msgs/PointCloud2.h"

#include "main_window.hpp"
#include "ros_event_loop.hpp"

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
		auto event_loop = std::thread([&n, &a]() {
			ros_event_loop(n, *w);
			a.exit(0);
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