#include <cstdio>
#include <exception>
#include <cstdint>
#include <algorithm>
#include <thread>
#include <memory>

#include "ros/ros.h"
#include <QApplication>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"

#include "main_window.hpp"

std::unique_ptr<MainWindow> w;

#define LOG

template<typename T>
constexpr static inline
auto identity(T const x) -> T {return x;};

static
ros::Publisher perimeter_publisher;
static
ros::Subscriber extend_perimeter_subscriber;

void extend_perimeter(geometry_msgs::PointStamped point) {
	puts("extending perimeter");
	// insert point into description of polygon
	geometry_msgs::Point32 p;
	p.x = static_cast<float>(point.point.x);
	p.y = static_cast<float>(point.point.y);
	p.z = static_cast<float>(point.point.z);
	w->perimeter_points.push_back(p);

	// upload polygon to rviz machinery
	geometry_msgs::PolygonStamped polygon;
	polygon.header = point.header;
	std::transform(w->perimeter_points.begin(), w->perimeter_points.end(), std::back_inserter(polygon.polygon.points), identity<geometry_msgs::Point32>);
	perimeter_publisher.publish(polygon);
}

int main(int argc, char *argv[]) {
	try {
		puts("init");
		ros::init(argc, argv, "fotona_gui");
		ros::NodeHandle n; //= ros::NodeHandle("fotona_gui");
		extend_perimeter_subscriber = n.subscribe("/clicked_point", 8, extend_perimeter);
		perimeter_publisher = n.advertise<geometry_msgs::PolygonStamped>("perimeter", 8, false);
		QApplication a(argc, argv);
		w = std::make_unique<MainWindow>();

		std::thread ros_event_loop([]{ros::spin();});

		puts("running");
		return a.exec();
	} catch (ros::InvalidNodeNameException) {
		std::printf("[err]: Please reinstall fotona_gui, it is likely broken beyond repair.");
	} catch (ros::InvalidNameException) {
		std::printf("[err]: failed to create /perimeter topic");
	} catch (std::bad_alloc) {
		std::printf("[err]: Failed to allocate enough memory.");
	} catch (std::runtime_error e) {
		std::printf("[err]: %s", e.what());
	}
}