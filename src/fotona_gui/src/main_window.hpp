#pragma once

#include <memory>

#include <QMainWindow>
#include <QPushButton>
#include <QGridLayout>

#include "ros/ros.h"
// Visualization_manager.h uses `register`,
// which GCC flags a warning for as not existing in C++17,
// which catkin converts into an error via GCC's -Werror flag.
// Pragmas are (according to ISO) ignored if unsupported
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include "rviz/visualization_manager.h"
#pragma GCC diagnostic pop
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "rviz/viewport_mouse_event.h"
#include "rviz/view_manager.h"

// #define NO_CXX_EXTENSIONS
#ifdef NO_CXX_EXTENSIONS
	#define __restrict__ // find-replace "__restrict__" with ""
#endif

auto const camera_position = Ogre::Vector3(0, 0, 5);  // Ogre does not support constexpr

class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow() noexcept;
	void set_view_matrix(Ogre::Matrix4);
	void set_pointcloud_alpha(float alpha);
	void set_pointcloud_size(float size);
	void set_pointcloud_color_transformer(char const* const transformer);

private:
	rviz::Display*            pointcloud;
	// CustomDepthMapTransformer       depth_map_transformer;
	Ogre::Camera*             camera;
	// C++ allows pointer aliasing by default
	// GCC (which we are forced to use) supports an extension that fixes that
	QPushButton* __restrict__ clear_button;
	QPushButton* __restrict__ scan_button;
	QPushButton* __restrict__ start_button;
	QPushButton* __restrict__ stop_button;
	QGridLayout* __restrict__ main_layout;
	QWidget*     __restrict__ central_widget;

	std::unique_ptr<rviz::VisualizationManager> manager;  // still needs to be freed manually, but C++ has a thing for that
	rviz::RenderPanel* __restrict__ render_panel;
	// `Display` in this context meaning something that is shown or element in the reneder pannel.
	rviz::Display*     __restrict__ grid;

	// cached values becaues updating them regardless is just too expensive
	float pointcloud_alpha;
	float pointcloud_size;
	// camera projection is
};

// https://en.wikipedia.org/wiki/Orthographic_projection
inline
Ogre::Matrix4 calculate_projection_matrix(
	float const left,
	float const right,
	float const bottom,
	float const top,
	float const near,
	float const far
) {
	return Ogre::Matrix4(
		2.f/(right-left),   0.f,              0.f,            -(right + left)/(right-left),
		0.f,                2.f/(top-bottom), 0.f,            -(top+bottom)/(top-bottom),
		0.f,                0.f,              2.f/(far-near), -(far+near)/(far-near),
		0.f,                0.f,              0.f,             1.f
	);
}