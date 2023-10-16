#pragma once

#include <memory>

#include <QMainWindow>
#include <QStackedWidget>
#include <QGridLayout>
#include <QPushButton>
#include <QObject>
#include <QStackedLayout>
#include "rviz_bundle.hpp"
#include "ros/ros.h"
// #include <QPushButton>
// #include <QGridLayout>
// #include <rviz/properties/property.h>
// #include "ros/ros.h"
// // Visualization_manager.h uses `register`,
// // which GCC flags a warning for as not existing in C++17,
// // which catkin converts into an error via GCC's -Werror flag.
// // Pragmas are (according to ISO) ignored if unsupported
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wregister"
// #include "rviz/visualization_manager.h"
// #pragma GCC diagnostic pop
// #include "rviz/render_panel.h"
// #include "rviz/display.h"

// #include "rviz/viewport_mouse_event.h"
// #include "rviz/view_manager.h"

// #define NO_CXX_EXTENSIONS
#ifdef NO_CXX_EXTENSIONS
	#define __restrict__ // find-replace "__restrict__" with ""
#endif

// auto const camera_position = Ogre::Vector3(0, 0, 5);  // Ogre does not support constexpr

enum Mode {
	Undefined,
	Torso,
	Limb
};

enum Screen {
	ModeSelect,
	Align,
	DrawBorder,
	ConfirmBorder,
	Execute,
	Finished
};

struct AppState {
	Screen screen;
	Mode mode;
};

class MainWindow : public QMainWindow {
	Q_OBJECT

private:
	// most widgets are pre-created and stored to be set as the central widget as needed
	AppState state;               // state of the app 
	QPushButton* left_button;
	QPushButton* right_button;
	RvizBundle rviz_bundle;       // all rviz-specific objects
	QStackedLayout* root_layout;  // used to switch between the different types of layout in the application
	std::unique_ptr<QObject> button_context;      // context object used to remove functors connected to buttons for reuse purposes
	ros::NodeHandle& n;
	// std::vector<> boundary;
public:
	// rviz objects/widgets that need to be handled separately from the Qt stuff
	// RvizBundle rviz_bundle;
	// QGridLayout* main_layout;
	// QWidget* layout_widget;  // reference to the button/ui container layered over rviz render panel.
public:
	MainWindow(ros::NodeHandle& n, QWidget *parent = nullptr);
	~MainWindow() noexcept;
	// void set_view_matrix(Ogre::Matrix4);
	// void set_pointcloud_alpha(float alpha);
	// void set_pointcloud_size(float size);
	// void set_pointcloud_color_transformer(char const* const transformer);
	// void set_view_scale(float scale);
	void transition_into(Screen);
	void transition_into(Mode, Screen);

private:
	void update_state();
	void add_point() noexcept;
	// Ogre::Camera* camera;

	// // `Display` in this context meaning something that is shown or element in the reneder pannel.
	// rviz::Display*  __restrict__ pointcloud;
	// rviz::Property* __restrict__ view_scale;
	// // cached values becaues updating them regardless is just too expensive
	// float pointcloud_alpha;
	// float pointcloud_size;
	// // camera projection is
};

// // https://en.wikipedia.org/wiki/Orthographic_projection
// inline
// Ogre::Matrix4 calculate_projection_matrix(
// 	float const left,
// 	float const right,
// 	float const bottom,
// 	float const top,
// 	float const near,
// 	float const far
// ) {
// 	return Ogre::Matrix4(
// 		2.f/(right-left),   0.f,              0.f,            -(right + left)/(right-left),
// 		0.f,                2.f/(top-bottom), 0.f,            -(top+bottom)/(top-bottom),
// 		0.f,                0.f,              2.f/(far-near), -(far+near)/(far-near),
// 		0.f,                0.f,              0.f,             1.f
// 	);
// }