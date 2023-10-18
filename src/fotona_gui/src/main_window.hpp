#pragma once

#include <memory>

#include <QMainWindow>
#include <QLabel>
#include <QGridLayout>
#include <QPushButton>
#include <QObject>
#include <QStackedLayout>
// #include "rviz_bundle.hpp"
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

enum Stage {
    Position,
    ConfigurePerimiter,
    Run,
    Done
};

class MainWindow : public QMainWindow {
	Q_OBJECT
private:
	QPushButton* button_left;
	QPushButton* button_right;
	QLabel*   status_text;
public:
	MainWindow();
	~MainWindow();
private:
	void transition(Stage);
};