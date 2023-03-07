#pragma once

#include <memory>

#include <QMainWindow>
#include <QPushButton>
#include <QGridLayout>

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

#include "ros/ros.h"
#include "rviz/visualization_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_widget.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property_tree_delegate.h"
#include "rviz/display_factory.h"
#include "rviz/load_resource.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/panel_dock_widget.h"
#include "rviz/render_panel.h"
#include "rviz/tool.h"
#include "rviz/tool_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/tool_manager.h"
#include "rviz/panel.h"
#include "rviz/ogre_helpers/orthographic.h"
// #include "rviz/default_plugin/point_cloud_transformer.h"
// #include "rviz/default_plugin/ortho_proj_transformer.h"

// #define NO_CXX_EXTENSIONS
#ifdef NO_CXX_EXTENSIONS
	#define __restrict__ // find-replace "__restrict__" with ""
#endif

auto const marker_topic = "visualization_marker";

class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow() noexcept;

private:
	// C++ allows pointer aliasing by default, which sucks
	// GCC (which we are forced to use) supports an extension that fixes that
	// "no, compiler, I guarantee these pointers do will never alias anything (as far as you can tell)."
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
	Ogre::Matrix4                   view_matrix;
};