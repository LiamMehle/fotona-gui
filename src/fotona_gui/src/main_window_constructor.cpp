#include <exception>
#include "main_window.hpp"
#include "transparent_button.hpp"
#include <rviz/view_manager.h>
#include <rviz/visualization_frame.h>
#include <rviz/tool_manager.h>
#include <qstringlist.h>

#include <rviz/default_plugin/tools/selection_tool.h>
#include <rviz/selection/selection_manager.h>
#include <QGraphicsOpacityEffect>
#include <QDockWidget>

namespace RvizDisplayType {
	auto const Grid   = "rviz/Grid";
	auto const Marker = "rviz/Marker";
	auto const MarkerType = "visualization_msgs/Marker";
}

auto const rviz_fixed_frame            = "pico_flexx_optical_frame";
auto const pointcloud_select_tool_name = "rviz/PublishPoint";

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	this->setAttribute(Qt::WA_PaintOnScreen, false);
	this->setAttribute(Qt::WA_PaintUnclipped, false);
	// init
	// new throws if it fails to allocate, which is (or at least should be) handled by the caller.
	auto clear_button          = new TransparentButton("clear", this);  // raw pointers instead of smart pointers because Qt will take ownership
	auto scan_button           = new QPushButton("scan");
	auto start_button          = new QPushButton("start");
	auto stop_button           = new QPushButton("stop");
	auto main_layout           = new QHBoxLayout();
	auto button_layout         = new QVBoxLayout();
	auto button_container      = new QWidget();
	auto button_dock           = new QDockWidget();
	auto central_widget        = new QWidget();
	auto visualization_frame   = new rviz::VisualizationFrame();
	auto render_panel          = new rviz::RenderPanel(visualization_frame);
	auto visualization_manager = new rviz::VisualizationManager(render_panel, visualization_frame);
	render_panel->initialize(visualization_manager->getSceneManager(), visualization_manager);
	// Pointers can be indexed into and restrict is an non-standard. A reference fixes it.

	main_layout->addWidget(visualization_frame);
	central_widget->setLayout(main_layout);
	this->setCentralWidget(central_widget);

	// button layout/setup
	button_layout->addWidget(clear_button);
	button_layout->addWidget(scan_button);
	button_layout->addWidget(start_button);
	button_layout->addWidget(stop_button);
	button_container->setLayout(button_layout);
	button_dock->setWidget(button_container);
	// auto window_manager = visualization_manager->getWindowManager();
	// window_manager->addPane("controls", button_container, Qt::LeftDockWidgetArea);
	visualization_frame->addDockWidget(Qt::LeftDockWidgetArea, button_dock);
	visualization_frame->setCentralWidget(render_panel);

	auto opacity_effect = new QGraphicsOpacityEffect(this);
	opacity_effect->setOpacity(0.2);
	clear_button->setGraphicsEffect(opacity_effect);

	// fix up default layout
#define FIX(WIDGET) WIDGET->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding)
	FIX(clear_button);
	FIX(scan_button);
	FIX(start_button);
	FIX(stop_button);
	FIX(central_widget);
	FIX(render_panel);
#undef FIX

	// rviz setup
	
	visualization_manager->initialize();

	Ogre::Vector3 const origin(0, 0, 0);
	auto view_manager = visualization_manager->getViewManager();
	view_manager->setCurrentViewControllerType("rviz/TopDownOrtho");
	auto view_controller = view_manager->getCurrent();
	view_controller->subProp("X")->setValue(0.f);                // center the view
	view_controller->subProp("Y")->setValue(0.f);
	this->view_scale = view_controller->subProp("Scale");

	auto grid = visualization_manager->createDisplay(RvizDisplayType::Grid, "grid", true);
	if(grid == nullptr)
		throw std::runtime_error("Failed to create grid, something went terribly wrong");
	// grid->initialize(visualization_manager);

	this->pointcloud = visualization_manager->createDisplay("rviz/PointCloud2", "pico flexx pointcloud", true);
	if(pointcloud == nullptr)
		throw std::runtime_error("Failed to create point cloud, something went terribly wrong");
	// this->pointcloud->initialize(visualization_manager);
	this->pointcloud->subProp("Alpha")->setValue(0.f);
	this->pointcloud->subProp("Size (m)")->setValue(0.003f);
	this->pointcloud->subProp("Color Transformer")->setValue("Intensity");
	this->pointcloud->subProp("Channel Name")->setValue("intensity");
	this->pointcloud->setTopic("/pico_flexx/points", "sensor_msgs/PointCloud2");

	auto mesh = visualization_manager->createDisplay("rviz/Marker", "pico flexx reconstructed mesh", true);
	mesh->setTopic("/pico_flexx/mesh", "visualization_msgs/Marker");

	auto tool_manager = visualization_manager->getToolManager();
	auto pointcloud_select_tool = tool_manager->addTool(pointcloud_select_tool_name);
	if (pointcloud_select_tool == nullptr)
		throw std::runtime_error("pointcloud_select_tool is not of type rviz::SelectionTool");
	pointcloud_select_tool->initialize(visualization_manager);  // missing piece
	tool_manager->setDefaultTool(pointcloud_select_tool);
	tool_manager->setCurrentTool(pointcloud_select_tool);

	visualization_manager->setFixedFrame(rviz_fixed_frame);

	visualization_manager->startUpdate();  // begin asynchronous update of the vizualization
}