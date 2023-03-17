#include <exception>
#include "main_window.hpp"
#include <rviz/view_manager.h>
#include <rviz/visualization_frame.h>
#include <rviz/tool_manager.h>
#include <qstringlist.h>

#include <rviz/default_plugin/tools/selection_tool.h>
#include <rviz/selection/selection_manager.h>

namespace RvizDisplayType {
	auto const Grid   = "rviz/Grid";
	auto const Marker = "rviz/Marker";
	auto const MarkerType = "visualization_msgs/Marker";
}

auto const rviz_fixed_frame            = "pico_flexx_optical_frame";
auto const pointcloud_select_tool_name = "rviz/PublishPoint";

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	// init
	// new throws if it fails to allocate, which is (or at least should be) handled by the caller.
	auto clear_button          = new QPushButton(parent);  // raw pointers instead of smart pointers because Qt will take ownership
	auto scan_button           = new QPushButton(parent);
	auto start_button          = new QPushButton(parent);
	auto stop_button           = new QPushButton(parent);
	auto main_layout           = new QGridLayout(parent);
	auto central_widget        = new QWidget(parent);
	auto visualization_frame   = new rviz::VisualizationFrame(parent);
	auto render_panel          = new rviz::RenderPanel(visualization_frame);
	auto visualization_manager = new rviz::VisualizationManager(render_panel, visualization_frame);
	render_panel->initialize(visualization_manager->getSceneManager(), visualization_manager);
	// Pointers can be indexed into and restrict is an non-standard. A reference fixes it.

	central_widget->setLayout(main_layout);
	this->setCentralWidget(central_widget);

	// set button text
	clear_button->setText("clear");
	scan_button->setText("scan");
	start_button->setText("start");
	stop_button->setText("stop it, get some help");

	// callbacks are connected to buttons like so:
	// this->connect(this->clear_button, &QPushButton::clicked, this,
	// 	[this](){ puts("button has been clicked!"); });

	// layout setup
	main_layout->addWidget(clear_button, 0, 0, 1, 1);
	main_layout->addWidget(scan_button,  1, 0, 1, 1);
	main_layout->addWidget(start_button, 2, 0, 1, 1);
	main_layout->addWidget(stop_button,  3, 0, 1, 1);
	main_layout->addWidget(render_panel, 0, 1, 4, 1);


	// fix up default layout
#define FIX(WIDGET) WIDGET->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding)
	FIX(clear_button);
	FIX(scan_button);
	FIX(start_button);
	FIX(stop_button);
	FIX(central_widget);
	FIX(render_panel);
#undef FIX

	for(int i=0; i<main_layout->rowCount(); i++)
		main_layout->setRowStretch(i, 1);
	main_layout->setColumnStretch(0, 1);
	main_layout->setColumnStretch(1, 4);

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
	grid->initialize(visualization_manager);

	this->pointcloud = visualization_manager->createDisplay("rviz/PointCloud2", "pico flexx pointcloud", true);
	if(pointcloud == nullptr)
		throw std::runtime_error("Failed to create point cloud, something went terribly wrong");
	this->pointcloud->initialize(visualization_manager);
	this->pointcloud->subProp("Alpha")->setValue(0.5f);
	this->pointcloud->subProp("Size (m)")->setValue(0.003f);
	this->pointcloud->subProp("Color Transformer")->setValue("Intensity");
	this->pointcloud->subProp("Channel Name")->setValue("intensity");
	this->pointcloud->setTopic("/pico_flexx/points", "sensor_msgs/PointCloud2");

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