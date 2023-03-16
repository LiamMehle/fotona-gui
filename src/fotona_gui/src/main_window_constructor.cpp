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
// apparently another option is PlantFlag
auto const pointcloud_select_tool_name = "rviz/PublishPoint";

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	// init
	// new throws if it fails to allocate, which is (or at least should be) handled by the caller.
	auto clear_button   = new QPushButton(parent);
	auto scan_button    = new QPushButton(parent);
	auto start_button   = new QPushButton(parent);
	auto stop_button    = new QPushButton(parent);
	auto main_layout    = new QGridLayout(parent);
	auto central_widget  = new QWidget(parent);
	auto visualization_frame = new rviz::VisualizationFrame(parent);
	auto render_panel = new rviz::RenderPanel(parent);
	render_panel->setMouseTracking(true);
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
#undef FIX

	for(int i=0; i<main_layout->rowCount(); i++)
		main_layout->setRowStretch(i, 1);
	main_layout->setColumnStretch(0, 1);
	main_layout->setColumnStretch(1, 4);

	// rviz setup
	
	visualization_manager->initialize();

	Ogre::Vector3 const origin(0, 0, 0);
	auto view_manager = visualization_manager->getViewManager()->getCurrent();
	this->camera       = view_manager->getCamera();
	camera->setPosition(camera_position);
	view_manager->lookAt(origin);

	auto grid = visualization_manager->createDisplay(RvizDisplayType::Grid, "grid", true);
	if(grid == nullptr)
		throw std::runtime_error("Failed to create grid, something went terribly wrong");

	this->pointcloud = visualization_manager->createDisplay("rviz/PointCloud2", "pico flexx pointcloud", true);
	if(pointcloud == nullptr)
		throw std::runtime_error("Failed to create point cloud, something went terribly wrong");
	this->pointcloud->subProp("Alpha")->setValue(0.5f);
	this->pointcloud->subProp("Size (m)")->setValue(0.003f);
	this->pointcloud->subProp("Color Transformer")->setValue("AxisColor");
	this->pointcloud->subProp("Axis")->setValue("Z");
	this->pointcloud->setTopic("/pico_flexx/points", "sensor_msgs/PointCloud2");
	auto tool_manager = visualization_manager->getToolManager();
	rviz::SelectionTool* pointcloud_select_tool = static_cast<rviz::SelectionTool*>(tool_manager->addTool(pointcloud_select_tool_name));
	if (pointcloud_select_tool == nullptr)
		throw std::runtime_error("pointcloud_select_tool is not of type rviz::SelectionTool");
	tool_manager->setCurrentTool(pointcloud_select_tool);
	pointcloud_select_tool->activate();
	pointcloud_select_tool->initialize(visualization_manager);  // missing piece
	tool_manager->setParent(parent);
	puts("---------------------------------");
	printf("ClassId:     %s\n", tool_manager->getCurrentTool()->getClassId().toLocal8Bit().data());
	printf("Name:        %s\n", tool_manager->getCurrentTool()->getName().toLocal8Bit().data());
	printf("Description: %s\n", tool_manager->getCurrentTool()->getDescription().toLocal8Bit().data());
	puts("---------------------------------");
	// auto& tool_manager = *this->manager->getToolManager();
	// auto tool_classes = tool_manager.getToolClasses();
	// auto const tool_classes_count = tool_classes.size();
	// for(int i=0; i<tool_classes_count; i++)
	// 	printf("tool: %s\n",  tool_classes[i]);
	// auto& pointcloud_select_tool = *tool_manager.addTool(pointcloud_select_tool_name);
	// tool_manager.setCurrentTool(&pointcloud_select_tool);

	// // setting up a valid view transform such that the camera won't complain
	// auto view_matrix = calculate_projection_matrix(-1, 1, -1, 1, 0.001, 1000);
	// camera->setCustomViewMatrix(true, view_matrix);  // this will be updated in `set_view_matrix`

	auto selection_manager = visualization_manager->getSelectionManager();
	selection_manager->enableInteraction(true);

	visualization_manager->setFixedFrame(rviz_fixed_frame);

	visualization_manager->startUpdate();  // begin asynchronous update of the vizualization
}