#include <exception>
#include "main_window.hpp"
#include "rviz/view_manager.h"
#include "rviz/visualization_frame.h"
#include "rviz/tool_manager.h"
#include <qstringlist.h>

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
	auto clear_button   = new QPushButton();
	auto scan_button    = new QPushButton();
	auto start_button   = new QPushButton();
	auto stop_button    = new QPushButton();
	auto main_layout    = new QGridLayout();
	auto central_widget  = new QWidget();
	auto visualization_frame = new rviz::VisualizationFrame();

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
	main_layout->addWidget(visualization_frame, 0, 1, 4, 1);


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
	
	// this->manager->initialize();
	// this->grid = this->manager->createDisplay(RvizDisplayType::Grid, "top-down orthogonal", true);
	// if(grid == nullptr)
	// 	throw std::runtime_error("Failed to create grid, something went terribly wrong");

	// auto& tool_manager = *this->manager->getToolManager();
	// auto tool_classes = tool_manager.getToolClasses();
	// auto const tool_classes_count = tool_classes.size();
	// for(int i=0; i<tool_classes_count; i++)
	// 	printf("tool: %s\n",  tool_classes[i]);
	// auto& pointcloud_select_tool = *tool_manager.addTool(pointcloud_select_tool_name);
	// tool_manager.setCurrentTool(&pointcloud_select_tool);

	// Ogre::Vector3 const origin(0, 0, 0);
	// auto& view_manager = *(this->manager->getViewManager()->getCurrent());
	// this->camera       = view_manager.getCamera();
	// camera->setPosition(camera_position);
	// // view_manager.lookAt(origin);
	// // setting up a valid view transform such that the camera won't complain
	// auto view_matrix = calculate_projection_matrix(-1, 1, -1, 1, 0.001, 1000);
	// camera->setCustomViewMatrix(true, view_matrix);  // this will be updated in `set_view_matrix`

	// this->pointcloud = this->manager->createDisplay("rviz/PointCloud2", "pico flexx pointcloud", true);
	// this->manager->setFixedFrame(rviz_fixed_frame);
	// this->pointcloud->subProp("Alpha")->setValue(0.5f);
	// this->pointcloud->subProp("Size (m)")->setValue(0.003f);
	// this->pointcloud->subProp("Color Transformer")->setValue("AxisColor");
	// this->pointcloud->subProp("Axis")->setValue("Z");
	// pointcloud->setTopic("/pico_flexx/points", "sensor_msgs/PointCloud2");

	// this->manager->startUpdate();  // begin asynchronous update of the vizualization
}