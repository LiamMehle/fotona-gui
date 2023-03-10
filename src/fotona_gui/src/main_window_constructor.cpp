#include <exception>
#include "main_window.hpp"
#include "rviz/view_manager.h"
#include "rviz/tool_manager.h"
#include <qstringlist.h>

namespace RvizDisplayType {
	auto const Grid   = "rviz/Grid";
	auto const Marker = "rviz/Marker";
	auto const MarkerType = "visualization_msgs/Marker";
}

auto const rviz_fixed_frame            = "pico_flexx_optical_frame";
// rviz/FocusCamera rviz/Interact rviz/Measure rviz/MoveCamera rviz/PublishPoint rviz/Select rviz/SetGoal rviz/SetInitialPose rviz_plugin_tutorials/PlantFlag
auto const pointcloud_select_tool_name = "rviz/PublishPoint";

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	// init
	// new throws if it fails to allocate, which is (or at least should be) handled by the caller.
	this->render_panel   = new rviz::RenderPanel(parent);
	this->manager        = std::unique_ptr<rviz::VisualizationManager>(new rviz::VisualizationManager(this->render_panel));
	this->clear_button   = new QPushButton();
	this->scan_button    = new QPushButton();
	this->start_button   = new QPushButton();
	this->stop_button    = new QPushButton();
	this->main_layout    = new QGridLayout();
	this->central_widget = new QWidget();

	// set button text
	this->clear_button->setText("clear");
	this->scan_button->setText("scan");
	this->start_button->setText("start");
	this->stop_button->setText("stop it, get some help");

	// callbacks are connected to buttons like so:
	// this->connect(this->clear_button, &QPushButton::clicked, this,
	// 	[this](){ puts("button has been clicked!"); });

	// layout setup
	this->main_layout->addWidget(this->clear_button, 0, 0, 1, 1);
	this->main_layout->addWidget(this->scan_button,  1, 0, 1, 1);
	this->main_layout->addWidget(this->start_button, 2, 0, 1, 1);
	this->main_layout->addWidget(this->stop_button,  3, 0, 1, 1);
	this->main_layout->addWidget(this->render_panel, 0, 1, 4, 1);

	this->central_widget->setLayout(main_layout);
	this->setCentralWidget(central_widget);

	// fix up default layout
#define FIX(WIDGET) this->WIDGET->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding)
	FIX(clear_button);
	FIX(scan_button);
	FIX(start_button);
	FIX(stop_button);
	FIX(render_panel);
	FIX(central_widget);
#undef FIX

	for(int i=0; i<this->main_layout->rowCount(); i++)
		this->main_layout->setRowStretch(i, 1);
	this->main_layout->setColumnStretch(0, 1);
	this->main_layout->setColumnStretch(1, 4);

	// rviz setup
	this->manager->initialize();
	this->grid = this->manager->createDisplay(RvizDisplayType::Grid, "top-down orthogonal", true);
	if(grid == nullptr)
		throw std::runtime_error("Failed to create grid, something went terribly wrong");


	Ogre::Vector3 const origin(0, 0, 0);
	auto& view_manager = *(this->manager->getViewManager()->getCurrent());
	this->camera       = view_manager.getCamera();
	camera->setPosition(camera_position);
	view_manager.lookAt(origin);
	// setting up a valid view transform such that the camera won't complain
	auto view_matrix = calculate_projection_matrix(-1, 1, -1, 1, 0.001, 1000);
	camera->setCustomViewMatrix(true, view_matrix);  // this will be updated in `set_view_matrix`

	// create a marker to show;
	this->pointcloud = this->manager->createDisplay("rviz/PointCloud2", "pico flexx pointcloud", true);
	// this->pointcloud->subProp("Channel")->setValue("Intensity");
	this->pointcloud->subProp("Alpha")->setValue(0.5f);
	this->pointcloud->subProp("Size (m)")->setValue(0.003f);
	this->pointcloud->subProp("Color Transformer")->setValue("AxisColor");
	this->pointcloud->subProp("Axis")->setValue("Z");
	// Position Transformer
	// Color Transformer
	pointcloud->setTopic("/pico_flexx/points", "sensor_msgs/PointCloud2");
	this->connect(this->start_button, &QPushButton::clicked, this,
			[](){ puts("start button has been clicked!"); });

	this->manager->setFixedFrame(rviz_fixed_frame);

	this->manager->startUpdate();  // begin asynchronous update of the vizualization

	auto tool_manager = this->manager->getToolManager();
	auto tool_classes  = tool_manager->getToolClasses();
	for (int i=0; i<tool_classes.length(); i++)
		printf("tool class: %s\n", tool_classes[i].toLocal8Bit().data());

	auto pointcloud_select_tool = tool_manager->addTool(pointcloud_select_tool_name);
	// auto& pointcloud_select_tool = *tool_manager.addTool("rviz/MoveCamera");

	this->setEnabled(true);
	this->setVisible(true);
	this->setUpdatesEnabled(true);
	this->setMouseTracking(true);
	this->central_widget->setMouseTracking(true);
	this->central_widget->setEnabled(true);
	this->central_widget->setVisible(true);
	this->central_widget->setUpdatesEnabled(true);
	this->central_widget->setMouseTracking(true);

	tool_manager->setCurrentTool(pointcloud_select_tool);
	tool_manager->setDefaultTool(pointcloud_select_tool);
	tool_manager->setCurrentTool(pointcloud_select_tool);
	pointcloud_select_tool->activate();

	printf("ClassId:     %s\n", pointcloud_select_tool->getClassId().toLocal8Bit().data()    );
	printf("Name:        %s\n", pointcloud_select_tool->getName().toLocal8Bit().data()       );
	printf("Description: %s\n", pointcloud_select_tool->getDescription().toLocal8Bit().data());
	printf("ShortcutKey: %d\n", pointcloud_select_tool->getShortcutKey());

#define CHECK_TRACKING(NAME, QOBJECT) printf("%s mouse tracking is %s\n", NAME, QOBJECT->hasMouseTracking() ? "enabled" : "DISABLED")
	CHECK_TRACKING("main window", this);
	CHECK_TRACKING("central widget", central_widget);
	CHECK_TRACKING("rviz display", render_panel);
#undef CHECK_TRACKING
}