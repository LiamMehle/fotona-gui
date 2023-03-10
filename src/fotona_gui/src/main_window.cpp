#include "main_window.hpp"
// #include "rviz/view_controller.h"
#include <ctgmath>
#include <exception>
#include "rviz/view_manager.h"
#include <QList>
#include <numeric>
#include <algorithm>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>



namespace RvizDisplayType {
	auto const Grid   = "rviz/Grid";
	auto const Marker = "rviz/Marker";
	auto const MarkerType = "visualization_msgs/Marker";
}

auto const rviz_fixed_frame = "pico_flexx_optical_frame";

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
	auto& camera       = *view_manager.getCamera();
	camera.setPosition(camera_position);
	view_manager.lookAt(origin);

	// create a marker to show;
	this->pointcloud = this->manager->createDisplay("rviz/PointCloud2", "pico flexx pointcloud", true);
	this->pointcloud->subProp("Alpha")->setValue(0.5f);
	this->pointcloud->subProp("Channel")->setValue("Intensity");
	this->pointcloud->subProp("Size (m)")->setValue(0.003);
	this->pointcloud->subProp("Color Transformer")->setValue("AxisColor");
	this->pointcloud->subProp("Axis")->setValue("Z");
	// Position Transformer
	// Color Transformer
	pointcloud->setTopic("/pico_flexx/points", "sensor_msgs/PointCloud2");
	this->connect(this->start_button, &QPushButton::clicked, this,
			[](){ puts("start button has been clicked!"); });

	this->manager->setFixedFrame(rviz_fixed_frame);

	this->manager->startUpdate();  // begin asynchronous update of the vizualization
}

// cleanup handled by RAII and Qt
MainWindow::~MainWindow() noexcept {}

void MainWindow::set_view_matrix(Ogre::Matrix4 const m) {
	this->view_matrix = m;
	auto& view_manager = *(this->manager->getViewManager()->getCurrent());
	auto& camera       = *view_manager.getCamera();
	camera.setCustomViewMatrix(true, this->view_matrix);
	this->manager->notifyConfigChanged();
}

void MainWindow::set_pointcloud_alpha(float alpha) {
	this->pointcloud->subProp("Alpha")->setValue(alpha);
}
void MainWindow::set_pointcloud_size(float size) {
	this->pointcloud->subProp("Size (m)")->setValue(size);
}
void MainWindow::set_pointcloud_color_transformer(char const* const transformer) {
	this->pointcloud->subProp("Color Transformer")->setValue(transformer);
}