#include "main_window.hpp"

#include <cstdio>
#include <exception>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>
#include <QStackedLayout>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include <rviz/visualization_manager.h>
#include <rviz/visualization_frame.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/display.h>
#include "geometry_msgs/PointStamped.h"
#pragma GCC diagnostic pop

/*
    - never make the user wait
    - never make the user guess
    - never make the user fix it
*/

MainWindow::MainWindow() : QMainWindow(nullptr) {
    auto const central_widget = new QWidget();
    auto const layout         = new QGridLayout();
    auto const button_left    = new QPushButton();
    auto const button_right   = new QPushButton();

    auto const render_panel = new rviz::RenderPanel();
    auto const visualization_manager = new rviz::VisualizationManager(render_panel);
    visualization_manager->initialize();
    render_panel->initialize(visualization_manager->getSceneManager(), visualization_manager);

    this->setCentralWidget(central_widget);
    central_widget->setLayout(layout);
    layout->addWidget(render_panel, 0, 0, 1, 3);
    layout->addWidget(button_left,  1, 0, 1, 1);
    layout->addWidget(button_right, 1, 2, 1, 1);

    this->show();

    auto const grid_display       = visualization_manager->createDisplay("rviz/Grid",        "grid",   true);
    auto const pointcloud_display = visualization_manager->createDisplay("rviz/PointCloud2", "points", true);

    auto view_manager = visualization_manager->getViewManager();
    if (view_manager == nullptr)
        throw std::runtime_error("failed to get reference to view manager");
	view_manager->setCurrentViewControllerType("rviz/TopDownOrtho");
	auto view_controller = view_manager->getCurrent();
    if (view_manager == nullptr)
        throw std::runtime_error("failed to get reference to view controller");
	view_controller->subProp("X")->setValue(0.f);                // center the view
	view_controller->subProp("Y")->setValue(0.f);

    if (grid_display == nullptr)
        puts("failed to create grid display");
    if (pointcloud_display == nullptr)
		throw std::runtime_error("Failed to create point cloud, something went terribly wrong");
    pointcloud_display->initialize(visualization_manager);
    pointcloud_display->subProp("Alpha")->setValue(0.f);
    pointcloud_display->subProp("Size (m)")->setValue(0.003f);
    pointcloud_display->subProp("Color Transformer")->setValue("Intensity");
    pointcloud_display->subProp("Channel Name")->setValue("intensity");
    pointcloud_display->setTopic("/pico_flexx/points", "sensor_msgs/PointCloud2");
    visualization_manager->startUpdate();
    puts("done initing");
}

MainWindow::~MainWindow() { }