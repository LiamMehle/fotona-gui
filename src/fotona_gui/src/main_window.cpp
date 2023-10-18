#include "main_window.hpp"

#include <cstdio>
#include <utility>
#include <exception>
#include <QPushButton>
#include <QHBoxLayout>
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


// unused
// class to abuse event filtering to run custom code upon click
class RenderPanelEventFilter : public QObject {
private:
	std::function<void()> callable;
public:
	RenderPanelEventFilter(std::function<void()> callable) {
		this->callable = callable;
	}
protected:
    bool eventFilter(QObject *obj, QEvent *event) override {
		if (event->type() == QEvent::MouseButtonPress)
			this->callable();
		return false;
	};
};


MainWindow::MainWindow() : 
	QMainWindow(nullptr),
	button_left(new QPushButton()),
	button_right(new QPushButton()),
	status_text(new QLabel()) {
	auto const central_widget = new QWidget();
	auto const layout         = new QGridLayout();

	auto const render_panel = new rviz::RenderPanel();
	this->visualization_manager = new rviz::VisualizationManager(render_panel);
	visualization_manager->initialize();
	render_panel->initialize(visualization_manager->getSceneManager(), visualization_manager);

	this->setCentralWidget(central_widget);
	central_widget->setLayout(layout);
	layout->addWidget(render_panel, 0, 0, 1, 3);
	layout->addWidget(this->button_left,  1, 0, 1, 1);
	layout->addWidget(this->status_text,  1, 1, 1, 1);
	layout->addWidget(this->button_right, 1, 2, 1, 1);

	this->transition(Stage::Position);

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

void MainWindow::transition(Stage const stage) {
	switch (stage) {
		case Stage::Position:
			this->status_text->setText("Positioning");
			this->button_left->hide();
			this->button_right->show();
			this->button_right->setText("next: configure perimeter");
			QPushButton::connect(this->button_right, &QPushButton::clicked, [=]{
				this->transition(Stage::ConfigurePerimiter);
			});
			break;

		case Stage::ConfigurePerimiter:
			this->status_text->setText("configure perimeter");
			this->button_left->show();
			this->button_left->setText("back: position");
			QPushButton::connect(this->button_left, &QPushButton::clicked, [=]{
        		this->transition(Stage::Position);
    		});
			this->button_right->show();
			this->button_right->setText("next: run");
			QPushButton::connect(this->button_right, &QPushButton::clicked, [=]{
				this->transition(Stage::Run);
			});

			{
				// set up mouse tool for picking point in cloud
				auto tool_manager = this->visualization_manager->getToolManager();
				auto tool = tool_manager->addTool("rviz/PublishPoint");
				tool_manager->setCurrentTool(tool);
				tool_manager->setDefaultTool(tool);
			}
			break;


		case Stage::Run:
			this->status_text->setText("running");
			this->button_left->show();
			this->button_left->setText("CANCEL");
			QPushButton::connect(this->button_left, &QPushButton::clicked, [=]{
        		this->transition(Stage::ConfigurePerimiter);
    		});
			this->button_right->hide();
			// todo implement
			break;

		case Stage::Done:
			this->status_text->setText("DONE");
			this->button_left->show();
			this->button_left->setText("back: positioning");
			QPushButton::connect(this->button_left, &QPushButton::clicked, [=]{
        		this->transition(Stage::Position);
    		});
			this->button_right->hide();
			break;
	
		default:
			break;
	}
}