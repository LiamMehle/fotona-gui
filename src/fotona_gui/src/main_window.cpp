#include "main_window.hpp"

#include <exception>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>

// #include "rviz/view_manager.h"
// #include "transparent_button.hpp"
// #include <rviz/view_manager.h>
// #include <rviz/visualization_frame.h>
// #include <rviz/tool_manager.h>
// #include <qstringlist.h>

// #include <rviz/default_plugin/tools/selection_tool.h>
// #include <rviz/selection/selection_manager.h>
// #include <QGraphicsOpacityEffect>
// #include <QDockWidget>
// #include <QFrame>

QWidget* create_main_screen(MainWindow* main_window) {
	auto* const central_widget = new QWidget();

	auto* const torso_mode_button = new QPushButton("TORSO (chest/back)");
	auto* const limb_mode_button  = new QPushButton("LIMB (arm/leg)");
	QPushButton::connect(torso_mode_button, &QPushButton::clicked, [=] {
		main_window->transition_into(Mode::Torso, Screen::Align);
	});
	QPushButton::connect(limb_mode_button, &QPushButton::clicked, [=] {
		main_window->transition_into(Mode::Limb, Screen::Align);
	});

	auto* const layout = new QHBoxLayout();
	layout->addWidget(torso_mode_button);
	layout->addWidget(limb_mode_button);
	central_widget->setLayout(layout);

	central_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	torso_mode_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	limb_mode_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	return central_widget;
}

QWidget* create_rviz_screen(MainWindow* main_window) {
	auto* const central_widget = new QWidget();

	auto* const torso_mode_button = new QPushButton("TORSO (chest/back)");
	auto* const limb_mode_button  = new QPushButton("LIMB (arm/leg)");
	QPushButton::connect(torso_mode_button, &QPushButton::clicked, [=] {
		main_window->transition_into(Mode::Torso, Screen::Align);
	});
	QPushButton::connect(limb_mode_button, &QPushButton::clicked, [=] {
		main_window->transition_into(Mode::Limb, Screen::Align);
	});

	auto* const layout = new QHBoxLayout();
	layout->addWidget(torso_mode_button);
	layout->addWidget(limb_mode_button);
	central_widget->setLayout(layout);

	central_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	torso_mode_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	limb_mode_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	return central_widget;
}
QWidget* create_finish_screen(MainWindow* main_window) {
	auto const central_widget = new QPushButton();

	QPushButton::connect(central_widget, &QPushButton::clicked, [=] {
		main_window->transition_into(Mode::Undefined, Screen::ModeSelect);
	});
	auto const main_label     = new QLabel("FINISHED");
	auto const sub_label      = new QLabel("tap anywhere on the screen to continue");

	main_label->setAlignment(Qt::AlignHCenter);
	sub_label->setAlignment(Qt::AlignHCenter);

	auto const layout = new QVBoxLayout();
	layout->addWidget(main_label);
	layout->addWidget(sub_label);
	layout->setAlignment(Qt::AlignCenter);
	central_widget->setLayout(layout);

	return central_widget;
}

// should be the same as the order of widgets in stack
enum CentralWidgetStack {
	Main,
	Rviz,
	Finish
};

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	// should be the same as the order of widgets in enum
	this->central_widget = new QStackedWidget();
	this->central_widget->addWidget(create_main_screen(this));
	this->central_widget->addWidget(create_rviz_screen(this));
	this->central_widget->addWidget(create_finish_screen(this));
	
	this->setCentralWidget(this->central_widget);
	this->central_widget->setCurrentIndex(CentralWidgetStack::Finish);
	this->transition_into(Mode::Undefined, Screen::Finished);
	this->show();
}


void MainWindow::transition_into(Mode const mode, Screen const screen) {
	this->state.mode   = mode;
	this->state.screen = screen;
	this->update_state();
}

void MainWindow::update_state() {
	switch (this->state.screen) {
		case Screen::ModeSelect:
			this->central_widget->setCurrentIndex(CentralWidgetStack::Main);
			break;
		case Screen::Align:
			this->central_widget->setCurrentIndex(CentralWidgetStack::Rviz);
			break;
		case Screen::DrawBorder:
			this->central_widget->setCurrentIndex(CentralWidgetStack::Rviz);
			break;
		case Screen::ConfirmBorder:
			this->central_widget->setCurrentIndex(CentralWidgetStack::Rviz);
			break;
		case Screen::Execute:
			this->central_widget->setCurrentIndex(CentralWidgetStack::Rviz);
			break;
		case Screen::Finished:
			this->central_widget->setCurrentIndex(CentralWidgetStack::Finish);
			break;
		default:
			throw std::runtime_error("bad Screen state.");
	}
}
// cleanup handled by RAII and Qt
MainWindow::~MainWindow() noexcept {}

// namespace RvizDisplayType {
// 	auto const Grid   = "rviz/Grid";
// 	auto const Marker = "rviz/Marker";
// 	auto const MarkerType = "visualization_msgs/Marker";
// }

// auto const rviz_fixed_frame            = "pico_flexx_optical_frame";
// auto const pointcloud_select_tool_name = "rviz/PublishPoint";
// MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
// 	this->setAttribute(Qt::WA_PaintOnScreen, false);
// 	this->setAttribute(Qt::WA_PaintUnclipped, false);
// 	// init
// 	// new throws if it fails to allocate, which is (or at least should be) handled by the caller.
// 	auto clear_button          = new TransparentButton("clear", this);  // raw pointers instead of smart pointers because Qt will take ownership
// 	auto scan_button           = new QPushButton("scan");
// 	auto start_button          = new QPushButton("start");
// 	auto stop_button           = new QPushButton("stop");
// 	auto main_layout           = new QGridLayout();
// 	auto button_layout         = new QVBoxLayout();
// 	auto button_container      = new QWidget();
// 	auto button_dock           = new QDockWidget();
// 	auto central_widget        = new QWidget();
// 	auto visualization_frame   = new rviz::VisualizationFrame();
// 	auto render_panel          = new rviz::RenderPanel(visualization_frame);
// 	auto visualization_manager = new rviz::VisualizationManager(render_panel, visualization_frame);
// 	render_panel->initialize(visualization_manager->getSceneManager(), visualization_manager);
// 	// Pointers can be indexed into and restrict is an non-standard. A reference fixes it.

// 	// main_layout->addWidget(visualization_frame);
// 	// central_widget->setLayout(main_layout);
// 	this->setCentralWidget(visualization_frame);

// 	// button layout/setup
// 	button_layout->addWidget(clear_button);
// 	button_layout->addWidget(scan_button);
// 	button_layout->addWidget(start_button);
// 	button_layout->addWidget(stop_button);
// 	// button_container->setLayout(button_layout);
// 	// button_dock->setWidget(button_container);
// 	main_layout->addWidget(render_panel, 0, 0, 1, 2);
// 	main_layout->addLayout(button_layout, 0, 0, 1, 1);
// 	main_layout->setColumnStretch(0, 1);
// 	main_layout->setColumnStretch(1, 4);
// 	central_widget->setLayout(main_layout);
// 	visualization_frame->setCentralWidget(central_widget);

// 	// button_dock->setGraphicsEffect(opacity(0.2f));

// 	// fix up default layout
// #define FIX(WIDGET) WIDGET->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding)
// 	FIX(clear_button);
// 	FIX(scan_button);
// 	FIX(start_button);
// 	FIX(stop_button);
// 	FIX(central_widget);
// 	FIX(render_panel);
// #undef FIX

// 	// rviz setup
	
// 	visualization_manager->initialize();

// 	Ogre::Vector3 const origin(0, 0, 0);
// 	auto view_manager = visualization_manager->getViewManager();
// 	view_manager->setCurrentViewControllerType("rviz/TopDownOrtho");
// 	auto view_controller = view_manager->getCurrent();
// 	view_controller->subProp("X")->setValue(0.f);                // center the view
// 	view_controller->subProp("Y")->setValue(0.f);
// 	this->view_scale = view_controller->subProp("Scale");

// 	auto grid = visualization_manager->createDisplay(RvizDisplayType::Grid, "grid", true);
// 	if(grid == nullptr)
// 		throw std::runtime_error("Failed to create grid, something went terribly wrong");
// 	// grid->initialize(visualization_manager);

// 	this->pointcloud = visualization_manager->createDisplay("rviz/PointCloud2", "pico flexx pointcloud", true);
// 	if(pointcloud == nullptr)
// 		throw std::runtime_error("Failed to create point cloud, something went terribly wrong");
// 	// this->pointcloud->initialize(visualization_manager);
// 	this->pointcloud->subProp("Alpha")->setValue(0.f);
// 	this->pointcloud->subProp("Size (m)")->setValue(0.003f);
// 	this->pointcloud->subProp("Color Transformer")->setValue("Intensity");
// 	this->pointcloud->subProp("Channel Name")->setValue("intensity");
// 	this->pointcloud->setTopic("/pico_flexx/points", "sensor_msgs/PointCloud2");

// 	auto mesh = visualization_manager->createDisplay("rviz/Marker", "pico flexx reconstructed mesh", true);
// 	mesh->setTopic("/pico_flexx/mesh", "visualization_msgs/Marker");

// 	auto tool_manager = visualization_manager->getToolManager();
// 	auto pointcloud_select_tool = tool_manager->addTool(pointcloud_select_tool_name);
// 	if (pointcloud_select_tool == nullptr)
// 		throw std::runtime_error("pointcloud_select_tool is not of type rviz::SelectionTool");
// 	pointcloud_select_tool->initialize(visualization_manager);  // missing piece
// 	tool_manager->setDefaultTool(pointcloud_select_tool);
// 	tool_manager->setCurrentTool(pointcloud_select_tool);

// 	visualization_manager->setFixedFrame(rviz_fixed_frame);

// 	this->show();
// 	visualization_manager->startUpdate();  // begin asynchronous update of the vizualization
// }

// void MainWindow::set_view_matrix(Ogre::Matrix4 m) {
// 	// this->camera->setCustomViewMatrix(true, m);
// }

// void MainWindow::set_pointcloud_alpha(float alpha) {
// 	if (alpha == this->pointcloud_alpha)  // caching because updating is expensive
// 		return;
// 	this->pointcloud_alpha = alpha;
// 	this->pointcloud->subProp("Alpha")->setValue(alpha);
// }

// void MainWindow::set_pointcloud_size(float size) {
// 	if (size == this->pointcloud_size)  // caching because updating is expensive
// 		return;
// 	this->pointcloud_size = size;
// 	this->pointcloud->subProp("Size (m)")->setValue(size);
// }
// void MainWindow::set_pointcloud_color_transformer(char const* const transformer) {
// 	throw std::runtime_error("unimplemented");
// 	// this->pointcloud->subProp("Color Transformer")->setValue(transformer);
// }

// void MainWindow::set_view_scale(float const scale) {
// 	this->view_scale->setValue(scale);
// }