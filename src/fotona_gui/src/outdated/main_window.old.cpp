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

// #include "transparent_button.hpp"
// #include <rviz/tool_manager.h>
// #include <qstringlist.h>

// #include <rviz/default_plugin/tools/selection_tool.h>
// #include <rviz/selection/selection_manager.h>
// #include <QGraphicsOpacityEffect>
// #include <QDockWidget>
// #include <QFrame>

	/*
	root < can't be assigned a layout >
	|-widget
		|-grid layout
			|-render panel < can't be assinged a layout >
			|- widget (because layouts can't be removed from layouts)
				|-screen (grid) layout < this one is replaced depending on the screen >
					|-button[s] / widgets
	*/

template<typename T>
static
T* apply_attributes(T* const widget) {
	widget->setAttribute(Qt::WA_AcceptDrops, false);
	widget->setAttribute(Qt::WA_AlwaysShowToolTips, false);
	widget->setAttribute(Qt::WA_Hover, false);
	widget->setAttribute(Qt::WA_InputMethodEnabled, false);
	widget->setAttribute(Qt::WA_KeyboardFocusChange, false);
	widget->setAttribute(Qt::WA_LayoutOnEntireRect, false);  // experiment
	widget->setAttribute(Qt::WA_LayoutUsesWidgetRect, false);  // experiment
	widget->setAttribute(Qt::WA_MouseTracking, false);
	widget->setAttribute(Qt::WA_NoChildEventsForParent, true);
	widget->setAttribute(Qt::WA_NoChildEventsFromChildren, true);
	widget->setAttribute(Qt::WA_NoMousePropagation, true);
	widget->setAttribute(Qt::WA_NoSystemBackground, true);
	// widget->setAttribute(Qt::WA_OpaquePaintEvent, true);
	// widget->setAttribute(Qt::WA_PaintOnScreen, true);
	// widget->setAttribute(Qt::WA_PaintUnclipped, true);
	return widget;
}

static
QWidget* init_main_menu(MainWindow* self) {
	auto* const torso_mode_button = new QPushButton("TORSO (chest/back)");
	auto* const limb_mode_button  = new QPushButton("LIMB (arm/leg)");
	QPushButton::connect(torso_mode_button, &QPushButton::clicked, [=] {
		self->transition_into(Mode::Torso, Screen::Align);
	});
	QPushButton::connect(limb_mode_button, &QPushButton::clicked, [=] {
		self->transition_into(Mode::Limb, Screen::Align);
	});

	// layout screen
	auto* const layout = new QHBoxLayout();
	layout->addWidget(torso_mode_button);
	layout->addWidget(limb_mode_button);
	torso_mode_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	limb_mode_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

	// attach
	auto const central_widget = new QWidget();
	central_widget->setLayout(layout);
	return central_widget;
}

void MainWindow::transition_into(Screen const screen) {
	this->state.screen = screen;
	this->update_state();
}
void MainWindow::transition_into(Mode const mode, Screen const screen) {
	this->state.mode   = mode;
	this->state.screen = screen;
	this->update_state();
}

// template<typename T0, typename T1>
static
std::tuple<QWidget*, QPushButton*, QPushButton*, RvizBundle> create_generic_2button_screen(
									// 	QString button_text_left,
                                    //    T0 action_left,
                                    //    QString button_text_right,
                                    //    T1 action_right
									   ) {
	// define buttons
	RvizBundle rviz_bundle;
	auto const left_button  = new QPushButton(); // new QPushButton(button_text_left);
	auto const right_button = new QPushButton(); // new QPushButton(button_text_right);

	// QPushButton::connect(button_left, &QPushButton::clicked, action_left);
	// QPushButton::connect(button_right, &QPushButton::clicked, action_right);

	// layout buttons
	auto const layout = new QGridLayout();
	layout->addWidget(rviz_bundle.render_panel, 0, 0, 1, 3);
	layout->addWidget(left_button, 1, 0);
	layout->addWidget(right_button, 1, 2);

	// attach to widget
	auto const widget = new QWidget();
	widget->setLayout(layout);

	return {widget, left_button, right_button, rviz_bundle};
}

MainWindow::MainWindow(ros::NodeHandle& n, QWidget *parent) : 
	QMainWindow(parent),
	button_context(std::make_unique<QPushButton>()), 
	n(n),
	root_layout(new QStackedLayout()) {
	// so far, we have
	/*
	root
	|-widget
		|-grid layout
			|-render panel < can't be assinged a layout >
	*/
	this->setAttribute(Qt::WA_QuitOnClose, true);
	this->setAttribute(Qt::WA_NativeWindow, true);
	auto central_widget = new QWidget();
	apply_attributes(central_widget);
	central_widget->setLayout(root_layout);  // mandatory, else another window shows up on call to this->show()
	this->setCentralWidget(central_widget);

	auto main_menu = init_main_menu(this);
	apply_attributes(main_menu);
	auto [common_layout_, left_button_, right_button_, rviz_bundle_] = create_generic_2button_screen();
	this->left_button = left_button_;
	this->right_button = right_button_;
	this->rviz_bundle = rviz_bundle_;
	// define buttons
	apply_attributes(main_menu);
	apply_attributes(common_layout_);
	apply_attributes(left_button);
	apply_attributes(right_button);
	this->root_layout->insertWidget(0, main_menu);
	this->root_layout->insertWidget(1, common_layout_);

	this->transition_into(Mode::Undefined, Screen::ModeSelect);
	this->show();
}

template<typename T>
static void configure_button(QPushButton* const button, QObject* const context, QString const text, T on_click) {
	button->setText(text);
	QPushButton::connect(button, &QPushButton::clicked, context, on_click);
}

void MainWindow::update_state() {
	// default settings overriden based on the current state
	this->button_context = std::make_unique<QPushButton>();
	this->right_button->show();
	this->root_layout->setCurrentIndex(1);

	switch (this->state.screen) {
		case Screen::ModeSelect:
			this->root_layout->setCurrentIndex(0);
			break;
		case Screen::Align:
			configure_button(this->left_button, button_context.get(), "abort", [this] {
				this->transition_into(Screen::ModeSelect);
			});
			configure_button(this->right_button, button_context.get(), "set border", [this] {
				this->transition_into(Screen::DrawBorder);
			});
			break;
		case Screen::DrawBorder:
			// set up buttons
			configure_button(this->left_button, button_context.get(), "abort", [this] {
				this->transition_into(Screen::ModeSelect);
			});
			configure_button(this->right_button, button_context.get(), "done drawing border", [this] {
				this->transition_into(Screen::ConfirmBorder);
			});

			// set up tool
			{
				auto tool_manager = this->rviz_bundle.visualization_manager->getToolManager();
				auto tool = tool_manager->addTool("rviz/PublishPoint");
				tool_manager->setCurrentTool(tool);
			}
			// this->n.subscribe(std::string("/clicked_point"), 4, &MainWindow::add_point, nullptr);
			// this->rviz_bundle.boundary->show();
			// this->rviz_bundle.visualization_manager->addDisplay(this->rviz_bundle.boundary, true);
			break;
		case Screen::ConfirmBorder:
			configure_button(this->left_button, button_context.get(), "redraw border", [this] {
				this->transition_into(Screen::DrawBorder);
			});
			configure_button(this->right_button, button_context.get(), "run", [this] {
				this->transition_into(Screen::Execute);
			});
			break;
		case Screen::Execute:
			configure_button(this->left_button, button_context.get(), "abort", [this] {
				this->transition_into(Screen::ModeSelect);
			});
			this->right_button->hide();
			break;
		default:
			break;
			// throw std::runtime_error("bad Screen state.");
	}
	// this->rviz_bundle.render_panel->raise();
	// this->rviz_bundle->render_panel->lower();
	// this->rviz_bundle.visualization_manager->removeAllDisplays();
	// auto const grid = this->rviz_bundle.visualization_manager->createDisplay(RvizDisplayType::Grid, "grid", true);
	// this->rviz_bundle.visualization_manager->addDisplay(grid, true);
}

void MainWindow::add_point() noexcept {}

// cleanup handled by RAII and Qt
MainWindow::~MainWindow() noexcept {}

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
// 	main_layout->addWidget(render_panel, 0, 0, 0, 2);
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