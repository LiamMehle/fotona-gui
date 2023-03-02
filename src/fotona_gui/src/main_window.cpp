#include "main_window.h"
// #include "rviz/view_controller.h"
#include "rviz/view_manager.h"
#include <ctgmath>
#include <exception>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	// init
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
#undef FIX

	this->main_layout->setRowStretch(0, 1);
	this->main_layout->setRowStretch(1, 1);
	this->main_layout->setColumnStretch(0, 1);
	this->main_layout->setColumnStretch(1, 4);

	// rviz setup
	this->grid = this->manager->createDisplay("rviz/Grid", "top-down orthogonal", true);
	if(grid == nullptr)
		throw std::runtime_error("Failed to create grid, something went terribly wrong");
	auto const fixed_frame = "map";
	this->manager->setFixedFrame(fixed_frame);
	this->manager->initialize();
	this->manager->startUpdate();

	auto const camera_position = Ogre::Vector3(0, 0, 1);  // Ogre does not support constexpr
	Ogre::Vector3 const origin(0, 0, 0);
	auto view_manager = this->manager->getViewManager()->getCurrent();
	view_manager->getCamera()->setPosition(camera_position);
	view_manager->lookAt(origin);
}

MainWindow::~MainWindow() noexcept {
}
