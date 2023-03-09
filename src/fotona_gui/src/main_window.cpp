#include "main_window.h"
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

#include "rviz/default_plugin/point_cloud_transformers.h"

// class DepthMapTransformer : rviz::AxisColorPCTransformer {
// 	// sets up any properties when loaded
// 	void createProperties(rviz::Property* parent_property, uint32_t mask, QList<rviz::Property*>& out_props) {}
// 	// preference/priority. Should return 0 for flat color, but for custom embeded applications, it returns 255.
// 	uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud)    { return ~0; }
// 	uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) { return ~0;}

// 	// ignores `mask`, assume little-endian
// 	bool transform(sensor_msgs::PointCloud2ConstPtr const& cloud_pointer, uint32_t mask, const Ogre::Matrix4& transform, rviz::V_PointCloudPoint& out) {
// 		auto const& cloud  = *(cloud_pointer.get());
// 		// todo: check that the layout is:
// 		// all float32: x, y, z, noise, intensity, gray
// 		struct point {
// 			float x, y, z, noise, intensity, gray;
// 		};
// 		// hacky stuff for convenience: telling the compiler that this is actually a bunch of structured sigle-precision floats
// 		auto const  input_points = reinterpret_cast<point const* const __restrict__>(cloud.data.data());
// 		auto       output_points = reinterpret_cast<point      * const __restrict__>(out.data());
// 		auto highest_z = input_points[0].z;
// 		auto  lowest_z = input_points[0].z;
// 		for (int i=1; i<cloud.data.size(); i++) {
// 			highest_z = std::max(highest_z, input_points[i].z);
// 			 lowest_z = std::min( lowest_z, input_points[i].z);
// 		}
// 		for (int i=0; i<cloud.data.size(); i++) {
// 			auto point = input_points[i];
// 			point.z -= lowest_z;
// 			point.gray = point.z / (highest_z - lowest_z);
// 			output_points[i] = point;
// 		}
// 		// auto const highest_z = std::reduce(points[0], points[cloud.data.size()], points[0].z, [](point const& a, point const& b) {
// 		// 	return std::max(a.z, b.z);
// 		// });
// 		// auto const lowest_z = std::reduce(points[0], points[cloud.data.size()], points[0].z, [](point const& a, point const& b) {
// 		// 	return std::min(a.z, b.z);
// 		// });
// 		// for (int index=0; index<cloud.width*cloud.height; index++) { // structure is ignored
// 		// 	depth
// 		// }
// 	}
// };

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
	rviz::Display& pointcloud = *this->manager->createDisplay("rviz/PointCloud2", "pico flexx pointcloud", true);
	pointcloud.setTopic("/pico_flexx/points", "sensor_msgs/PointCloud2");

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
