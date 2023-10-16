#pragma once

#include <memory>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include <rviz/visualization_manager.h>
#include <rviz/visualization_frame.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/display.h>
#pragma GCC diagnostic pop

namespace RvizDisplayType {
	auto const Grid   = "rviz/Grid";
	auto const Marker = "rviz/Marker";
	auto const Polygon = "rviz/Polygon";
	auto const MarkerType = "visualization_msgs/Marker";
}

class RvizBundle {
public:
	// std::unique_ptr<rviz::VisualizationFrame> visualization_frame;
 	rviz::RenderPanel* render_panel;
 	rviz::VisualizationManager* visualization_manager; // owned by render panel, I think
	rviz::Display* boundary;

	RvizBundle() {
		auto const rviz_fixed_frame            = "pico_flexx_optical_frame";
		// this->visualization_frame   = std::unique_ptr<rviz::VisualizationFrame>(new rviz::VisualizationFrame());
		this->render_panel          = new rviz::RenderPanel();
		this->visualization_manager = new rviz::VisualizationManager(this->render_panel);
		this->visualization_manager->initialize();
		this->render_panel->initialize(this->visualization_manager->getSceneManager(), this->visualization_manager);

		auto view_manager = this->visualization_manager->getViewManager();
		view_manager->setCurrentViewControllerType("rviz/TopDownOrtho");
		visualization_manager->setFixedFrame(rviz_fixed_frame);

		this->boundary = this->visualization_manager->createDisplay(RvizDisplayType::Polygon, "boundary", false);
		this->visualization_manager->addDisplay(this->boundary, false);

		auto grid = this->visualization_manager->createDisplay(RvizDisplayType::Grid, "grid", true);
		this->visualization_manager->addDisplay(grid, true);

		auto pointcloud = this->visualization_manager->createDisplay("rviz/PointCloud2", "pico flexx pointcloud", true);
		if(pointcloud == nullptr)
			throw std::runtime_error("Failed to create point cloud, something went terribly wrong");
		pointcloud->subProp("Alpha")->setValue(0.5f);
		pointcloud->subProp("Size (m)")->setValue(0.003f);
		//pointcloud->subProp("Color Transformer")->setValue("AxisColor");
		pointcloud->subProp("Axis")->setValue("Z");
		pointcloud->setTopic("/pico_flexx/points", "sensor_msgs/PointCloud2");


		visualization_manager->startUpdate();
	}
	~RvizBundle() noexcept {}
};