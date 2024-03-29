#include "main_window.hpp"
#include "rviz/view_manager.h"

// cleanup handled by RAII and Qt
MainWindow::~MainWindow() noexcept {}

void MainWindow::set_view_matrix(Ogre::Matrix4 m) {
	// this->camera->setCustomViewMatrix(true, m);
}

void MainWindow::set_pointcloud_alpha(float alpha) {
	if (alpha == this->pointcloud_alpha)  // caching because updating is expensive
		return;
	this->pointcloud_alpha = alpha;
	this->pointcloud->subProp("Alpha")->setValue(alpha);
}

void MainWindow::set_pointcloud_size(float size) {
	if (size == this->pointcloud_size)  // caching because updating is expensive
		return;
	this->pointcloud_size = size;
	this->pointcloud->subProp("Size (m)")->setValue(size);
}
void MainWindow::set_pointcloud_color_transformer(char const* const transformer) {
	throw std::runtime_error("unimplemented");
	// this->pointcloud->subProp("Color Transformer")->setValue(transformer);
}

void MainWindow::set_view_scale(float const scale) {
	this->view_scale->setValue(scale);
}