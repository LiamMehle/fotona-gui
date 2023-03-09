#pragma once

#include <ctgmath>
#include <exception>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include "rviz/view_manager.h"
#pragma GCC diagnostic pop
#include <QList>
#include <numeric>
#include <algorithm>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include "rviz/default_plugin/point_cloud_transformers.h"

class CustomDepthMapTransformer : rviz::AxisColorPCTransformer {
	// sets up any properties when loaded
	void createProperties(rviz::Property* parent_property, uint32_t mask, QList<rviz::Property*>& out_props);
	// preference/priority. Should return 0 for flat color, but for custom embeded applications, it returns 255.
	uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud);
	uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud);

	// ignores `mask`, assume little-endian
	bool transform(sensor_msgs::PointCloud2ConstPtr const& cloud_pointer, uint32_t mask, const Ogre::Matrix4& transform, rviz::V_PointCloudPoint& out);
};