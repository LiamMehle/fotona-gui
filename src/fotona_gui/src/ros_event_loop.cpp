#include "ros_event_loop.hpp"

void ros_event_loop(ros::NodeHandle& n, MainWindow& w) {
	setup_mesh_generator(n, "/pico_flexx/mesh", "/pico_flexx/points");
	auto const subscriber = n.subscribe("/pico_flexx/points", 128, update_view_matrix);
	auto rate = ros::Rate(1);
	auto alpha_parameter_key      = std::string{"Alpha"};
	auto size_parameter_key       = std::string{"Size"};
	auto view_scale_parameter_key = std::string{"ViewScale"};
	n.param(alpha_parameter_key,   0.5f);
	n.param(size_parameter_key,    0.003f);
	while (ros::ok()) {
		ros::spinOnce();
		auto const alpha      = get_parameter<float>(n, alpha_parameter_key);
		auto const size       = get_parameter<float>(n, size_parameter_key);
		auto const view_scale = get_parameter<float>(n, view_scale_parameter_key);

		map(alpha,      [&](auto const alpha     ){ w.set_pointcloud_alpha(alpha); });
		map(size,       [&](auto const size      ){ w.set_pointcloud_size(size);   });
		map(view_scale, [&](auto const view_scale){ w.set_view_scale(view_scale);  });

		rate.sleep();
	}
}