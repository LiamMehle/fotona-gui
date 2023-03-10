#pragma once
#include <string>
#include "ros/ros.h"

template<typename T>
constexpr inline
auto parse(std::string const& string) noexcept -> T {
	std::istringstream string_stream(string);
	T parse;
	string_stream >> parse;
	return parse;
}

template<typename T>
constexpr inline
auto get_parameter(ros::NodeHandle const& n, char const* const parameter_name) noexcept -> T {
	std::string parameter_as_string;
	n.getParamCached(parameter_name, parameter_as_string);
	return parse<T>(parameter_as_string);
}
