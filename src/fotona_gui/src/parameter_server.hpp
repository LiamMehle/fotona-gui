#pragma once
#include <string>
#include "ros/ros.h"
#include <optional>

template<typename T>
inline
auto parse(std::string const& string) noexcept -> T {
	std::istringstream string_stream(string);
	T parse;
	string_stream >> parse;
	return parse;
}

template<typename T>
constexpr inline
auto get_parameter(ros::NodeHandle const& n, std::string& parameter_name) noexcept -> std::optional<T> {
	T parameter;
	bool const was_set = n.getParam(parameter_name, parameter);
	return was_set ? std::optional{parameter} : std::nullopt;
}
