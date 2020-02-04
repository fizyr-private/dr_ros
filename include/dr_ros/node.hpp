#pragma once
#include "dispatch.hpp"

#include <ros/ros.h>
#include <dr_log/dr_log.hpp>
#include <dr_param/param.hpp>

#include <vector>
#include <algorithm>
#include <utility>

namespace dr {

/// Shorthand to get a ServiceEvent type for a service.
template<typename Service>
using ServiceEvent = ros::ServiceEvent<typename Service::Request, typename Service::Response>;

/// A ROS node wrapper with some utility functions.
class Node : public ros::NodeHandle {
private:
	/// Run prefix for saving log files and logged data.
	std::string run_prefix_;

	/// Node prefix for saving log files and logged data.
	std::string node_prefix_;

public:
	/// Construct a node.
	Node();

	/// Get the raw node handle.
	ros::NodeHandle       & nodeHandle()       { return *this; }
	ros::NodeHandle const & nodeHandle() const { return *this; }

	/// Get the run prefix for saving log files and logged data.
	std::string runPrefix();

	/// Get the node prefix for saving log files and logged data.
	std::string nodePrefix();

protected:
	/// Get a parameter from the ROS parameter server.
	/**
	 * If the parameter doesn't exist on the parameter server, an exception is thrown.
	 */
	template<typename T>
	[[deprecated("Using the ROS parameter server is a bad idea. Consider using something else.")]]
	T getParam(std::string const & name) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		return dr::getParam<T>(*this, name);
#pragma GCC diagnostic pop
	}

	/// Get a parameter from the ROS parameter server.
	/**
	 * If the parameter doesn't exist on the parameter server, the fallback is returned.
	 */
	template<typename T>
	[[deprecated("Using the ROS parameter server is a bad idea. Consider using something else.")]]
	T getParam(std::string const & name, T const & fallback, bool warn = true) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		return dr::getParam<T>(*this, name, fallback, warn);
#pragma GCC diagnostic pop
	}

	/// Get a list of parameters from the ROS parameter server.
	/**
	 * If the parameters do not exist on the parameter server, an exception is thrown.
	 */
	template<typename T>
	[[deprecated("Using the ROS parameter server is a bad idea. Consider using something else.")]]
	std::vector<T> getParamList(std::string const & name) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		return dr::getParamList<T>(*this, name);
#pragma GCC diagnostic pop
	}

	/// Get a list of parameters from the ROS parameter server.
	/**
	 * If the parameters do not exist on the parameter server, the fallback is returned.
	 */
	template<typename T>
	[[deprecated("Using the ROS parameter server is a bad idea. Consider using something else.")]]
	std::vector<T> getParamList(std::string const & name, T const & fallback, bool warn = true) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		return dr::getParamList<T>(*this, name, fallback, warn);
#pragma GCC diagnostic pop
	}

	/// Search for a parameter up the namespace hierarchy and return it's value.
	template<typename T>
	[[deprecated("Using the ROS parameter server is a bad idea. Consider using something else.")]]
	T searchParam(std::string const & name, T const & fallback, bool warn = true) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		std::string key;
		if (!ros::NodeHandle::searchParam(name, key)) return fallback;
		return getParam<T>(key, fallback, warn);
#pragma GCC diagnostic pop
	}

	/// Dispatch a callback for later invocation.
	template<typename F>
	void dispatch(F && f) {
		dr::dispatch(*this, std::forward<F>(f));
	}

public:
	/// Check if ROS thinks we should keep running.
	bool ok() {
		return ros::NodeHandle::ok();
	}
};

}
