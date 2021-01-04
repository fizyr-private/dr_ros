#pragma once
#include "dispatch.hpp"

#include <ros/ros.h>
#include <dr_log/dr_log.hpp>

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
