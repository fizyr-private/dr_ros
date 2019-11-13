// This repository
#include "resource.hpp"

// ROS
#include <ros/package.h>

// C++
#include <algorithm>
#include <stdexcept>
#include <iostream>

namespace dr {

std::string getPathX(const std::string& package_name) {

	std::string path = ros::package::command("find " + package_name);

	std::cout << "path:" << path << "\n";

	// scrape any newlines out of it
	for (
		size_t newline = path.find('\n');
		newline != std::string::npos;
		newline = path.find('\n')
	) {
		path.erase(newline, 1);
	}

	return path;
}

std::string resolveResourceUrl(std::string const & url) {
	std::string const scheme_seperator = "://";
	auto scheme_end = std::search(url.begin(), url.end(), scheme_seperator.begin(), scheme_seperator.end());

	// No scheme? Treat the input as a relative file path.
	if (scheme_end == url.end()) {
		return url;
	}

	std::string scheme(url.begin(), scheme_end);
	auto authority_start = scheme_end + 3;
	auto path_start      = std::find(authority_start, url.end(), '/');

	std::string authority(authority_start, path_start);
	std::string path(path_start, url.end());

	std::cout << "scheme:'" << scheme << "'\n";
	std::cout << "authority:'" << authority << "'\n";
	std::cout << "path:'" << path << "'\n";
	std::cout << "=======================\n\n";

	if (scheme == "package") {
		// std::string package_path = ros::package::getPath(authority);
		std::string package_path = getPathX(authority);
		if (package_path.empty()) {
			throw std::runtime_error("Failed to find package `" + authority + "' for URL: " + url);
		}
		return package_path + path;

	} else if (scheme == "file") {
		if (authority.size()) throw std::runtime_error("Tried to convert non-local file URL to file path: " + url);
		return path;
	}

	throw std::runtime_error("Unsupported URL scheme `" + scheme + "' in URL: " + url);
}

/// Convert a ROS package URL or a regular file URL to a file path.
/*
std::string resolveResourceUrl(std::string const & url) {
	std::string const scheme_seperator = "://";
	auto scheme_end = std::search(url.begin(), url.end(), scheme_seperator.begin(), scheme_seperator.end());

	// No scheme? Treat the input as a relative file path.
	if (scheme_end == url.end()) return url;

	std::string scheme(url.begin(), scheme_end);
	auto authority_start = scheme_end + 3;
	auto path_start      = std::find(authority_start, url.end(), '/');

	std::string authority(authority_start, path_start);
	std::string path(path_start, url.end());

	if (scheme == "package") {
		std::string package_path = ros::package::getPath(authority);
		if (package_path.empty()) throw std::runtime_error("Failed to find package `" + authority + "' for URL: " + url);
		return package_path + path;

	} else if (scheme == "file") {
		if (authority.size()) throw std::runtime_error("Tried to convert non-local file URL to file path: " + url);
		return path;
	}

	throw std::runtime_error("Unsupported URL scheme `" + scheme + "' in URL: " + url);
}
*/

}
