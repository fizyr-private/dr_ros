// This repository
#include "resource.hpp"

// ROS
#include <ros/package.h>

// Catch2
#include <catch2/catch.hpp>

namespace dr {

TEST_CASE("ResourceTest -- packageUrl", "packageUrl") {
	/**
	 * TODO: This test fails in BuildBot because it can not resolve the location
	 *       of the "package://some_package". At the end of a chain of calls,
	 *       'ROSPack::run(..)' excecutes "rospack find some_package".
	 * 
	 *       More about this method.
	 *       http://docs.ros.org/indigo/api/rospack/html/rospack__backcompat_8cpp_source.html		 * 
	 * 
	 *       This method succeeds if rospack is installed and located in the PATH.
	 *       In BuildBot test-container rospack is installed, but its location is not in the PATH.
	 **/
	REQUIRE(ros::package::getPath("dr_ros") + "/test.file" == resolveResourceUrl("package://dr_ros/test.file"));
}

TEST_CASE("ResourceTest -- localFileUrl", "localFileUrl") {
	REQUIRE("/test.file" == resolveResourceUrl("file:///test.file"));
}

TEST_CASE("ResourceTest -- relativeFileUrl", "relativeFileUrl") {
	REQUIRE("foo/test.file" == resolveResourceUrl("foo/test.file"));
}

TEST_CASE("ResourceTest -- remoteFileUrl", "remoteFileUrl") {
	REQUIRE_THROWS(resolveResourceUrl("file://host/test.file"));
}

TEST_CASE("ResourceTest -- unsupportedScheme", "unsupportedScheme") {
	REQUIRE_THROWS(resolveResourceUrl("http://example.com/test.file"));
}

}
