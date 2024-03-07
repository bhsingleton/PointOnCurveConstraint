cmake_minimum_required(VERSION 3.21)
project(PointOnCurveConstraint)
include($ENV{DEVKIT_LOCATION}/cmake/pluginEntry.cmake)

set(
	SOURCE_FILES
	"pluginMain.cpp"
	"PointOnCurveConstraint.h"
	"PointOnCurveConstraint.cpp"
)

set(
	LIBRARIES
	OpenMaya
	OpenMayaAnim
	Foundation
)

build_plugin()