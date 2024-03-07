//
// File: pluginMain.cpp
//
// Author: Benjamin H. Singleton
//

#include "PointOnCurveConstraint.h"

#include <maya/MFnPlugin.h>


MStatus initializePlugin(MObject obj) 
{

	MStatus status;

	// Register nodes
	//
	MFnPlugin plugin(obj, "Ben Singleton", "2023", "Any");

	status = plugin.registerNode("pointOnCurveConstraint", PointOnCurveConstraint::id, PointOnCurveConstraint::creator, PointOnCurveConstraint::initialize, MPxNode::kConstraintNode, &PointOnCurveConstraint::classification);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	if (!status)
	{

		status.perror("registerNode");
		return status;

	}

	return status;

}

MStatus uninitializePlugin(MObject obj) 
{

	MStatus status;

	// Deregister nodes
	//
	MFnPlugin plugin(obj);

	status = plugin.deregisterNode(PointOnCurveConstraint::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	if (!status)
	{

		status.perror("deregisterNode");
		return status;

	}

	return status;

}
