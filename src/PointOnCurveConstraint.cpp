//
// File: PointOnCurveConstraint.cpp
//
// Dependency Graph Node: pointOnCurveConstraint
//
// Author: Benjamin H. Singleton
//

#include "PointOnCurveConstraint.h"


MObject PointOnCurveConstraint::forwardVector;
MObject PointOnCurveConstraint::forwardVectorX;
MObject PointOnCurveConstraint::forwardVectorY;
MObject PointOnCurveConstraint::forwardVectorZ;
MObject PointOnCurveConstraint::upVector;
MObject PointOnCurveConstraint::upVectorX;
MObject PointOnCurveConstraint::upVectorY;
MObject PointOnCurveConstraint::upVectorZ;
MObject PointOnCurveConstraint::worldUpType;
MObject PointOnCurveConstraint::worldUpVector;
MObject PointOnCurveConstraint::worldUpVectorX;
MObject PointOnCurveConstraint::worldUpVectorY;
MObject PointOnCurveConstraint::worldUpVectorZ;
MObject PointOnCurveConstraint::worldUpMatrix;
MObject PointOnCurveConstraint::twist;
MObject PointOnCurveConstraint::offsetTranslate;
MObject PointOnCurveConstraint::offsetTranslateX;
MObject PointOnCurveConstraint::offsetTranslateY;
MObject PointOnCurveConstraint::offsetTranslateZ;
MObject PointOnCurveConstraint::offsetRotate;
MObject PointOnCurveConstraint::offsetRotateX;
MObject PointOnCurveConstraint::offsetRotateY;
MObject PointOnCurveConstraint::offsetRotateZ;
MObject PointOnCurveConstraint::restTranslate;
MObject PointOnCurveConstraint::restTranslateX;
MObject PointOnCurveConstraint::restTranslateY;
MObject PointOnCurveConstraint::restTranslateZ;
MObject PointOnCurveConstraint::restRotate;
MObject PointOnCurveConstraint::restRotateX;
MObject PointOnCurveConstraint::restRotateY;
MObject PointOnCurveConstraint::restRotateZ;

MObject PointOnCurveConstraint::target;
MObject PointOnCurveConstraint::targetWeight;
MObject PointOnCurveConstraint::targetParameter;
MObject PointOnCurveConstraint::targetUseFraction;
MObject PointOnCurveConstraint::targetLoop;
MObject PointOnCurveConstraint::targetCurve;

MObject PointOnCurveConstraint::constraintTranslate;
MObject PointOnCurveConstraint::constraintTranslateX;
MObject PointOnCurveConstraint::constraintTranslateY;
MObject PointOnCurveConstraint::constraintTranslateZ;
MObject PointOnCurveConstraint::constraintRotate;
MObject PointOnCurveConstraint::constraintRotateX;
MObject PointOnCurveConstraint::constraintRotateY;
MObject PointOnCurveConstraint::constraintRotateZ;
MObject PointOnCurveConstraint::constraintRotateOrder;
MObject	PointOnCurveConstraint::constraintJointOrient;
MObject	PointOnCurveConstraint::constraintJointOrientX;
MObject	PointOnCurveConstraint::constraintJointOrientY;
MObject	PointOnCurveConstraint::constraintJointOrientZ;
MObject PointOnCurveConstraint::constraintMatrix;
MObject PointOnCurveConstraint::constraintInverseMatrix;
MObject PointOnCurveConstraint::constraintWorldMatrix;
MObject PointOnCurveConstraint::constraintWorldInverseMatrix;
MObject PointOnCurveConstraint::constraintParentInverseMatrix;

MString	PointOnCurveConstraint::inputCategory("Input");
MString	PointOnCurveConstraint::offsetCategory("Offset");
MString	PointOnCurveConstraint::restCategory("Rest");
MString	PointOnCurveConstraint::targetCategory("Target");
MString	PointOnCurveConstraint::outputCategory("Output");

MString	PointOnCurveConstraint::classification("animation");
MTypeId PointOnCurveConstraint::id(0x0013b1ec);


template<class N> N limit(const N value, const N min, const N max)
/**
Clamps the supplied number between the specified range.

@param value: The value to clamp.
@param min: The minimum value.
@param max: The maximum value.
@return: The clamped value.
*/
{

	return (value < min) ? min : (value > max) ? max : value;

};


template<class N> N lerp(const N start, const N end, const double weight)
/**
Linearly interpolates the two given numbers using the supplied weight.

@param start: The start number.
@param end: The end number.
@param weight: The amount to blend.
@return: The interpolated value.
*/
{

	return (start * (1.0 - weight)) + (end * weight);

};


template<class N> N loop(const N value, const N min, const N max)
/**
Loops the supplied number to ensure it is within range.

@param start: The value to loop.
@param min: The minimum value.
@param max: The maximum value.
@return: The looped value.
*/
{

	N range = max - min;
	double division = static_cast<double>(value) / static_cast<double>(range);
	long quotient = static_cast<long>(std::floor(division));

	return value - (range * static_cast<N>(quotient));

};


PointOnCurveConstraint::PointOnCurveConstraint() {}
PointOnCurveConstraint::~PointOnCurveConstraint() {};


MStatus PointOnCurveConstraint::compute(const MPlug& plug, MDataBlock& data)
/**
This method should be overridden in user defined nodes.
Recompute the given output based on the nodes inputs.
The plug represents the data value that needs to be recomputed, and the data block holds the storage for all of the node's attributes.
The MDataBlock will provide smart handles for reading and writing this node's attribute values.
Only these values should be used when performing computations!

@param plug: Plug representing the attribute that needs to be recomputed.
@param data: Data block containing storage for the node's attributes.
@return: Return status.
*/
{

	MStatus status;

	// Check requested attribute
	//
	MObject attribute = plug.attribute(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnAttribute fnAttribute(attribute, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	if (fnAttribute.hasCategory(PointOnCurveConstraint::outputCategory))
	{

		// Get input data handles
		//
		MDataHandle forwardVectorHandle = data.inputValue(PointOnCurveConstraint::forwardVector, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle upVectorHandle = data.inputValue(PointOnCurveConstraint::upVector, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle worldUpTypeHandle = data.inputValue(PointOnCurveConstraint::worldUpType, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle worldUpVectorHandle = data.inputValue(PointOnCurveConstraint::worldUpVector, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle worldUpMatrixHandle = data.inputValue(PointOnCurveConstraint::worldUpMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle twistHandle = data.inputValue(PointOnCurveConstraint::twist, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle offsetTranslateHandle = data.inputValue(PointOnCurveConstraint::offsetTranslate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle offsetRotateHandle = data.inputValue(PointOnCurveConstraint::offsetRotate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle enableRestPositionHandle = data.inputValue(PointOnCurveConstraint::enableRestPosition, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle restTranslateHandle = data.inputValue(PointOnCurveConstraint::restTranslate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle restRotateHandle = data.inputValue(PointOnCurveConstraint::restRotate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintRotateOrderHandle = data.inputValue(PointOnCurveConstraint::constraintRotateOrder, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintJointOrientHandle = data.inputValue(PointOnCurveConstraint::constraintJointOrient, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintParentInverseMatrixHandle = data.inputValue(PointOnCurveConstraint::constraintParentInverseMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MArrayDataHandle targetArrayHandle = data.inputArrayValue(PointOnCurveConstraint::target, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Get values from handles
		//
		WorldUpType worldUpType = WorldUpType(worldUpTypeHandle.asShort());
		MVector worldUpVector = worldUpVectorHandle.asVector();
		MMatrix worldUpMatrix = worldUpMatrixHandle.asMatrix();
		WorldUpSettings worldUpSettings = { worldUpType, worldUpVector, worldUpMatrix };

		MVector forwardVector = forwardVectorHandle.asVector();
		MVector upVector = upVectorHandle.asVector();
		MAngle twistAngle = twistHandle.asAngle();
		AxisSettings axisSettings = AxisSettings{ forwardVector, upVector, twistAngle, worldUpSettings };

		MEulerRotation::RotationOrder constraintRotateOrder = MEulerRotation::RotationOrder(constraintRotateOrderHandle.asShort());
		MEulerRotation constraintJointOrient = MEulerRotation(constraintJointOrientHandle.asVector(), constraintRotateOrder);
		MMatrix constraintJointOrientMatrix = constraintJointOrient.asMatrix();
		MMatrix constraintParentInverseMatrix = constraintParentInverseMatrixHandle.asMatrix();
		MMatrix constraintParentMatrix = constraintParentInverseMatrix.inverse();

		MVector offsetTranslate = offsetTranslateHandle.asVector();
		MMatrix offsetTranslateMatrix = PointOnCurveConstraint::createPositionMatrix(offsetTranslate);
		MVector offsetRotate = offsetRotateHandle.asVector();
		MMatrix offsetRotateMatrix = MEulerRotation(offsetRotate, constraintRotateOrder).asMatrix();
		MMatrix offsetMatrix = offsetRotateMatrix * offsetTranslateMatrix;

		bool restEnabled = enableRestPositionHandle.asBool();
		MVector restTranslate = restTranslateHandle.asVector();
		MMatrix restTranslateMatrix = PointOnCurveConstraint::createPositionMatrix(restTranslate);
		MEulerRotation restRotate = MEulerRotation(restRotateHandle.asVector(), constraintRotateOrder);
		MMatrix restRotateMatrix = restRotate.asMatrix();
		MMatrix restMatrix = restEnabled ? (restRotateMatrix * restTranslateMatrix) : MMatrix::identity;
		MMatrix restWorldMatrix = restMatrix * constraintParentMatrix;

		// Initialize target matrices
		//
		unsigned int targetCount = targetArrayHandle.elementCount();

		MFloatArray targetWeights = MFloatArray(targetCount);
		MMatrixArray targetMatrices = MMatrixArray(targetCount);

		// Iterate through targets
		//
		MDataHandle targetHandle, targetWeightHandle, targetParameterHandle, targetUseFractionHandle, targetLoopHandle, targetCurveHandle;

		MObject curve;
		MFnNurbsCurve fnCurve;
		double parameter, initialParameter, minParameter, maxParameter;
		bool looping, fractional;
		MMatrix targetMatrix;
		
		for (unsigned int i = 0; i < targetCount; i++)
		{

			// Jump to array element
			//
			status = targetArrayHandle.jumpToArrayElement(i);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			targetHandle = targetArrayHandle.inputValue(&status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			// Get target data handles
			//
			targetWeightHandle = targetHandle.child(PointOnCurveConstraint::targetWeight);
			targetParameterHandle = targetHandle.child(PointOnCurveConstraint::targetParameter);
			targetUseFractionHandle = targetHandle.child(PointOnCurveConstraint::targetUseFraction);
			targetLoopHandle = targetHandle.child(PointOnCurveConstraint::targetLoop);
			targetCurveHandle = targetHandle.child(PointOnCurveConstraint::targetCurve);

			// Get weight value
			//
			targetWeights[i] = limit(targetWeightHandle.asFloat(), 0.0f, 1.0f);

			// Get curve parameter range
			//
			curve = targetCurveHandle.asNurbsCurve();

			status = fnCurve.setObject(curve);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			status = fnCurve.getKnotDomain(minParameter, maxParameter);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			// Interpret requested parameter
			//
			looping = targetLoopHandle.asBool();
			fractional = targetUseFractionHandle.asBool();
			initialParameter = targetParameterHandle.asDouble();

			if (looping)
			{

				parameter = loop(fractional ? lerp(minParameter, maxParameter, initialParameter) : initialParameter, minParameter, maxParameter);

			}
			else
			{

				parameter = fractional ? lerp(minParameter, maxParameter, limit(initialParameter, 0.0, 1.0)) : limit(initialParameter, minParameter, maxParameter);

			}

			// Create matrix from curve at parameter
			//
			status = PointOnCurveConstraint::createMatrixFromCurve(curve, parameter, axisSettings, targetMatrix);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			targetMatrices[i] = targetMatrix;

		}

		// Calculate weighted constraint matrix
		//
		MMatrix matrix = PointOnCurveConstraint::blendMatrices(restWorldMatrix, targetMatrices, targetWeights);

		MMatrix constraintWorldMatrix = offsetMatrix * matrix;
		MMatrix constraintMatrix = constraintWorldMatrix * constraintParentInverseMatrix;

		// Get output data handles
		//
		MDataHandle constraintTranslateXHandle = data.outputValue(PointOnCurveConstraint::constraintTranslateX, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintTranslateYHandle = data.outputValue(PointOnCurveConstraint::constraintTranslateY, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintTranslateZHandle = data.outputValue(PointOnCurveConstraint::constraintTranslateZ, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintRotateXHandle = data.outputValue(PointOnCurveConstraint::constraintRotateX, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintRotateYHandle = data.outputValue(PointOnCurveConstraint::constraintRotateY, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintRotateZHandle = data.outputValue(PointOnCurveConstraint::constraintRotateZ, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintMatrixHandle = data.outputValue(PointOnCurveConstraint::constraintMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintInverseMatrixHandle = data.outputValue(PointOnCurveConstraint::constraintInverseMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintWorldMatrixHandle = data.outputValue(PointOnCurveConstraint::constraintWorldMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintWorldInverseMatrixHandle = data.outputValue(PointOnCurveConstraint::constraintWorldInverseMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Update constraint translation
		//
		MVector constraintTranslate = PointOnCurveConstraint::getTranslationPart(constraintMatrix);

		constraintTranslateXHandle.setMDistance(MDistance(constraintTranslate.x, MDistance::kCentimeters));
		constraintTranslateYHandle.setMDistance(MDistance(constraintTranslate.y, MDistance::kCentimeters));
		constraintTranslateZHandle.setMDistance(MDistance(constraintTranslate.z, MDistance::kCentimeters));

		constraintTranslateXHandle.setClean();
		constraintTranslateYHandle.setClean();
		constraintTranslateZHandle.setClean();

		// Update constraint rotation
		//
		MMatrix constraintRotateMatrix = PointOnCurveConstraint::createRotationMatrix(constraintMatrix) * constraintJointOrientMatrix.inverse();

		MEulerRotation constraintRotate = PointOnCurveConstraint::getRotationPart(constraintRotateMatrix).asEulerRotation();
		constraintRotate.reorderIt(constraintRotateOrder);

		constraintRotateXHandle.setMAngle(MAngle(constraintRotate.x, MAngle::kRadians));
		constraintRotateYHandle.setMAngle(MAngle(constraintRotate.y, MAngle::kRadians));
		constraintRotateZHandle.setMAngle(MAngle(constraintRotate.z, MAngle::kRadians));

		constraintRotateXHandle.setClean();
		constraintRotateYHandle.setClean();
		constraintRotateZHandle.setClean();

		// Update constraint matrices
		//
		constraintMatrixHandle.setMMatrix(constraintMatrix);
		constraintInverseMatrixHandle.setMMatrix(constraintMatrix.inverse());
		constraintWorldMatrixHandle.setMMatrix(constraintWorldMatrix);
		constraintWorldInverseMatrixHandle.setMMatrix(constraintWorldMatrix.inverse());

		constraintMatrixHandle.setClean();
		constraintInverseMatrixHandle.setClean();
		constraintWorldMatrixHandle.setClean();
		constraintWorldInverseMatrixHandle.setClean();

		// Mark data block as clean
		//
		status = data.setClean(plug);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;

	}
	else
	{

		return MS::kUnknownParameter;

	}

};


MStatus	PointOnCurveConstraint::createMatrixFromCurve(const MObject& curve, const double parameter, const AxisSettings& settings, MMatrix& matrix)
/**
Samples the supplied curve at the specified parameter.

@param curve: The curve data object to sample from.
@param parameter: The curve parameter to sample at.
@param point: The position at the specified parameter.
@param forwardVector: The forward-vector at the specified parameter.
@param upVector: The up-vector at the specified parameter.
@return: Return status.
*/
{

	MStatus status;

	// Get curve point
	//
	MPoint origin = MPoint::origin;

	status = PointOnCurveConstraint::getCurvePoint(curve, parameter, origin);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Get forward-vector
	//
	MVector forwardVector = MVector::xAxis;

	status = PointOnCurveConstraint::getForwardVector(curve, parameter, forwardVector);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Get up-vector
	//
	MVector upVector = MVector::yAxis;

	status = PointOnCurveConstraint::getUpVector(curve, parameter, settings.worldUpSettings, origin, upVector);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Compose matrix
	//
	MVector forwardAxis = settings.forwardAxis.normal();
	MVector rightAxis = (forwardAxis ^ settings.upAxis.normal()).normal();
	MVector upAxis = (rightAxis ^ forwardAxis).normal();
	MMatrix axisMatrix = PointOnCurveConstraint::composeMatrix(forwardAxis, upAxis, rightAxis, MPoint::origin);

	MVector rightVector = (forwardVector ^ upVector).normal();
	MVector altUpVector = (rightVector ^ forwardVector).normal();
	MMatrix aimMatrix = PointOnCurveConstraint::composeMatrix(forwardVector, altUpVector, rightVector, origin);

	MMatrix twistMatrix = MQuaternion(settings.twistAngle.asRadians(), forwardAxis).asMatrix();

	matrix = twistMatrix * axisMatrix * aimMatrix;

	return status;

};


double PointOnCurveConstraint::clampCurveParameter(const MObject& curve, const double parameter, MStatus* status)
/**
Returns a clamped parameter based on the supplied curve.

@param curve: The curve data object to sample from.
@param parameter: The curve parameter to clamp.
@param forwardVector: Return status.
@return: The clamped parameter.
*/
{

	// Initialize function set
	//
	MFnNurbsCurve fnCurve(curve, status);
	CHECK_MSTATUS_AND_RETURN(*status, parameter);

	// Get curve parameter range
	//
	double minParameter, maxParameter;

	*status = fnCurve.getKnotDomain(minParameter, maxParameter);
	CHECK_MSTATUS_AND_RETURN(*status, parameter);

	// Clamp parameter within range
	//
	return limit(parameter, (minParameter + 1e-3), (maxParameter - 1e-3));

};


MStatus PointOnCurveConstraint::getForwardVector(const MObject& curve, const double parameter, MVector& forwardVector)
/**
Returns the forward vector at the specified parameter.

@param curve: The curve data object to sample from.
@param parameter: The curve parameter to sample at.
@param forwardVector: The passed vector to populate.
@return: Return status.
*/
{

	MStatus status;

	// Clamp curve parameter
	//
	double clampedParameter = PointOnCurveConstraint::clampCurveParameter(curve, parameter, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Evaluate tangent at parameter
	//	
	MFnNurbsCurve fnCurve(curve, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	forwardVector = fnCurve.tangent(clampedParameter, MSpace::kWorld, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = forwardVector.normalize();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;

};


MStatus PointOnCurveConstraint::getUpVector(const MObject& curve, const double parameter, const WorldUpSettings& settings, const MVector& origin, MVector& upVector)
/**
Returns the up vector based on the selected world up type.

@param data: Data block containing storage for the node's attributes.
@param parameter: The curve parameter to sample at.
@param curve: The curve data object to sample from.
@param upVector: The passed vector to populate.
@return: Return status.
*/
{

	MStatus status;

	// Evaluate world-up type
	//
	switch (settings.worldUpType)
	{

	case WorldUpType::SceneUp:
	{

		upVector = MGlobal::upAxis();

	}
	break;

	case WorldUpType::ObjectUp:
	{

		upVector = (MVector(settings.worldUpMatrix[3]) - MVector(origin)).normal();

	}
	break;

	case WorldUpType::ObjectRotationUp:
	{

		upVector = PointOnCurveConstraint::getObjectRotationUpVector(settings.worldUpVector, settings.worldUpMatrix);

	}
	break;

	case WorldUpType::Vector:
	{

		upVector = MVector(settings.worldUpVector).normal();

	}
	break;

	case WorldUpType::CurveNormal:
	{

		status = PointOnCurveConstraint::getCurveNormal(curve, parameter, upVector);
		CHECK_MSTATUS_AND_RETURN_IT(status);

	}
	break;

	default:
	{

		return MS::kFailure;

	}
	break;

	}

	return MS::kSuccess;

};


MVector PointOnCurveConstraint::getObjectRotationUpVector(const MVector& worldUpVector, const MMatrix& worldUpMatrix)
/**
Returns the weighted up-vector derived from the supplied world-matrix.

@param worldUpVector: The weighted values to average from.
@param worldUpMatrix: The world matrix of the up object.
@return: MVector
*/
{

	// Extract axis vectors
	//
	MVector xAxis = MVector(worldUpMatrix[0]);
	MVector yAxis = MVector(worldUpMatrix[1]);
	MVector zAxis = MVector(worldUpMatrix[2]);

	// Calculate weighted vector average
	//
	return MVector((xAxis * worldUpVector.x) + (yAxis * worldUpVector.y) + (zAxis * worldUpVector.z)).normal();

};


MStatus	PointOnCurveConstraint::getCurvePoint(const MObject& curve, const double parameter, MPoint& position)
/**
Returns the point on a curve from the given percentile.

@param curve: The curve data object to sample from.
@param parameter: The curve parameter to sample at.
@param position: The passed point to populate.
@return: Return status.
*/
{

	MStatus status;

	// Clamp curve parameter
	//
	double clampedParameter = PointOnCurveConstraint::clampCurveParameter(curve, parameter, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Evaluate point at parameter
	//	
	MFnNurbsCurve fnCurve(curve, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = fnCurve.getPointAtParam(parameter, position, MSpace::kWorld);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return status;

};


MStatus	PointOnCurveConstraint::getCurveNormal(const MObject& curve, const double parameter, MVector& upVector)
/**
Returns the tangent normal at the given percentile.

@param curve: The curve data object to sample from.
@param parameter: The curve parameter to sample at.
@param upVector: The passed vector to populate.
@return: Return status.
*/
{

	MStatus status;

	// Clamp curve parameter
	//
	double clampedParameter = PointOnCurveConstraint::clampCurveParameter(curve, parameter, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Evaluate normal at parameter
	//	
	MFnNurbsCurve fnCurve(curve, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	upVector = fnCurve.normal(clampedParameter, MSpace::kWorld, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return status;

};


MMatrix PointOnCurveConstraint::composeMatrix(const MVector& xAxis, const MVector& yAxis, const MVector& zAxis, const MPoint& position)
/**
Returns a matrix from the supplied axis vectors and position.

@param xAxis: The x-axis vector.
@param yAxis: The y-axis vector.
@param zAxis: The z-axis vector.
@param position: The positional value.
@return: Transform matrix.
*/
{

	double rows[4][4] = {
		{xAxis.x, xAxis.y, xAxis.z, 0.0},
		{yAxis.x, yAxis.y, yAxis.z, 0.0},
		{zAxis.x, zAxis.y, zAxis.z, 0.0},
		{position.x, position.y, position.z, 1.0}
	};

	return MMatrix(rows);

};


MStatus PointOnCurveConstraint::decomposeMatrix(const MMatrix& matrix, MVector& position, MQuaternion& rotation, MVector& scale)
/**
Returns the translate, rotate and scale components from the supplied transform matrix.

@param matrix: The transform matrix to extract from.
@param position: The translation component.
@param rotation: The rotation component.
@param scale: The scale component.
@return: Status code.
*/
{

	position = PointOnCurveConstraint::getTranslationPart(matrix);
	rotation = PointOnCurveConstraint::getRotationPart(matrix);
	scale = PointOnCurveConstraint::getScalePart(matrix);

	return MS::kSuccess;

};


MVector PointOnCurveConstraint::getTranslationPart(const MMatrix& matrix)
/**
Returns the translation component from the supplied transform matrix.

@param matrix: The transform matrix to extract from.
@return: The translation component.
*/
{

	return MVector(matrix(3, 0), matrix(3, 1), matrix(3, 2));

};


MQuaternion PointOnCurveConstraint::getRotationPart(const MMatrix& matrix)
/**
Returns the rotation component from the supplied transform matrix.

@param matrix: The transform matrix to extract from.
@return: The rotation component.
*/
{

	MQuaternion rotationPart;
	rotationPart = PointOnCurveConstraint::createRotationMatrix(matrix);

	return rotationPart;

};


MVector PointOnCurveConstraint::getScalePart(const MMatrix& matrix)
/**
Returns the scale component from the supplied transform matrix.

@param matrix: The transform matrix to extract from.
@return: The scale component.
*/
{

	MVector xAxis = MVector(matrix(0, 0), matrix(0, 1), matrix(0, 2));
	MVector yAxis = MVector(matrix(1, 0), matrix(1, 1), matrix(1, 2));
	MVector zAxis = MVector(matrix(2, 0), matrix(2, 1), matrix(2, 2));

	return MVector(xAxis.length(), yAxis.length(), zAxis.length());

};


MMatrix PointOnCurveConstraint::createPositionMatrix(const MPoint& position)
/**
Creates a position matrix from the given point.

@param position: The point to convert.
@return: The new position matrix.
*/
{

	double matrixRows[4][4] = {
		{ 1.0, 0.0, 0.0, 0.0 },
		{ 0.0, 1.0, 0.0, 0.0 },
		{ 0.0, 0.0, 1.0, 0.0 },
		{ position.x, position.y, position.z, position.w },
	};

	return MMatrix(matrixRows);

};


MMatrix PointOnCurveConstraint::createPositionMatrix(const MVector& position)
/**
Creates a position matrix from the given vector.

@param position: The vector to convert.
@return: The new position matrix.
*/
{

	return PointOnCurveConstraint::createPositionMatrix(MPoint(position));

};


MMatrix PointOnCurveConstraint::createPositionMatrix(const MMatrix& matrix)
/**
Returns the position component from the supplied transform matrix.

@param position: The transform matrix to extract from.
@return: The new position matrix.
*/
{

	return PointOnCurveConstraint::createPositionMatrix(PointOnCurveConstraint::getTranslationPart(matrix));

};


MMatrix PointOnCurveConstraint::createRotationMatrix(const MMatrix& matrix)
/**
Returns the rotation component from the supplied transform matrix.

@param matrix: The transform matrix to extract from.
@return: The new rotation matrix.
*/
{

	MVector xAxis = MVector(matrix(0, 0), matrix(0, 1), matrix(0, 2));
	MVector yAxis = MVector(matrix(1, 0), matrix(1, 1), matrix(1, 2));
	MVector zAxis = MVector(matrix(2, 0), matrix(2, 1), matrix(2, 2));

	double matrixRows[4][4] = {
		{ xAxis.x, xAxis.y, xAxis.z, 0.0 },
		{ yAxis.x, yAxis.y, yAxis.z, 0.0 },
		{ zAxis.x, zAxis.y, zAxis.z, 0.0 },
		{ 0.0, 0.0, 0.0, 1.0 },
	};

	return MMatrix(matrixRows);

};


MMatrix PointOnCurveConstraint::createScaleMatrix(const MVector& scale)
/**
Returns a scale matrix from the supplied vector.

@param scale: The vector to convert.
@return: The new scale matrix.
*/
{

	double matrixRows[4][4] = {
		{ scale.x, 0.0, 0.0, 0.0 },
		{ 0.0, scale.y, 0.0, 0.0 },
		{ 0.0, 0.0, scale.z, 0.0 },
		{ 0.0, 0.0, 0.0, 1.0 },
	};

	return MMatrix(matrixRows);

};


MMatrix PointOnCurveConstraint::createScaleMatrix(const MMatrix& matrix)
/**
Returns a scale matrix from the supplied transform matrix.

@param matrix: The transform matrix to extract from.
@return: The new scale matrix.
*/
{

	return PointOnCurveConstraint::createScaleMatrix(PointOnCurveConstraint::getScalePart(matrix));

};


double PointOnCurveConstraint::dot(const MQuaternion& quat, const MQuaternion& otherQuat)
/**
Returns the dot product of two quaternions.

@param quat: Quaternion.
@param: otherQuat: Other quaternion.
@return: Dot length.
*/
{

	return (quat.x * otherQuat.x) + (quat.y * otherQuat.y) + (quat.z * otherQuat.z) + (quat.w * otherQuat.w);

};


MQuaternion PointOnCurveConstraint::slerp(const MQuaternion& startQuat, const MQuaternion& endQuat, const float weight)
/**
Spherical interpolates two quaternions.
See the following for details: https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm

@param startQuat: Start Quaternion.
@param endQuat: End Quaternion.
@param weight: The amount to interpolate.
@return: The interpolated quaternion.
*/
{

	MQuaternion q1 = MQuaternion(startQuat);
	MQuaternion q2 = MQuaternion(endQuat);

	double dot = PointOnCurveConstraint::dot(q1, q2);

	if (dot < 0.0)
	{

		dot = PointOnCurveConstraint::dot(q1, q2.negateIt());

	}

	double theta = acos(dot);
	double sinTheta = sin(theta);

	double w1, w2;

	if (sinTheta > 1e-3)
	{

		w1 = sin((1.0 - weight) * theta) / sinTheta;
		w2 = sin(weight * theta) / sinTheta;

	}
	else
	{

		w1 = 1.0 - weight;
		w2 = weight;

	}

	q1.scaleIt(w1);
	q2.scaleIt(w2);

	return q1 + q2;

};


float PointOnCurveConstraint::sum(const MFloatArray& items)
/**
Calculates the sum of all the supplied items.

@param items: The items to add up.
@return: The total sum.
*/
{

	// Iterate through numbers
	//
	unsigned int numItems = items.length();
	float sum = 0.0;

	for (unsigned int i = 0; i < numItems; i++)
	{

		sum += items[i];

	}

	return sum;

};


MFloatArray PointOnCurveConstraint::clamp(const MFloatArray& items)
/**
Clamps the supplied items so they don't exceed 1.
Anything below that is left alone and compensated for using the rest matrix.

@param items: The float array containing the weighted averages.
@return: The newly clamped array of weights.
*/
{

	// Check if sum is greater than one
	//
	float sum = PointOnCurveConstraint::sum(items);

	if (sum < 1.0)
	{

		return MFloatArray(items);

	}

	// Iterate through numbers
	//
	float fraction = 1.0f / sum;

	unsigned int numItems = items.length();
	MFloatArray normalizedItems = MFloatArray(numItems);

	for (unsigned int i = 0; i < numItems; i++)
	{

		normalizedItems[i] = items[i] * fraction;

	}

	return normalizedItems;


};


MMatrix PointOnCurveConstraint::blendMatrices(const MMatrix& startMatrix, const MMatrix& endMatrix, const float weight)
/**
Interpolates the two given matrices using the supplied weight.
Both translate and scale will be lerp'd while rotation will be slerp'd.

@param startMatrix: The start matrix.
@param endMatrix: The end matrix.
@param weight: The amount to blend.
@return: The interpolated matrix.
*/
{

	MStatus status;

	// Decompose transform matrices
	//
	MVector startTranslation, endTranslation;
	MVector startScale, endScale;
	MQuaternion startQuat, endQuat;

	PointOnCurveConstraint::decomposeMatrix(startMatrix, startTranslation, startQuat, startScale);
	PointOnCurveConstraint::decomposeMatrix(endMatrix, endTranslation, endQuat, endScale);

	// Interpolate translation
	//
	MVector translation = lerp(startTranslation, endTranslation, weight);
	MQuaternion quat = PointOnCurveConstraint::slerp(startQuat, endQuat, weight);
	MVector scale = lerp(startScale, endScale, weight);

	// Compose interpolated matrix
	//
	MMatrix translateMatrix = PointOnCurveConstraint::createPositionMatrix(translation);
	MMatrix rotateMatrix = quat.asMatrix();
	MMatrix scaleMatrix = PointOnCurveConstraint::createScaleMatrix(scale);

	return scaleMatrix * rotateMatrix * translateMatrix;

};


MMatrix PointOnCurveConstraint::blendMatrices(const MMatrix& restMatrix, const MMatrixArray& matrices, const MFloatArray& weights)
/**
Interpolates the supplied matrices using the weight array as a blend aplha.
The rest matrix is used just in case the weights don't equal 1.

@param restMatrix: The default matrix to blend from in case the weights don't equal 1.
@param matrices: The matrix array to blend.
@param weights: The float array containing the weighted averages.
@return: The interpolated matrix.
*/
{

	// Check the number of matrices
	//
	unsigned int numMatrices = matrices.length();

	switch (numMatrices)
	{

	case 0:
	{

		// Reuse rest matrix
		//
		return MMatrix(restMatrix);

	}
	break;

	case 1:
	{

		// Check weight sum before committing to either matrix
		//
		float weightSum = PointOnCurveConstraint::sum(weights);

		if (weightSum == 1.0f)
		{

			return MMatrix(matrices[0]);

		}
		else if (weightSum == 0.f)
		{

			return MMatrix(restMatrix);

		}
		else
		{

			return PointOnCurveConstraint::blendMatrices(restMatrix, matrices[0], weights[0]);

		}

	}
	break;

	default:
	{

		// Get start matrix
		//
		MFloatArray clampedWeights = PointOnCurveConstraint::clamp(weights);
		float weightSum = PointOnCurveConstraint::sum(clampedWeights);

		MMatrix matrix = MMatrix(matrices[0]);

		if (weightSum < 1.0f)
		{

			matrix = MMatrix(restMatrix);

		}

		// Get start transform components
		//
		unsigned int numMatrices = matrices.length();

		for (unsigned int i = 0; i < numMatrices; i++)
		{

			matrix = PointOnCurveConstraint::blendMatrices(matrix, matrices[i], weights[i]);

		}

		return matrix;

	}
	break;

	}

	return MMatrix::identity;

};


const MObject PointOnCurveConstraint::targetAttribute() const
/**
Returns the target attribute for the constraint.
Default implementation returns MObject::kNullObj.

@return: MObject
*/
{


	return PointOnCurveConstraint::target;

};


const MObject PointOnCurveConstraint::weightAttribute() const
/**
Returns the weight attribute for the constraint.
Default implementation returns MObject::kNullObj.

@return: MObject
*/
{


	return PointOnCurveConstraint::targetWeight;

};


const MObject PointOnCurveConstraint::constraintRotateOrderAttribute() const
/**
Returns the rotate order attribute for the constraint.
Default implementation returns MObject::kNullObj.

@return: MObject
*/
{


	return PointOnCurveConstraint::constraintRotateOrder;

};


void* PointOnCurveConstraint::creator()
/**
This function is called by Maya when a new instance is requested.
See pluginMain.cpp for details.

@return: PointOnCurveConstraint
*/
{

	return new PointOnCurveConstraint();

};


MStatus PointOnCurveConstraint::initialize()
/**
This function is called by Maya after a plugin has been loaded.
Use this function to define any static attributes.

@return: MStatus
*/
{

	MStatus	status;

	// Declare attribute function sets
	//
	MFnCompoundAttribute fnCompoundAttr;
	MFnNumericAttribute fnNumericAttr;
	MFnUnitAttribute fnUnitAttr;
	MFnTypedAttribute fnTypedAttr;
	MFnEnumAttribute fnEnumAttr;
	MFnMatrixAttribute fnMatrixAttr;

	// Input attributes:
	// ".forwardVectorX" attribute
	//
	PointOnCurveConstraint::forwardVectorX = fnNumericAttr.create("forwardVectorX", "fvx", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".forwardVectorY" attribute
	//
	PointOnCurveConstraint::forwardVectorY = fnNumericAttr.create("forwardVectorY", "fvy", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".forwardVectorZ" attribute
	//
	PointOnCurveConstraint::forwardVectorZ = fnNumericAttr.create("forwardVectorZ", "fvz", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".forwardVector" attribute
	//
	PointOnCurveConstraint::forwardVector = fnNumericAttr.create("forwardVector", "fv", PointOnCurveConstraint::forwardVectorX, PointOnCurveConstraint::forwardVectorY, PointOnCurveConstraint::forwardVectorZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".upVectorX" attribute
	//
	PointOnCurveConstraint::upVectorX = fnNumericAttr.create("upVectorX", "uvx", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".upVectorY" attribute
	//
	PointOnCurveConstraint::upVectorY = fnNumericAttr.create("upVectorY", "uvy", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".upVectorZ" attribute
	//
	PointOnCurveConstraint::upVectorZ = fnNumericAttr.create("upVectorZ", "uvz", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".upVector" attribute
	//
	PointOnCurveConstraint::upVector = fnNumericAttr.create("upVector", "uv", PointOnCurveConstraint::upVectorX, PointOnCurveConstraint::upVectorY, PointOnCurveConstraint::upVectorZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".worldUpType" attribute
	//
	PointOnCurveConstraint::worldUpType = fnEnumAttr.create("worldUpType", "wut", short(0), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("Scene Up", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("Object Up", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("Object Rotation Up", 2));
	CHECK_MSTATUS(fnEnumAttr.addField("Vector", 3));
	CHECK_MSTATUS(fnEnumAttr.addField("Normal", 4));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".worldUpVectorX" attribute
	//
	PointOnCurveConstraint::worldUpVectorX = fnUnitAttr.create("worldUpVectorX", "wuvx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".worldUpVectorY" attribute
	//
	PointOnCurveConstraint::worldUpVectorY = fnUnitAttr.create("worldUpVectorY", "wuvy", MFnUnitAttribute::kDistance, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".worldUpVectorZ" attribute
	//
	PointOnCurveConstraint::worldUpVectorZ = fnUnitAttr.create("worldUpVectorZ", "wuvz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".worldUpVector" attribute
	//
	PointOnCurveConstraint::worldUpVector = fnNumericAttr.create("worldUpVector", "wuv", PointOnCurveConstraint::worldUpVectorX, PointOnCurveConstraint::worldUpVectorY, PointOnCurveConstraint::worldUpVectorZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".worldUpMatrix" attribute
	//
	PointOnCurveConstraint::worldUpMatrix = fnMatrixAttr.create("worldUpMatrix", "wum", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".twist" attribute
	//
	PointOnCurveConstraint::twist = fnUnitAttr.create("twist", "twst", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".offsetTranslateX" attribute
	//
	PointOnCurveConstraint::offsetTranslateX = fnUnitAttr.create("offsetTranslateX", "otx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::offsetCategory));

	// ".offsetTranslateY" attribute
	//
	PointOnCurveConstraint::offsetTranslateY = fnUnitAttr.create("offsetTranslateY", "oty", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::offsetCategory));

	// ".offsetTranslateZ" attribute
	//
	PointOnCurveConstraint::offsetTranslateZ = fnUnitAttr.create("offsetTranslateZ", "otz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::offsetCategory));

	// ".offsetTranslate" attribute
	//
	PointOnCurveConstraint::offsetTranslate = fnNumericAttr.create("offsetTranslate", "ot", PointOnCurveConstraint::offsetTranslateX, PointOnCurveConstraint::offsetTranslateY, PointOnCurveConstraint::offsetTranslateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::offsetCategory));

	// ".offsetRotateX" attribute
	//
	PointOnCurveConstraint::offsetRotateX = fnUnitAttr.create("offsetRotateX", "orx", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::offsetCategory));

	// ".offsetRotateY" attribute
	//
	PointOnCurveConstraint::offsetRotateY = fnUnitAttr.create("offsetRotateY", "ory", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::offsetCategory));

	// ".offsetRotateZ" attribute
	//
	PointOnCurveConstraint::offsetRotateZ = fnUnitAttr.create("offsetRotateZ", "orz", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::offsetCategory));

	// ".offsetRotate" attribute
	//
	PointOnCurveConstraint::offsetRotate = fnNumericAttr.create("offsetRotate", "or", PointOnCurveConstraint::offsetRotateX, PointOnCurveConstraint::offsetRotateY, PointOnCurveConstraint::offsetRotateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::offsetCategory));

	// ".enableRestPosition" attribute
	//
	status = fnNumericAttr.setObject(PointOnCurveConstraint::enableRestPosition);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::restCategory));

	// ".restTranslateX" attribute
	//
	PointOnCurveConstraint::restTranslateX = fnUnitAttr.create("restTranslateX", "rtx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::restCategory));

	// ".restTranslateY" attribute
	//
	PointOnCurveConstraint::restTranslateY = fnUnitAttr.create("restTranslateY", "rty", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::restCategory));

	// ".restTranslateZ" attribute
	//
	PointOnCurveConstraint::restTranslateZ = fnUnitAttr.create("restTranslateZ", "rtz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::restCategory));

	// ".restTranslate" attribute
	//
	PointOnCurveConstraint::restTranslate = fnNumericAttr.create("restTranslate", "rt", PointOnCurveConstraint::restTranslateX, PointOnCurveConstraint::restTranslateY, PointOnCurveConstraint::restTranslateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::restCategory));

	// ".restRotateX" attribute
	//
	PointOnCurveConstraint::restRotateX = fnUnitAttr.create("restRotateX", "rrx", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::restCategory));

	// ".restRotateY" attribute
	//
	PointOnCurveConstraint::restRotateY = fnUnitAttr.create("restRotateY", "rry", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::restCategory));

	// ".restRotateZ" attribute
	//
	PointOnCurveConstraint::restRotateZ = fnUnitAttr.create("restRotateZ", "rrz", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::restCategory));

	// ".restRotate" attribute
	//
	PointOnCurveConstraint::restRotate = fnNumericAttr.create("restRotate", "rr", PointOnCurveConstraint::restRotateX, PointOnCurveConstraint::restRotateY, PointOnCurveConstraint::restRotateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::restCategory));

	// ".constraintRotateOrder" attribute
	//
	PointOnCurveConstraint::constraintRotateOrder = fnEnumAttr.create("constraintRotateOrder", "cro", short(0), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("xyz", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("yzx", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("zxy", 2));
	CHECK_MSTATUS(fnEnumAttr.addField("xzy", 3));
	CHECK_MSTATUS(fnEnumAttr.addField("yxz", 4));
	CHECK_MSTATUS(fnEnumAttr.addField("zyx", 5));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".constraintJointOrientX" attribute
	//
	PointOnCurveConstraint::constraintJointOrientX = fnUnitAttr.create("constraintJointOrientX", "cjox", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".constraintJointOrientY" attribute
	//
	PointOnCurveConstraint::constraintJointOrientY = fnUnitAttr.create("constraintJointOrientY", "cjoy", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".constraintJointOrientZ" attribute
	//
	PointOnCurveConstraint::constraintJointOrientZ = fnUnitAttr.create("constraintJointOrientZ", "cjoz", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".constraintJointOrient" attribute
	//
	PointOnCurveConstraint::constraintJointOrient = fnNumericAttr.create("constraintJointOrient", "cjo", PointOnCurveConstraint::constraintJointOrientX, PointOnCurveConstraint::constraintJointOrientY, PointOnCurveConstraint::constraintJointOrientZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// ".constraintParentInverseMatrix" attribute
	//
	PointOnCurveConstraint::constraintParentInverseMatrix = fnMatrixAttr.create("constraintParentInverseMatrix", "cpim", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(PointOnCurveConstraint::inputCategory));

	// Target Attributes:
	// ".targetWeight" attribute
	//
	PointOnCurveConstraint::targetWeight = fnNumericAttr.create("targetWeight", "tw", MFnNumericData::kFloat, 0.0f, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0f));
	CHECK_MSTATUS(fnNumericAttr.setMax(1.0f));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::targetCategory));

	// ".targetParameter" attribute
	//
	PointOnCurveConstraint::targetParameter = fnNumericAttr.create("targetParameter", "tp", MFnNumericData::kDouble, 0.0f, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::targetCategory));

	// ".targetUseFraction" attribute
	//
	PointOnCurveConstraint::targetUseFraction = fnNumericAttr.create("targetUseFraction", "tuf", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::targetCategory));
	
	// ".targetLoop" attribute
	//
	PointOnCurveConstraint::targetLoop = fnNumericAttr.create("targetLoop", "tl", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::targetCategory));

	// ".targetCurve" attribute
	//
	PointOnCurveConstraint::targetCurve = fnTypedAttr.create("targetCurve", "tc", MFnData::kNurbsCurve, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnTypedAttr.addToCategory(PointOnCurveConstraint::targetCategory));

	// ".target" attribute
	//
	PointOnCurveConstraint::target = fnCompoundAttr.create("target", "target", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnCompoundAttr.addChild(PointOnCurveConstraint::targetWeight));
	CHECK_MSTATUS(fnCompoundAttr.addChild(PointOnCurveConstraint::targetParameter));
	CHECK_MSTATUS(fnCompoundAttr.addChild(PointOnCurveConstraint::targetUseFraction));
	CHECK_MSTATUS(fnCompoundAttr.addChild(PointOnCurveConstraint::targetLoop));
	CHECK_MSTATUS(fnCompoundAttr.addChild(PointOnCurveConstraint::targetCurve));
	CHECK_MSTATUS(fnCompoundAttr.setArray(true));
	CHECK_MSTATUS(fnCompoundAttr.addToCategory(PointOnCurveConstraint::inputCategory));
	CHECK_MSTATUS(fnCompoundAttr.addToCategory(PointOnCurveConstraint::targetCategory));

	// Output attributes:
	// ".constraintTranslateX" attribute
	//
	PointOnCurveConstraint::constraintTranslateX = fnUnitAttr.create("constraintTranslateX", "ctx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintTranslateY" attribute
	//
	PointOnCurveConstraint::constraintTranslateY = fnUnitAttr.create("constraintTranslateY", "cty", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintTranslateZ" attribute
	//
	PointOnCurveConstraint::constraintTranslateZ = fnUnitAttr.create("constraintTranslateZ", "ctz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintTranslate" attribute
	//
	PointOnCurveConstraint::constraintTranslate = fnNumericAttr.create("constraintTranslate", "ct", PointOnCurveConstraint::constraintTranslateX, PointOnCurveConstraint::constraintTranslateY, PointOnCurveConstraint::constraintTranslateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintRotateX" attribute
	//
	PointOnCurveConstraint::constraintRotateX = fnUnitAttr.create("constraintRotateX", "crx", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintRotateY" attribute
	//
	PointOnCurveConstraint::constraintRotateY = fnUnitAttr.create("constraintRotateY", "cry", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintRotateZ" attribute
	//
	PointOnCurveConstraint::constraintRotateZ = fnUnitAttr.create("constraintRotateZ", "crz", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintRotate" attribute
	//
	PointOnCurveConstraint::constraintRotate = fnNumericAttr.create("constraintRotate", "cr", PointOnCurveConstraint::constraintRotateX, PointOnCurveConstraint::constraintRotateY, PointOnCurveConstraint::constraintRotateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintMatrix" attribute
	//
	PointOnCurveConstraint::constraintMatrix = fnMatrixAttr.create("constraintMatrix", "cm", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));
	CHECK_MSTATUS(fnMatrixAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintInverseMatrix" attribute
	//
	PointOnCurveConstraint::constraintInverseMatrix = fnMatrixAttr.create("constraintInverseMatrix", "cim", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));
	CHECK_MSTATUS(fnMatrixAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintWorldMatrix" attribute
	//
	PointOnCurveConstraint::constraintWorldMatrix = fnMatrixAttr.create("constraintWorldMatrix", "cwm", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));
	CHECK_MSTATUS(fnMatrixAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// ".constraintWorldInverseMatrix" attribute
	//
	PointOnCurveConstraint::constraintWorldInverseMatrix = fnMatrixAttr.create("constraintWorldInverseMatrix", "cwim", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));
	CHECK_MSTATUS(fnMatrixAttr.addToCategory(PointOnCurveConstraint::outputCategory));

	// Add attributes
	//
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::forwardVector));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::upVector));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::worldUpType));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::worldUpVector));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::worldUpMatrix));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::twist));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::restTranslate));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::restRotate));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::offsetTranslate));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::offsetRotate));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::target));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::constraintTranslate));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::constraintRotateOrder));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::constraintRotate));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::constraintJointOrient));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::constraintMatrix));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::constraintInverseMatrix));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::constraintWorldMatrix));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::constraintWorldInverseMatrix));
	CHECK_MSTATUS(PointOnCurveConstraint::addAttribute(PointOnCurveConstraint::constraintParentInverseMatrix));

	// Define target attribute relationships
	//
	status = fnCompoundAttr.setObject(PointOnCurveConstraint::target);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	unsigned int numChildren = fnCompoundAttr.numChildren();

	for (unsigned int i = 0; i < numChildren; i++) {

		MObject child = fnCompoundAttr.child(i);

		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintTranslate));
		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintTranslateX));
		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintTranslateY));
		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintTranslateZ));

		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintRotate));
		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintRotateX));
		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintRotateY));
		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintRotateZ));

		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintMatrix));
		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintInverseMatrix));
		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintWorldMatrix));
		CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(child, PointOnCurveConstraint::constraintWorldInverseMatrix));

	}

	// Define rest attribute relationships
	//
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::restTranslate, PointOnCurveConstraint::constraintTranslate));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::restTranslateX, PointOnCurveConstraint::constraintTranslateX));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::restTranslateY, PointOnCurveConstraint::constraintTranslateY));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::restTranslateZ, PointOnCurveConstraint::constraintTranslateZ));

	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::restRotate, PointOnCurveConstraint::constraintRotate));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::restRotateX, PointOnCurveConstraint::constraintRotateX));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::restRotateY, PointOnCurveConstraint::constraintRotateY));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::restRotateZ, PointOnCurveConstraint::constraintRotateZ));

	// Define constraint attribute relationships
	//
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintRotateOrder, PointOnCurveConstraint::constraintRotate));

	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintJointOrient, PointOnCurveConstraint::constraintRotate));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintJointOrientX, PointOnCurveConstraint::constraintRotateX));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintJointOrientY, PointOnCurveConstraint::constraintRotateY));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintJointOrientZ, PointOnCurveConstraint::constraintRotateZ));

	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintTranslate));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintTranslateX));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintTranslateY));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintTranslateZ));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintRotate));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintRotateX));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintRotateY));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintRotateZ));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintMatrix));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintInverseMatrix));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintWorldMatrix));
	CHECK_MSTATUS(PointOnCurveConstraint::attributeAffects(PointOnCurveConstraint::constraintParentInverseMatrix, PointOnCurveConstraint::constraintWorldInverseMatrix));

	return status;

};