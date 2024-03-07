#ifndef _POINT_ON_CURVE_CONSTRAINT
#define _POINT_ON_CURVE_CONSTRAINT
//
// File: PathConstraint.h
//
// Dependency Graph Node: pointOnCurveConstraint
//
// Author: Benjamin H. Singleton
//

#include <maya/MPxConstraint.h>
#include <maya/MObject.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MPlug.h>
#include <maya/MDistance.h>
#include <maya/MAngle.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MFloatArray.h>
#include <maya/MVector.h>
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <maya/MMatrixArray.h>
#include <maya/MString.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MTypeId.h>
#include <maya/MGlobal.h>


enum class WorldUpType
{

	SceneUp = 0,
	ObjectUp = 1,
	ObjectRotationUp = 2,
	Vector = 3,
	CurveNormal = 4

};


struct WorldUpSettings
{

	WorldUpType worldUpType;
	MVector worldUpVector;
	MMatrix worldUpMatrix;

};


struct AxisSettings
{

	MVector forwardAxis;
	MVector upAxis;
	MAngle twistAngle;
	WorldUpSettings worldUpSettings;

};


class PointOnCurveConstraint : public MPxConstraint
{

public:

							PointOnCurveConstraint();
	virtual					~PointOnCurveConstraint();

	virtual MStatus			compute(const MPlug& plug, MDataBlock& data);

	static  void*			creator();
	static  MStatus			initialize();

	const	MObject			targetAttribute() const override;
	const	MObject			weightAttribute() const override;
	const	MObject			constraintRotateOrderAttribute() const override;

	static	MStatus			createMatrixFromCurve(const MObject& curve, const double parameter, const AxisSettings& settings, MMatrix& matrix);
	static	double			clampCurveParameter(const MObject& curve, const double parameter, MStatus* status);
	static	MStatus			getForwardVector(const MObject& curve, const double parameter, MVector& forwardVector);
	static	MStatus			getUpVector(const MObject& curve, const double parameter, const WorldUpSettings& settings, const MVector& origin, MVector& upVector);
	static	MVector			getObjectRotationUpVector(const MVector& worldUpVector, const MMatrix& worldUpMatrix);
	static	MStatus			getCurvePoint(const MObject& curve, const double parameter, MPoint& point);
	static	MStatus			getCurveNormal(const MObject& curve, const double parameter, MVector& upVector);

	static	MMatrix			composeMatrix(const MVector& xAxis, const MVector& yAxis, const MVector& zAxis, const MPoint& position);
	static	MStatus			decomposeMatrix(const MMatrix& matrix, MVector& position, MQuaternion& rotation, MVector& scale);
	static	MVector			getTranslationPart(const MMatrix& matrix);
	static	MQuaternion		getRotationPart(const MMatrix& matrix);
	static	MVector			getScalePart(const MMatrix& matrix);

	static	MMatrix			createPositionMatrix(const MPoint& position);
	static	MMatrix			createPositionMatrix(const MVector& position);
	static	MMatrix			createPositionMatrix(const MMatrix& matrix);

	static	MMatrix			createRotationMatrix(const MMatrix& matrix);

	static	MMatrix			createScaleMatrix(const MVector& scale);
	static	MMatrix			createScaleMatrix(const MMatrix& matrix);

	static	float			sum(const MFloatArray& items);
	static	MFloatArray		clamp(const MFloatArray& items);

	static	double			dot(const MQuaternion& startQuat, const MQuaternion& endQuat);
	static	MQuaternion		slerp(const MQuaternion& startQuat, const MQuaternion& endQuat, const float weight);

	static	MMatrix			blendMatrices(const MMatrix& restMatrix, const MMatrixArray& matrices, const MFloatArray& weights);
	static	MMatrix			blendMatrices(const MMatrix& startMatrix, const MMatrix& endMatrix, const float weight);

public:

	static	MObject			forwardVector;
	static	MObject			forwardVectorX;
	static	MObject			forwardVectorY;
	static	MObject			forwardVectorZ;
	static  MObject			upVector;
	static  MObject			upVectorX;
	static  MObject			upVectorY;
	static  MObject			upVectorZ;
	static  MObject			worldUpType;
	static  MObject			worldUpVector;
	static  MObject			worldUpVectorX;
	static  MObject			worldUpVectorY;
	static  MObject			worldUpVectorZ;
	static  MObject			worldUpMatrix;
	static	MObject			twist;
	static	MObject			offsetTranslate;
	static	MObject			offsetTranslateX;
	static	MObject			offsetTranslateY;
	static	MObject			offsetTranslateZ;
	static	MObject			offsetRotate;
	static	MObject			offsetRotateX;
	static	MObject			offsetRotateY;
	static	MObject			offsetRotateZ;
	static	MObject			restTranslate;
	static	MObject			restTranslateX;
	static	MObject			restTranslateY;
	static	MObject			restTranslateZ;
	static	MObject			restRotate;
	static	MObject			restRotateX;
	static	MObject			restRotateY;
	static	MObject			restRotateZ;

	static  MObject			target;
	static  MObject			targetWeight;
	static  MObject			targetParameter;
	static  MObject			targetUseFraction;
	static  MObject			targetLoop;
	static  MObject			targetCurve;

	static	MObject			constraintTranslate;
	static	MObject			constraintTranslateX;
	static	MObject			constraintTranslateY;
	static	MObject			constraintTranslateZ;
	static	MObject			constraintRotateOrder;
	static	MObject			constraintRotate;
	static	MObject			constraintRotateX;
	static	MObject			constraintRotateY;
	static	MObject			constraintRotateZ;
	static	MObject			constraintJointOrient;
	static	MObject			constraintJointOrientX;
	static	MObject			constraintJointOrientY;
	static	MObject			constraintJointOrientZ;
	static	MObject			constraintMatrix;
	static	MObject			constraintInverseMatrix;
	static	MObject			constraintWorldMatrix;
	static	MObject			constraintWorldInverseMatrix;
	static	MObject			constraintParentInverseMatrix;

public:

	static	MString			inputCategory;
	static	MString			offsetCategory;
	static	MString			restCategory;
	static	MString			targetCategory;
	static	MString			outputCategory;

	static	MString			classification;

	static	MTypeId			id;

};
#endif