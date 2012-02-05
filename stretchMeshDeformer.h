#ifndef __stretchMeshDEFORMER_H__
#define __stretchMeshDEFORMER_H__
#pragma once

//  File: stretchMeshDeformer.h
//
//  Description:
// 	A deformer which defines a local coordinate space for each vertex based on the neighboring 
//	vertices.  The deformer positions each vertex based on its local coordinates on each iteration.
//	The result is a deformer that tries to maintain the local characteristics of a surface.  Upstream
//	deformations are "seen" by the stretchMesh resulting in a propagation of local deformations.
//

#define MAYA_stretchMeshDEFORMER_NAME			"stretchMesh"

class MFnPlugin;

#include "stretchMeshConsts.h"
#include <maya/MIOStream.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MNodeMessage.h>
#include <maya/MPlugArray.h>
#include <maya/MFnDoubleArrayData.h>

#define McheckErr(stat,msg)		\
	if ( MS::kSuccess != stat ) {	\
		cerr << msg;				\
		return MS::kFailure;		\
	}

class stretchMeshDeformer : public MPxDeformerNode
{
public:
						stretchMeshDeformer();
	virtual				~stretchMeshDeformer();

	static bool			Licensed;
	static bool			Registered;
	static MStatus		Register(MFnPlugin& ioPlugin, bool pLicensed );
	static MStatus		Deregister(MFnPlugin& ioPlugin);

	static  void*		creator();
	static  MStatus		initialize();
	
	virtual void		postConstructor();
	virtual MStatus		connectionMade(const MPlug& plug, const MPlug& otherPlug, bool asSrc);

	// deformation function
	//
    virtual MStatus   	deform(MDataBlock& 		block,
							   MItGeometry& 	iter,
							   const MMatrix& 	mat,
							   unsigned int 	multiIndex);

public:
	// stretchMeshDeformer attributes
	//
	static MObject		stretchMeshVersion;
	static MObject		collisionStep;  // Optimization: defines how frequently collisions are performed.
								// a value of 3 means perform collisions on every third iteration.
								// collisions are always performed on the last iteration.
	static MObject		iterations;
	static MObject		collisions;		// whether or not to evaluate collision objects
	static MObject		meanWeightsList;
	static MObject		meanWeights;
	static MObject		connVrtIdList;	
	static MObject		connVrtId;
	static MObject		connVrtIdNrmlOrderList;	
	static MObject		connVrtIdNrmlOrder;
	static MObject		enableScaleSafe;
	static MObject		b;
	static MObject		bScalableList;
	static MObject		bScalable;
	static MObject		stiffnessList;
	static MObject		stiffness;
	static MObject		attrWorldMatrixList;
	static MObject		attrctrStrengthList;
	static MObject		attrctrStrength;
	static MObject		attrctrVrtMultList;
	static MObject		attrctrVrtMult;
	static MObject		attrPaintWeights;	// Temporarily stores the weights for one of the influences 
	static MObject		attrPaintTrans;		// Identifies the influence whose weights are used in the paintWeights attribute 
	static MObject		attrPaintArrDirty;	// Gets marked dirty when the values in paintWeights change 
	// curve attractors
	static MObject		crvAttractorCurve;
	static MObject		crvAttractorStrength;
	static MObject		crvAttractorVrtMultList;
	static MObject		crvAttractorVrtMult;
	static MObject		crvAttractorAttachUVList;
	static MObject		crvAttractorAttachUV;	
	// end curve attractors
	static MObject		mshCollider;
	static MObject		mshColliderPad;
	static MObject		mshColliderInflated;
	static MObject		mshColliderMult;
	static MObject		mshColliderVrtMultList;
	static MObject		mshColliderVrtMult;
	static MObject		nrbsCollider;
	static MObject		primSphrColliderList;
	static MObject		primSphrColliderMult;
	static MObject		primSphrColliderVrtMultList;
	static MObject		primSphrColliderVrtMult;
	static MObject		primCrvColliderList;
	static MObject		primCrvColliderMult;
	static MObject		crvColliderVrtMultList;
	static MObject		crvColliderVrtMult;
	static MObject		crvColliderRadiusList;
	static MObject		crvColliderRadius;
	static MTypeId	id;

private:
	// Methods
	static MVector project_vrt_to_plane(MVector vrt, MVector normal, double d);
	
protected:
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//  Methods below are necessary to support painting of per-vertex attractor multiplier //
	static void	 sAttrChangedCallback(MNodeMessage::AttributeMessage inMsg, MPlug& ioPlug, MPlug& ioOtherPlug, void* inUserPtr);
	void AttrChangedCallback(MNodeMessage::AttributeMessage inMsg, MPlug& ioPlug, MPlug& ioOtherPlug);
	// influenceType == 1 for a point attractor, influenceType == 2 for a curve attractor
	MStatus FindInfluenceObjectIndex(const MObject& inInfluenceObjectToFind, uint& outIndex, uint& influenceType) const;
	// influenceType == 1 for a point attractor, influenceType == 2 for a curve attractor
	void GetInfluenceObjectWeights(uint inInfluenceObjectIndex, uint influenceType, MDoubleArray& outWeights) const;
	void SetInfluenceObjectWeights(uint inInfluenceObjectIndex, const MDoubleArray& inWeights, uint influenceType) const;
	void GetPaintWeights(MDoubleArray& outWeights) const;
	void RemoveCallbacks();
	
	MCallbackId mWeightListCallbackId;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

};

#endif