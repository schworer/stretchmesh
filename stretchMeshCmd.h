#ifndef __stretchMeshCMD_H__
#define __stretchMeshCMD_H__
#pragma once

//  File: stretchMeshCmd.h
//
//  Description:
// 		This defines a command that will create a stretch mesh node, and initialize
//		all of the relevant attributes for that node (pyramid coordinates etc.).  The 
//		deformer itself is defined in another project (stretchMesh.cpp)
//

#define MAYA_stretchMeshCMD_NAME			"stretchMesh"

class MFnPlugin;

#include "stretchMeshConsts.h"
#include <maya/MIOStream.h>
#include <maya/MPxCommand.h>
#include <maya/MVector.h>
#include <maya/MSelectionList.h>
#include <maya/MPointArray.h>
#include <maya/MDagPath.h>
#include <maya/MStringArray.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MDGModifier.h>
#include <maya/MFloatArray.h>
#include <maya/MItMeshVertex.h>

#define kCollisionStepFlag			"-cs"
#define kCollisionStepFlagLong		"-collisionStep"
#define kAddColliderFlag				"-ac"
#define kAddColliderFlagLong			"-addCollider"
#define kAddCurveColliderFlag		"-acc"
#define kAddCurveColliderFlagLong	"-addCurveCollider"
#define kAddSphereColliderFlag		"-asc"
#define kAddSphereColliderFlagLong	"-addSphereCollider"
#define kAddAttractorFlag			"-aa"
#define kAddAttractorFlagLong		"-addAttractor"
#define kAddCurveAttractorFlag		"-aca"
#define kAddCurveAttractorFlagLong	"-addCurveAttractor"
#define kIterationFlag				"-i"
#define kIterationFlagLong			"-iterations"
#define kStiffnessFlag				"-s"
#define kStiffnessFlagLong			"-stiffness"
#define kCollisionFlag				"-c"
#define kCollisionFlagLong			"-collisions"
#define kScaleSafeFlag				"-ess"
#define kScaleSafeFlagLong			"-enableScaleSafe"
#define kExtendConnectedVertsFlag	"-ecv"
#define kExtendConnectedVertsFlagLong	"-enableExtendConnectedVerts"

class stretchMeshCmd : public MPxCommand
{
public:
					stretchMeshCmd() {};
	virtual			~stretchMeshCmd(); 

	static bool		Registered;
	static MStatus	Register(MFnPlugin& ioPlugin);
	static MStatus	Deregister(MFnPlugin& ioPlugin);

	// MPxCommand interface
	MStatus			doIt( const MArgList& args );
	MStatus			parseArgs(const MArgList& args);
	MStatus			redoIt( );
	MStatus			undoIt( );
	bool				isUndoable( ) const;
	static MSyntax		newSyntax();
	static void*		creator();
	
	static MStatus buildstretchMeshCmdMenu();

private:
	// This is data that is necessary to redo/undo/edit the command.
	// In edit mode, redoit() needs to know which flags were set on the command line 
	// so it only sets those.  The <attribute>FlagSet booleans store this information. 
	bool isEditMode;
	MSelectionList selected;
	int collisionStepFlag;
	bool collisionStepFlagSet;
	MString addColliderFlag;
	bool addColliderFlagSet;
	MString addCurveColliderFlag;
	bool addCurveColliderFlagSet;
	MString addSphereColliderFlag;
	bool addSphereColliderFlagSet;
	MString addAttractorFlag;
	bool addAttractorFlagSet;
	MString addCurveAttractorFlag;
	bool addCurveAttractorFlagSet;
	int iterationFlag;
	bool iterationFlagSet;
	double stiffnessFlag;
	bool stiffnessFlagSet;
	bool collisionFlag;
	bool collisionFlagSet;
	bool scaleSafeFlag;
	bool scaleSafeFlagSet;
	bool extendConnectedVertsFlag;
	bool extendConnectedVertsFlagSet;
	MFnDependencyNode deformerFnDepNode;
	MStringArray stretchMeshesCreated;
	MDGModifier dgModifier;
	
	// These attributes store the original values of the stretchMesh attributes for 
	// undo purposes
	int collisionStepOrig;
	int iterationOrig;
	bool collisionsOrig;
	bool scaleSafeOrig;
	MFloatArray stiffnessOrigArray;

	// Methods
	bool getConnectedVerts(MItMeshVertex& meshIter, MIntArray& connVerts, int currVertIndex);
	bool bubbleSort(MIntArray& vertsToRemove);
	static MVector getCurrNormal(MPointArray& inputPts, MIntArray& connVerts);
	static MVector projectVrtToPlane(MVector vrt, MVector nrml, double d);	
	bool addCollider();
	bool addCurveCollider();
	bool addSphereCollider();
	bool addAttractor();
	bool addCurveAttractor();
	
};

#endif