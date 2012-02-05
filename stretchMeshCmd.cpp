//  File: stretchMeshCmd.cpp
//
//  Description:
// 		This defines a command that will create a stretch mesh node, and initialize
//		all of the relevant attributes for that node (pyramid coordinates etc.).  The 
//		deformer itself is defined in another project (stretchMesh.cpp)
//
#define KS_DOUBLE_EQ(x,v) (((v - 0.00000001) < x) && (x <( v + 0.00000001)))

#include "stretchMeshCmd.h"

#include <math.h>
#include <vector>
#include <algorithm>

#include <maya/MItSelectionList.h>
#include <maya/MString.h>
#include <maya/MArgList.h>
#include <maya/MItMeshVertex.h>
#include <maya/MPointArray.h>

#include <maya/MFnCamera.h>
#include <maya/MGlobal.h>
#include <maya/M3dView.h>
#include <maya/MDagPath.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MCommandResult.h>
#include <maya/MFnNurbsCurve.h>

#include <maya/MSyntax.h>
#include <maya/MArgParser.h>
#include <maya/MArgDatabase.h>
#include <maya/MCursor.h>
#include <maya/MMatrix.h>

#include "../ksUtils/ksUtils.h"
#include "../ksUtils/IncludeMFnPluginClass.h"

using namespace std;

// Custom struct for holding the polar coordinates of connected verts.  PolarAngle and polarDistance
// represent the polar coordinates for the connected vert relative to the current vert. We are using the
// vector from the current vert to the 1st connected vert (as returned by getConnectedVertices() ) as the 
// xAxis/polarAxis of the polar coordinate system.  The wedgeAngle represents the angle between the 
// two adjacent connected verts
struct polarCoords {
	int vertID;
	double polarAngle;
	double polarDistance;
	double wedgeAngle;
};
// Function for sorting an array of objects of type <polarCoords>
bool polarCoordsCompare (polarCoords i, polarCoords j) { 
	return (i.polarAngle < j.polarAngle);
}

bool		stretchMeshCmd::Registered = false;

stretchMeshCmd::~stretchMeshCmd() {}

void* stretchMeshCmd::creator()
{
	return new stretchMeshCmd();
}

MSyntax stretchMeshCmd::newSyntax()
{
	MSyntax syntax;

	syntax.enableEdit(true);
	syntax.addFlag(kCollisionStepFlag, kCollisionStepFlagLong, MSyntax::kDouble);
	syntax.addFlag(kIterationFlag, kIterationFlagLong, MSyntax::kDouble);
	syntax.addFlag(kStiffnessFlag, kStiffnessFlagLong, MSyntax::kDouble);
	syntax.addFlag(kCollisionFlag, kCollisionFlagLong, MSyntax::kBoolean);
	syntax.addFlag(kAddColliderFlag, kAddColliderFlagLong, MSyntax::kString);
	syntax.addFlag(kAddCurveColliderFlag, kAddCurveColliderFlagLong, MSyntax::kString);
	syntax.addFlag(kAddSphereColliderFlag, kAddSphereColliderFlagLong, MSyntax::kString);
	syntax.addFlag(kAddAttractorFlag, kAddAttractorFlagLong, MSyntax::kString);
	syntax.addFlag(kAddCurveAttractorFlag, kAddCurveAttractorFlagLong, MSyntax::kString);
	syntax.addFlag(kScaleSafeFlag, kScaleSafeFlagLong, MSyntax::kBoolean);
	syntax.addFlag(kExtendConnectedVertsFlag, kExtendConnectedVertsFlagLong, MSyntax::kBoolean);
	syntax.useSelectionAsDefault( true );
	syntax.setObjectType( MSyntax::kSelectionList );
	
	return syntax;
}

MStatus stretchMeshCmd::doIt(  const MArgList& args )
//
// Description
//     Gets the currently selected object, and stores it in the local class data.  
//     Then calls redoit to actually execute the command. 
//
// Note
//     The doit method should collect whatever information is
//     required to do the task, and store it in local class data.
//     It should finally call redoIt to make the command happen.
//
{
	MStatus status;

	status = parseArgs(args);
	if (MS::kSuccess != status)
		return status;

	return redoIt();
}

MStatus stretchMeshCmd::parseArgs(const MArgList &args)
{

	MStatus status;
	MArgDatabase argData(syntax(), args);
	MSelectionList argObjects;

	
	// Initialize the <attribute>FlagSet variables to false so we can tell later on if they 
	// were set from the command line. 
	collisionStepFlagSet = false;
	iterationFlagSet = false;
	collisionFlagSet = false;
	addColliderFlagSet = false;
	addAttractorFlagSet = false;
	addCurveAttractorFlagSet = false;
	stiffnessFlagSet = false;
	isEditMode = false;
	addCurveColliderFlagSet = false;
	addSphereColliderFlagSet = false;
	scaleSafeFlagSet = false;
	extendConnectedVertsFlagSet = false;
	

	if (argData.isEdit()) { isEditMode = true; }
		
	if (argData.isFlagSet(kCollisionStepFlag)) {
		int tmp;
		status = argData.getFlagArgument(kCollisionStepFlag, 0, tmp);
		if (!status) {
			status.perror("collision step flag parsing failed");
			return status;
		}
		collisionStepFlag = tmp;
		collisionStepFlagSet = true;
	}else{ collisionStepFlag = COLLISION_STEP_DFLT; }

	if (argData.isFlagSet(kIterationFlag)) {
		int tmp;
		status = argData.getFlagArgument(kIterationFlag, 0, tmp);
		if (!status) {
			status.perror("iteration flag parsing failed");
			return status;
		}
		iterationFlag = tmp;
		iterationFlagSet = true;
	}else{ iterationFlag = ITERATIONS_DFLT; }

	if (argData.isFlagSet(kStiffnessFlag)) {
		double tmp;
		status = argData.getFlagArgument(kStiffnessFlag, 0, tmp);
		if (!status) {
			status.perror("stiffness flag parsing failed");
			return status;
		}
		stiffnessFlag = tmp;
		stiffnessFlagSet = true;
	}else{ stiffnessFlag = STIFFNESS_DFLT; }
		
	if (argData.isFlagSet(kCollisionFlag)) {
		bool tmp;
		status = argData.getFlagArgument(kCollisionFlag, 0, tmp);
		if (!status) {
			status.perror("collision flag parsing failed");
			return status;
		}
		collisionFlag = tmp;
		collisionFlagSet = true;
	}else{ collisionFlag = COLLISION_DFLT; }
	
	if (argData.isFlagSet(kAddColliderFlag)) {
		MString tmp;
		status = argData.getFlagArgument(kAddColliderFlag, 0, tmp);
		if (!status) {
			status.perror("Add collider flag parsing failed");
			return status;
		}
		addColliderFlag = tmp;
		addColliderFlagSet = true;
	}
	
	if (argData.isFlagSet(kAddCurveColliderFlag)) {
		MString tmp;
		status = argData.getFlagArgument(kAddCurveColliderFlag, 0, tmp);
		if (!status) {
			status.perror("Add curve collider flag parsing failed");
			return status;
		}
		addCurveColliderFlag = tmp;
		addCurveColliderFlagSet = true;
	}
	
	if (argData.isFlagSet(kAddSphereColliderFlag)) {
		MString tmp;
		status = argData.getFlagArgument(kAddSphereColliderFlag, 0, tmp);
		if (!status) {
			status.perror("Add sphere collider flag parsing failed");
			return status;
		}
		addSphereColliderFlag = tmp;
		addSphereColliderFlagSet = true;
	}
	
	if (argData.isFlagSet(kAddAttractorFlag)) {
		MString tmp;
		status = argData.getFlagArgument(kAddAttractorFlag, 0, tmp);
		if (!status) {
			status.perror("Add attractor flag parsing failed");
			return status;
		}
		addAttractorFlag = tmp;
		addAttractorFlagSet = true;
	}

	if (argData.isFlagSet(kAddCurveAttractorFlag)) {
		MString tmp;
		status = argData.getFlagArgument(kAddCurveAttractorFlag, 0, tmp);
		if (!status) {
			status.perror("Add attractor flag parsing failed");
			return status;
		}
		addCurveAttractorFlag = tmp;
		addCurveAttractorFlagSet = true;
	}
	
	if (argData.isFlagSet(kScaleSafeFlag)) {
		bool tmp;
		status = argData.getFlagArgument(kScaleSafeFlag, 0, tmp);
		if (!status) {
			status.perror("scale safe flag parsing failed");
			return status;
		}
		scaleSafeFlag = tmp;
		scaleSafeFlagSet = true;
	}else{ scaleSafeFlag = SCALE_SAFE_DFLT; }
	
	if (argData.isFlagSet(kExtendConnectedVertsFlag)) {
		bool tmp;
		status = argData.getFlagArgument(kExtendConnectedVertsFlag, 0, tmp);
		if (!status) {
			status.perror("Extend connected vert flag parsing failed");
			return status;
		}
		extendConnectedVertsFlag = tmp;
		extendConnectedVertsFlagSet = true;
	}else{ extendConnectedVertsFlag = EXTEND_CONN_VRTS_DFLT; }
	
	argData.getObjects(argObjects);
	if(argObjects.length() == 0){
		MGlobal::displayError( "No deformable objects selected." );
	}
	// if there were objects passed in on the command line, use those.  Else use the selected objects
	selected = argObjects;

	return MS::kSuccess;
}	

MStatus stretchMeshCmd::redoIt()
//
// Description
//     Creates/modifies a StretchMesh node
//
// Note
//     The redoIt method should do the actual work, based on the
//     internal data only.
//
{
	MStatus status;
	// We only want the polygonal meshes from the selection list
	MItSelectionList iter(selected, MFn::kMesh, &status );
	if (MS::kSuccess != status) {
		cerr << "redoIt: could not create selection list iterator\n";
		return status;
	}	
	MString currSelected;
	MSelectionList selectionList;
	MPlug deformerPlug;
	unsigned int numShapes;
	int prevIndex;
	MVector nrml;
	MPoint currVrtPt;
	MVector currVrtPos;
	MPointArray inputPts;
	MPoint connVrtPt;
	MVector connVrtPos;
	MVector currVrtProj;
	MVector connVrtProj;
	MVector projected;
	double d;
	MDoubleArray mvWeights;
	MPoint conn1Pt;
	MVector conn1Pos;
	MVector conn1Proj;
	MVector vec1;
	MPoint conn2Pt;
	MVector conn2Pos;
	MVector conn2Proj;
	MVector vec2;
	MVector vec;
	MStringArray melResult;
	MIntArray connVerts;
	MIntArray connVertsReverse;
	MIntArray degenerateVertIds;
	MPlug polarCoordsPlug;
	MPlug nrmlOrderPlug;
	MPlug connVrtIdPlug;
	MPlug connVrtIdNrmlOrderPlug;
	MPlug connVrtIdPlugArray;
	MPlug connVrtIdNrmlOrderPlugArray;
	MSelectionList degenerateVertList;
	degenerateVertList.clear();
	
	clearResult();
	
	// First, we must determine if we're in edit mode
	if (isEditMode){
		if(selected.length() != 1){
			MGlobal::displayError("A single deformer should be specified in edit/query mode.");
			return MStatus::kFailure;
		}
		
		// Find the StretchMesh node from the selected object
		MObject argObj;
		MObject stretchMeshObj;
		selected.getDependNode(0, argObj);
		MFnDependencyNode argFn;
		argFn.setObject(argObj);
		
		// First, we check to see if argObj is the StretchMesh node.
		if(argFn.typeName() == "stretchMesh"){
			deformerFnDepNode.setObject(argObj);
		}else{
			// If we've gotten here, the argument to the command was not the stretchMesh, so we 
			// have to search for it in the input to this mesh's inMesh attr. 
			MDagPath argDagPath;
			selected.getDagPath(0, argDagPath);
			argDagPath.extendToShapeDirectlyBelow(0);
			argFn.setObject(argDagPath.node());
			
			MPlug inMeshPlug;
			inMeshPlug = argFn.findPlug("inMesh");
			if(!inMeshPlug.isConnected()){
				MStringArray selectionStrings;
				selected.getSelectionStrings(selectionStrings);
				MGlobal::displayError("Expected a stretchMesh deformer or a deformed shape. Found '" + selectionStrings[0] + "' instead.");
				return MStatus::kFailure;
			}
			
			// get the plug connected to the inMesh, this should be a StretchMesh, if not, print error and exit.
			MPlugArray connectedPlugs;
			MPlug connectedPlug;
			inMeshPlug.connectedTo(connectedPlugs, true, false);
			argFn.setObject(connectedPlugs[0].node());
			if(argFn.typeName() == "stretchMesh"){
				deformerFnDepNode.setObject(connectedPlugs[0].node());
			}else{
				MGlobal::displayError("Expected a stretchMesh deformer or a deformed shape.");
				return MStatus::kFailure;
			}
		}
		
		// If we've made it here, deformerFnDepNode contains the stretchMesh node.
		// Set the attributes to be edited.  Only the attributes that were specified to the 
		// command should be set, we can determine this from the <attribute>FlagSet boolean 
		// member attributes. 
		MPlug stretchMeshPlug;
			
		// get the plug for nodeState, disable the stretchMesh deformer so it's not evaluating as we go.
		stretchMeshPlug = deformerFnDepNode.findPlug("nodeState");
		int nodeState;
		stretchMeshPlug.getValue(nodeState);
		stretchMeshPlug.setValue( 1 );

		if(collisionStepFlagSet){
			// Store the attribute values before making any changes for undo purposes.
			stretchMeshPlug = deformerFnDepNode.findPlug("collisionStep");
			stretchMeshPlug.getValue(collisionStepOrig);
			stretchMeshPlug.setValue( collisionStepFlag );
		}
		
		if(iterationFlagSet){
			// Store the attribute values before making any changes for undo purposes.
			stretchMeshPlug = deformerFnDepNode.findPlug("iterations");
			stretchMeshPlug.getValue(iterationOrig);
			stretchMeshPlug.setValue( iterationFlag );
		}

		if(stiffnessFlagSet){
			// Store the attribute values before making any changes for undo purposes.
			stretchMeshPlug = deformerFnDepNode.findPlug("stiffnessList");
			stretchMeshPlug = stretchMeshPlug.elementByLogicalIndex(0);
			stretchMeshPlug = stretchMeshPlug.child(0);
			stiffnessOrigArray.clear();
			stiffnessOrigArray.setLength(stretchMeshPlug.numElements());
			for(int itr = 0; itr < stiffnessOrigArray.length(); itr++){
				stretchMeshPlug = deformerFnDepNode.findPlug("stiffnessList");
				stretchMeshPlug = stretchMeshPlug.elementByLogicalIndex(0);
				stretchMeshPlug = stretchMeshPlug.child(0);
				stretchMeshPlug = stretchMeshPlug.elementByLogicalIndex(itr);
				stretchMeshPlug.getValue(stiffnessOrigArray[itr]);
			}
			
			// set the stiffness values
			for(int itr = 0; itr < stiffnessOrigArray.length(); itr++){
				stretchMeshPlug = deformerFnDepNode.findPlug("stiffnessList");
				stretchMeshPlug = stretchMeshPlug.elementByLogicalIndex(0);
				stretchMeshPlug = stretchMeshPlug.child(0);
				stretchMeshPlug = stretchMeshPlug.elementByLogicalIndex(itr);
				stretchMeshPlug.setValue( stiffnessFlag );
			}
		}

		if(scaleSafeFlagSet){
			// Store the attribute values before making any changes for undo purposes.
			stretchMeshPlug = deformerFnDepNode.findPlug("enableScaleSafe");
			stretchMeshPlug.getValue(scaleSafeOrig);
			stretchMeshPlug.setValue( scaleSafeFlag );
		}
		
		if(addColliderFlagSet){
			addCollider();
		}

		if(addCurveColliderFlagSet){
			addCurveCollider();
		}

		if(addSphereColliderFlagSet){
			addSphereCollider();
		}
		
		if(addAttractorFlagSet){
			addAttractor();
		}

		if(addCurveAttractorFlagSet){
			addCurveAttractor();
		}
		
		// turn nodeState back to what is was before we started
		stretchMeshPlug = deformerFnDepNode.findPlug("nodeState");
		stretchMeshPlug.setValue( nodeState );
		melResult.append(deformerFnDepNode.name());
		
		/////////////////////////////////////////////////////////////////////////////
		//						END EDIT MODE                                ////////
		/////////////////////////////////////////////////////////////////////////////
	}else{
		// We're not in edit mode, so just iterate through the selection list and apply a 
		// stretchMesh to any selected poly mesh
		
		for(iter.reset(); !iter.isDone(); iter.next() )
		{
			MDagPath selectedItem;
			iter.getDagPath(selectedItem);
			
			if(selectedItem.apiType() != MFn::kMesh){
				MGlobal::displayError("StretchMesh must be applied to a polygonal mesh");
				continue;
			}
			// First off... create the stretchMesh deformer
			MStringArray deformerResult;
			MGlobal::executeCommand( "deformer -type stretchMesh " + selectedItem.fullPathName(), deformerResult );
			melResult.append(deformerResult[0]);
			// Add the deformer string to the member list so it can be removed during undo.
			stretchMeshesCreated.append(deformerResult[0]);
			selectionList.clear();
			selectionList.add( deformerResult[0] );
			MObject deformerObj;
			selectionList.getDependNode(0, deformerObj);
			deformerFnDepNode.setObject(deformerObj);
			
			// set the version of stretchMesh that was used to create this node
			deformerPlug = deformerFnDepNode.findPlug("stretchMeshVersion");
			deformerPlug.setValue( SM_VERSION );

			// get the plug for nodeState, disable the stretchMesh deformer so it's not evaluating as we go.
			deformerPlug = deformerFnDepNode.findPlug("nodeState");
			deformerPlug.setValue( 1 );
			
			// Populate an array with the input point positions 
			MItMeshVertex vertIter( selectedItem );
			int vertCount = vertIter.count();
			inputPts.clear();
			inputPts.setLength(vertCount);
			for(vertIter.reset(); !vertIter.isDone(); vertIter.next() ){
				inputPts[vertIter.index()] = vertIter.position( MSpace::kWorld );
			}
			
			connVrtIdPlugArray = deformerFnDepNode.findPlug("connVrtIdList");
			connVrtIdPlugArray.setNumElements(vertCount);
			connVrtIdNrmlOrderPlugArray = deformerFnDepNode.findPlug("connVrtIdNrmlOrderList");
			connVrtIdNrmlOrderPlugArray.setNumElements(vertCount);
			
			// iterate through each vertex, and set the corresponding pyramid coords on the deformer
			for(int vertId = 0; vertId < vertCount; vertId++){
				MItMeshVertex pyramidCoordsIter( selectedItem );
				pyramidCoordsIter.setIndex(vertId, prevIndex);
				connVerts.clear();
				
				if(extendConnectedVertsFlag){
					getConnectedVerts(pyramidCoordsIter, connVerts, vertId);
				}else{
					pyramidCoordsIter.getConnectedVertices( connVerts );
				}
//					// The paper calls for the connected verts to be in counter clockwise order, but getConnectedVertices returns
//					// them in clockwise order.  I'm reversing them here to be consistent with the paper...
//					int connVertsItr = connVertsReverse.length();
//					connVertsItr = connVertsItr - 1;
//					connVerts.clear();
//					for( ; connVertsItr >= 0; connVertsItr--){
//						connVerts.append( connVertsReverse[connVertsItr] );
//					}
				nrml = getCurrNormal(inputPts, connVerts);
				
				// Determine the d term of the projection plane equation
				currVrtPt = inputPts[vertId];
				currVrtPos.x = currVrtPt.x; currVrtPos.y = currVrtPt.y; currVrtPos.z = currVrtPt.z;
				d = 0.0;
				for(int j = 0; j < connVerts.length(); j++){
					connVrtPt = inputPts[connVerts[j]];
					connVrtPos.x = connVrtPt.x; connVrtPos.y = connVrtPt.y; connVrtPos.z = connVrtPt.z; 
					d = d + nrml*connVrtPos;
				}
				d = -(d/connVerts.length());
				
				// project the current vertex and each neighboring vertex onto the projection plane, we'll call these v' and vi' 
				// respectively
				currVrtProj = projectVrtToPlane(currVrtPos, nrml, d);
				// NOTE: I'm not sure if this is necessary... connVrtProj isn't used anywhere else in the python version of this 
				// script... which is what I'm porting from.
				connVrtProj.x = 0; connVrtProj.y = 0; connVrtProj.z = 0;

				// Here, we're converting all connected points to polar coordinates.  This is a bug fix for vertices that were degenerate.
				// Converting to polar coordinates ensures that the total of all angles between adjacent connected verts and 
				// the current vert equals 360.  First we need to define 2d cartesian coordinates based on the planed defined by 
				// "nrml".  Here, we define the x and y axes of the 2d cartesian coordinate system:
				MPoint xAxisPt = inputPts[connVerts[0]];
				MVector xAxis;
				xAxis.x = xAxisPt.x; xAxis.y = xAxisPt.y; xAxis.z = xAxisPt.z; 
				xAxis = projectVrtToPlane(xAxis, nrml, d);
				xAxis = xAxis - currVrtProj;
				xAxis.normalize();
				MVector yAxis;
				yAxis = nrml^xAxis;
				yAxis.normalize();
				// now convert all the connected verts to polar coordinates with respect to the plane defined by "nrml":
				vector<polarCoords> polarCoordsArray;					
				polarCoordsArray.clear();
				
				for(int connVrtItr = 0; connVrtItr < connVerts.length(); connVrtItr++){
					double xCoord, yCoord;
					MPoint connPt = inputPts[connVerts[connVrtItr]];
					MVector connVec;
					connVec.x = connPt.x; connVec.y = connPt.y; connVec.z = connPt.z; 
					connVec = projectVrtToPlane(connVec, nrml, d);
					connVec = connVec - currVrtProj;
					xCoord = connVec*xAxis;
					yCoord = connVec*yAxis;
					
					polarCoords currPolarCoords;
					currPolarCoords.vertID = connVerts[connVrtItr];
					currPolarCoords.polarDistance = sqrt((xCoord*xCoord) + (yCoord*yCoord));

					// Using the polar coordinate coversion descirbed on wikipedia
					if(xCoord > 0 && (yCoord > 0 || KS_DOUBLE_EQ(yCoord, 0.0))){
						currPolarCoords.polarAngle = atan(yCoord/xCoord);
					}else if(xCoord > 0 && yCoord < 0){
						currPolarCoords.polarAngle = atan(yCoord/xCoord) + (2.0 * PI);
					}else if(xCoord < 0){
						currPolarCoords.polarAngle = atan(yCoord/xCoord) + PI;
					}else if(KS_DOUBLE_EQ(xCoord, 0.0) && yCoord > 0){
						currPolarCoords.polarAngle = PI/2.0;
					}else if(KS_DOUBLE_EQ(xCoord, 0.0) && yCoord < 0){
						currPolarCoords.polarAngle = (3.0 * PI)/2.0;
					}else if(KS_DOUBLE_EQ(xCoord, 0.0) && KS_DOUBLE_EQ(yCoord, 0.0)){   // Handling the case of colocated verts
						currPolarCoords.polarAngle = 0.0;
					}
					
					// fill the polarCoords struct
					polarCoordsArray.push_back(currPolarCoords);
					
				}
				// sort the polar coords array according to the polar angle:
				sort(polarCoordsArray.begin(), polarCoordsArray.end(), polarCoordsCompare );
				
				// We have the polar angles, but now we need to calculate the wedgeAngles which actually
				// represent the angles we're interested in (see description in polarCoords struct definition above)
				for(int i = 0; i < polarCoordsArray.size(); i++){
					if(i==0){
						polarCoordsArray[i].wedgeAngle = polarCoordsArray[i].polarAngle;
					}else{
						polarCoordsArray[i].wedgeAngle = polarCoordsArray[i].polarAngle - polarCoordsArray[i-1].polarAngle;
					}
				}
				
				// The first wedgeAngle is incorrectly set to zero, fixing that here:
				polarCoordsArray[0].wedgeAngle = (2.0*PI) - polarCoordsArray.back().polarAngle;
														
				// set the attributes representing the connected verts.  
				connVrtIdPlug = connVrtIdPlugArray.elementByLogicalIndex(vertId);
				connVrtIdPlug = connVrtIdPlug.child(0);
				connVrtIdPlug.setNumElements(connVerts.length());

				connVrtIdNrmlOrderPlug = connVrtIdNrmlOrderPlugArray.elementByLogicalIndex( vertId );
				connVrtIdNrmlOrderPlug = connVrtIdNrmlOrderPlug.child(0);
				connVrtIdNrmlOrderPlug.setNumElements(connVerts.length());
				for(int j = 0; j < connVerts.length(); j++ ){
					// We have to store connected vertices in two ways (for backward compatibility), 
					// one in an order consistent with the way the normal is calculated, and one in
					// a way that is consistent with the way the polar angles are stored. Polar 
					// angle order here:
					polarCoordsPlug = connVrtIdPlug.elementByLogicalIndex(j);
					polarCoordsPlug.setValue(polarCoordsArray[j].vertID);

					// ... and normal order here:
					nrmlOrderPlug = connVrtIdNrmlOrderPlug.elementByLogicalIndex(j);
					nrmlOrderPlug.setValue(connVerts[j]);
				}

				// Determine the mean-value weights:
				//
				mvWeights.clear();
				double weightsSum = 0.0;
				for(int j = 0; j < connVerts.length(); j++){
					if(connVerts.length() > MAX_NUM_CONN_VRTS){
						MGlobal::executeCommand( "error \"stretchMesh currently only supports meshes with 32 or fewer connected vertices per vertex\"" );
						return MStatus::kFailure;
					}
					double alpha1 = polarCoordsArray[j].wedgeAngle;						
					double alpha2 = polarCoordsArray[(j+1)%connVerts.length()].wedgeAngle;
					
					weightsSum = weightsSum + (tan(alpha1/2) + tan(alpha2/2))/polarCoordsArray[j].polarDistance;
					mvWeights.append((tan(alpha1/2) + tan(alpha2/2))/polarCoordsArray[j].polarDistance);
				}
				
				int weightItr = 0;
				for(int j = 0; j < mvWeights.length(); j++){
					mvWeights[j] = mvWeights[j]/weightsSum;
					deformerPlug = deformerFnDepNode.findPlug("meanWeightsList");
					deformerPlug = deformerPlug.elementByLogicalIndex( vertId );
					deformerPlug = deformerPlug.child(0);
					deformerPlug = deformerPlug.elementByLogicalIndex( j );
					deformerPlug.setValue( mvWeights[j] );
					weightItr = weightItr + 1;
				}
				
				// Determine the normal component of the pyramid coords (the "b" term in the paper)
				vec = currVrtPos - currVrtProj;
				double b = vec.length();
				if ( vec*nrml < 0 ){
					b = -b;
				}
									
				deformerPlug = deformerFnDepNode.findPlug("b");
				deformerPlug = deformerPlug.elementByLogicalIndex(vertId);
				deformerPlug.setValue( b );
				
				// This is an alternate method for determining "b".  This method produces better
				// results when scaling the mesh. 
				for(int j = 0; j < polarCoordsArray.size(); j++){
					if(polarCoordsArray.size() > MAX_NUM_CONN_VRTS){
						MGlobal::executeCommand( "error \"stretchMesh currently only supports meshes with 32 or fewer connected vertices per vertex\"" );
						return MStatus::kFailure;
					}
					// To represent the normal component of vi with respect to the local frame, we 
					// calculate the signed cosine of the angle between each edge incident to vi and the 
					// normal 
					connVrtPt = inputPts[polarCoordsArray[j].vertID];
					MVector connPtToCurrPt;
					connPtToCurrPt = (currVrtPos - connVrtPt);
					
					double cos = (connPtToCurrPt * nrml)/connPtToCurrPt.length();
					double bScales = (cos)/(sqrt(1 - (cos*cos)));
					deformerPlug = deformerFnDepNode.findPlug("bScalableList");
					deformerPlug = deformerPlug.elementByLogicalIndex( vertId );
					deformerPlug = deformerPlug.child(0);
					deformerPlug = deformerPlug.elementByLogicalIndex( j );
					deformerPlug.setValue( bScales );
					
				}
				
				
				deformerPlug = deformerFnDepNode.findPlug("stiffnessList");
				// "stiffnessList" is a compound attribute that has children that correspond to the 
				// individual vert stiffness values.  It must be a compound attribute in order to support
				// weight painting.
				deformerPlug = deformerPlug.elementByLogicalIndex(0);
				deformerPlug = deformerPlug.child(0);
				// Now get the actual stiffness plug
				deformerPlug = deformerPlug.elementByLogicalIndex(vertId);
				deformerPlug.setValue( stiffnessFlag );
			}

			//
			// Set iterations to 1, and compare the vertex positions to input mesh points.  This will determine which verts are
			// degenerate and should have stiffness values of 1.0
			MObject degenerateVert;
			bool stiffnessWarning = false;
			deformerPlug = deformerFnDepNode.findPlug("iterations");
			deformerPlug.setValue( 1 );
			deformerPlug = deformerFnDepNode.findPlug("nodeState");
			deformerPlug.setValue( 0 );
			deformerPlug = deformerFnDepNode.findPlug("enableScaleSafe");
			deformerPlug.setValue( false );
			degenerateVertIds.clear();
			for(vertIter.reset( selectedItem ); !vertIter.isDone(); vertIter.next() ){
				int currVrtId = vertIter.index();
				if(!vertIter.position(MSpace::kWorld).isEquivalent( inputPts[currVrtId], .00001)){
					degenerateVert = vertIter.currentItem();
					degenerateVertList.add(selectedItem, degenerateVert);
					stiffnessWarning = true;
					degenerateVertIds.append(currVrtId);
				}
			}
			// iterate through one more time to find degenerate verts... this time with scale safe on
			// to find degenerate scale safe verts
			deformerPlug = deformerFnDepNode.findPlug("enableScaleSafe");
			deformerPlug.setValue( true );
			for(vertIter.reset( selectedItem ); !vertIter.isDone(); vertIter.next() ){
				int currVrtId = vertIter.index();
				if(!vertIter.position(MSpace::kWorld).isEquivalent( inputPts[currVrtId], .00001)){
					degenerateVert = vertIter.currentItem();
					degenerateVertList.add(selectedItem, degenerateVert);
					stiffnessWarning = true;
					degenerateVertIds.append(currVrtId);
				}
			}
			
			
			//Turn off the deformer again while we modify the actual stiffness values: optimization
			deformerPlug = deformerFnDepNode.findPlug("nodeState");
			deformerPlug.setValue( 1 );
			for(int degenItr = 0; degenItr < degenerateVertIds.length(); degenItr++){
				deformerPlug = deformerFnDepNode.findPlug("stiffnessList");
				deformerPlug = deformerPlug.elementByLogicalIndex(0);
				deformerPlug = deformerPlug.child(0);
				// Now get the actual stiffness plug
				deformerPlug = deformerPlug.elementByLogicalIndex(degenerateVertIds[degenItr]);
				deformerPlug.setValue( 1.0 );
			}
			
			if(stiffnessWarning){
				MGlobal::displayWarning("Warning: StretchMesh stiffness has been set to 1.0 for one or more vertices.  See documentation for details.");
				MGlobal::setActiveSelectionList(degenerateVertList);
			}
			
			// Last thing we need to do: set the iterations step to something reasonable.  It defaults to zero so the deformer 
			// isn't trying to evaluate itself during the initialization procedure above:
			deformerPlug = deformerFnDepNode.findPlug("iterations");
			deformerPlug.setValue( iterationFlag );
			
			deformerPlug = deformerFnDepNode.findPlug("collisionStep");
			deformerPlug.setValue( collisionStepFlag );
			
			deformerPlug = deformerFnDepNode.findPlug("collisions");
			deformerPlug.setValue( collisionFlag );
			
			deformerPlug = deformerFnDepNode.findPlug("enableScaleSafe");
			deformerPlug.setValue( scaleSafeFlag );

			// add collisions if there are any
			if(addColliderFlagSet){
				addCollider();
			}
			
			// add attractors
			if(addAttractorFlagSet){
				addAttractor();
			}

			deformerPlug = deformerFnDepNode.findPlug("nodeState");
			deformerPlug.setValue( 0 );
		}
	}
		
	dgModifier.doIt();
	setResult( melResult ); 
	return MS::kSuccess;
}

MStatus stretchMeshCmd::undoIt()
//
// Description
//     the undo routine
//
{
	if(isEditMode){
		MPlug stretchMeshPlug;
		// If we're in edit mode, undo anything done in edit mode
		if(collisionStepFlagSet){
			stretchMeshPlug = deformerFnDepNode.findPlug("collisionStep");
			stretchMeshPlug.setValue( collisionStepOrig );
		}
		
		if(iterationFlagSet){
			stretchMeshPlug = deformerFnDepNode.findPlug("iterations");
			stretchMeshPlug.setValue( iterationOrig );
		}
		
		if(stiffnessFlagSet){
			// set the stiffness values to original values
			for(int itr = 0; itr < stiffnessOrigArray.length(); itr++){
				stretchMeshPlug = deformerFnDepNode.findPlug("stiffnessList");
				stretchMeshPlug = stretchMeshPlug.elementByLogicalIndex(0);
				stretchMeshPlug = stretchMeshPlug.child(0);
				stretchMeshPlug = stretchMeshPlug.elementByLogicalIndex(itr);
				stretchMeshPlug.setValue( stiffnessOrigArray[itr] );
			}
		}
		
		if(collisionFlagSet){
			stretchMeshPlug = deformerFnDepNode.findPlug("collisions");
			stretchMeshPlug.setValue( collisionsOrig );
		}
		
		if(addColliderFlagSet){
			dgModifier.undoIt();
		}
		
		if(addAttractorFlagSet){
			dgModifier.undoIt();
		}

		if(scaleSafeFlagSet){
			stretchMeshPlug = deformerFnDepNode.findPlug("enableScaleSafe");
			stretchMeshPlug.setValue( scaleSafeOrig );
		}
		
	}else{
		// ...otherwise, delete any stretchMesh nodes that were created.
		for(int i = 0; i < stretchMeshesCreated.length(); i++){
			MGlobal::executeCommand("delete " + stretchMeshesCreated[i]);
		}
	}
	
	return MS::kSuccess;
}

bool stretchMeshCmd::isUndoable() const
//
// Description
//     Make the command eligable for undo.
//
{ 
	return true;
}

bool stretchMeshCmd::getConnectedVerts(MItMeshVertex& meshIter, MIntArray& connVerts, int currVertIndex)
{
	meshIter.getConnectedVertices(connVerts);
	MIntArray scndDegreeConnVerts;
	MIntArray connVertsDup = connVerts;
	// for each connected vertex, find the connected vertices and add them to the connVerts
	// array
	for(int connVertsIter = 0; connVertsIter < connVertsDup.length(); connVertsIter++){
		int prevIndex;
		meshIter.setIndex(connVertsDup[connVertsIter], prevIndex);
		meshIter.getConnectedVertices(scndDegreeConnVerts);
		
		for(int vertIter = 0; vertIter < scndDegreeConnVerts.length(); vertIter++){
//			MGlobal::displayInfo(MString("Hello!"));
			//first, check that this vert isn't already in the connVertsArray
			bool vertIdIsDuplicate = false;
			for(int i = 0; i < connVerts.length(); i++){
				if(connVerts[i] == scndDegreeConnVerts[vertIter]){
					vertIdIsDuplicate=true;
				}
			}
			
			if(scndDegreeConnVerts[vertIter] != currVertIndex && !vertIdIsDuplicate){
				connVerts.append(scndDegreeConnVerts[vertIter]);
			}
		}
	}
	
/*	// iterate through the connected verts array and only add non-duplicate verts
	connVertsDup = connVerts;
	MIntArray	connVertsResult = connVerts;
	MIntArray vertIDsToRemove;
	for(int i = 0; i < connVerts.length(); i++){
		for(int j = 0 ; j < connVerts.length(); j++){
//			MGlobal::displayInfo(MString("Greetings!"));
			if(connVerts[i] == connVerts[j]){
				vertIDsToRemove.append(i);
			}
		}
	}
	
	//bubble sort the verIDsToRemove array
	bubbleSort(vertIDsToRemove);
//	bubbleSort(connVertsResult);
	
	for(int i=0; i<vertIDsToRemove.length(); i++){
		MGlobal::displayInfo(MString("\tvert id to remove") + (vertIDsToRemove[i]-i));
		connVertsResult.remove(vertIDsToRemove[i]-i);
	}
	
	connVerts = connVertsResult;
*/	
	return true;
}

			
			
bool stretchMeshCmd::bubbleSort(MIntArray& vertsToRemove)
{
	bool swapped;
	do
	{
		swapped=false;
		for(int i=0; i < (vertsToRemove.length()-1); i++){
			if(vertsToRemove[i] > vertsToRemove[i+1]){
				int swap1 = vertsToRemove[i];
				int swap2 = vertsToRemove[i+1];
				vertsToRemove[i] = swap2;
				vertsToRemove[i+1] = swap1;
				swapped = true;
			}
		}
	}while(swapped);
	return true;
}

			
/*procedure bubbleSort( A : list of sortable items ) defined as:
do
			swapped := false
			for each i in 0 to length(A) - 2 inclusive do:
				if A[i] > A[i+1] then
					swap( A[i], A[i+1] )
					swapped := true
				end if
			end for
while swapped
end procedure
*/			
			
 
MVector stretchMeshCmd::getCurrNormal(MPointArray& inputPts, MIntArray& connVerts)
{
	// Compute the average position of all the connected verts (the "l" term in the paper)
	MVector vrtAvg(0.0, 0.0, 0.0);
	MVector crossSum(0.0, 0.0, 0.0);
	MVector vec_1;
	MVector vec_2;
	MVector cross(0.0, 0.0, 0.0);
	MVector cross_sum(0.0, 0.0, 0.0);
	MPoint pt;
	MPoint pt_1;
	MPoint pt_2;
	
	for(unsigned int i = 0; i <  connVerts.length(); i++){
		pt = inputPts[connVerts[i]];
		vrtAvg.x = vrtAvg.x + pt.x;
		vrtAvg.y = vrtAvg.y + pt.y;
		vrtAvg.z = vrtAvg.z + pt.z;
	}
	vrtAvg.x = vrtAvg.x/connVerts.length();
	vrtAvg.y = vrtAvg.y/connVerts.length();
	vrtAvg.z = vrtAvg.z/connVerts.length();
	
	// next, we need to sum the cross products between adjacent verts and the vrt_avg
	for(unsigned int i = 0; i <  connVerts.length(); i++){
		pt_1 = inputPts[connVerts[(i+1)%connVerts.length()]];
		vec_1.x = pt_1.x; vec_1.y = pt_1.y; vec_1.z = pt_1.z; 

		pt_2 = inputPts[connVerts[i]];
		vec_2.x = pt_2.x; vec_2.y = pt_2.y; vec_2.z = pt_2.z; 
		cross = vec_1^vec_2;
		cross_sum = cross_sum + cross;
	}
	
	cross_sum.normalize();
	return cross_sum;
}	

MVector stretchMeshCmd::projectVrtToPlane(MVector vrt, MVector nrml, double d)
{
	// Using the algorithm described in the paper...
	double x;
	MVector vrtProj;
	
	x = d + (vrt*nrml);
	vrtProj = x*nrml;
	vrtProj = vrt - vrtProj;
	return vrtProj;
}

bool stretchMeshCmd::addCollider()
{
	// Make sure the collider exists first:
	MCommandResult cmdResult;
	int colliderExists;
	MGlobal::executeCommand("objExists " + addColliderFlag, cmdResult);
	cmdResult.getResult(colliderExists);
	if(!colliderExists){
		MGlobal::displayError("Specified collider does not exist");
		return false;
	}

	// Get the plug representing the collider mesh output
	MSelectionList selList;
	selList.clear();
	selList.add(addColliderFlag);
	MDagPath colliderPath;
	selList.getDagPath(0, colliderPath);
	colliderPath.extendToShapeDirectlyBelow(0);
	MFnDependencyNode colliderDepNode;
	colliderDepNode.setObject(colliderPath.node());
	MPlug colliderMeshPlugArray = colliderDepNode.findPlug("worldMesh");
	// The world mesh we're interested in is the first element of the list
	MPlug colliderMeshPlug = colliderMeshPlugArray.elementByLogicalIndex(0);
	
	// get the plug representing the stretchMesh mshCollider attribute
	MPlug stretchMeshColliderPlugArray = deformerFnDepNode.findPlug("mshCollider");
	MPlug stretchMeshColliderPlug;
	// Find the first unconnected element of this array plug, use that to connect
	int itr = 0;
	bool foundElement = false;
	while(!foundElement){
		if(!stretchMeshColliderPlugArray[itr].isConnected()){
			foundElement = true;
			stretchMeshColliderPlug = stretchMeshColliderPlugArray.elementByLogicalIndex(itr);
			dgModifier.commandToExecute("setAttr " + deformerFnDepNode.name() +".mshColliderPad[" + itr + "] 0");
			continue;
		}
		itr++;
	}			
	
	// Connect the attributes 
	dgModifier.connect(colliderMeshPlug, stretchMeshColliderPlug);
	
	return true;
}

bool stretchMeshCmd::addCurveCollider()
{
	int curveColliderIndex = -1;
	MPointArray inputPts;
	MFnNurbsCurve cAttractorFn;
	MStatus status;
	int numShapes;
	MFnNurbsCurve cColliderFn;
	
	// Make sure the collider exists first:
	MCommandResult cmdResult;
	int colliderExists;
	MGlobal::executeCommand("objExists " + addCurveColliderFlag, cmdResult);
	cmdResult.getResult(colliderExists);
	if(!colliderExists){
		MGlobal::displayError("Specified collider curve does not exist");
		return false;
	}
	
	// Get the curves dependency node
	MSelectionList selList;
	selList.clear();
	selList.add(addCurveColliderFlag);
	MDagPath curveColliderPath;
	selList.getDagPath(0, curveColliderPath);
	MFnDependencyNode curveColliderDepNode;
	curveColliderDepNode.setObject(curveColliderPath.node());
	
	// Add the colliderMult attribute to the curve if it hasn't already been added
	if(!curveColliderDepNode.hasAttribute("colliderMult")){
		dgModifier.commandToExecute("addAttr -ln \"colliderMult\"  -at double  -min 0 -max 1 -dv 1 -keyable true " + addCurveColliderFlag);
		dgModifier.doIt();
	}
	
	MDagPath crvShapePath;
	MStringArray crvShapesResult;
	MGlobal::executeCommand( MString("listRelatives -pa -shapes -ni ") + curveColliderPath.fullPathName(), crvShapesResult);
	numShapes = crvShapesResult.length();
	MSelectionList shapesList;
	
	for(unsigned int i = 0; i < numShapes; i++){
		shapesList.clear();
		shapesList.add(crvShapesResult[i]);
		shapesList.getDagPath(0, crvShapePath);
		
		if(crvShapePath.apiType() != MFn::kNurbsCurve){
			MGlobal::displayError("Must supply a nurbs curve as a curve collider.");
			continue;
		}
	}
	MFnDependencyNode curveColliderShapeDepNode;
	curveColliderShapeDepNode.setObject(crvShapePath.node());
	MPlug curveColliderXformPlugArray = curveColliderShapeDepNode.findPlug("worldMatrix");
	// The world xform we're interested in is the first element of the list
	MPlug curveColliderXformPlug = curveColliderXformPlugArray.elementByLogicalIndex(0);
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////    connect curve's "worldspace[0]" to stretchMesh's first available "primCrvCollider" attr
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	MPlug colliderCurvePlugArray = curveColliderShapeDepNode.findPlug("worldSpace");
	// The world curve we're interested in is the first element of the list
	MPlug colliderCurvePlug = colliderCurvePlugArray.elementByLogicalIndex(0);
	
	// get the plug representing the stretchMesh curve and strength attribute
	MPlug stretchMeshCurveColliderPlugArray = deformerFnDepNode.findPlug("primCrvCollider");
	MPlug stretchMeshCurveColliderPlug;
	
	// Find the first unconnected element of this array plug, use that to connect
	int crvItr = 0;
	bool foundElement = false;
	while(!foundElement){
		if(!stretchMeshCurveColliderPlugArray[crvItr].isConnected()){
			foundElement = true;
			stretchMeshCurveColliderPlug = stretchMeshCurveColliderPlugArray.elementByLogicalIndex(crvItr);
			curveColliderIndex = crvItr;
			continue;
		}
		crvItr++;
	}			
	// Connect the curve attribute 
	dgModifier.connect(colliderCurvePlug, stretchMeshCurveColliderPlug);
	

	// Connect the collider mult attribute using the same collider index found above
	MPlug stretchMeshCurveColliderMultArray = deformerFnDepNode.findPlug("crvColliderMult");
	MPlug stretchMeshCurveColliderMult = stretchMeshCurveColliderMultArray.elementByLogicalIndex(curveColliderIndex);
	MPlug colliderMultPlug = curveColliderDepNode.findPlug("colliderMult");
	dgModifier.connect(colliderMultPlug, stretchMeshCurveColliderMult);	
	
	// now we need to find the first unconnected matrix plug (might not be the same index
	// as the curve attributes because there could be point attractors)
	MPlug stretchMeshMatrixPlugArray = deformerFnDepNode.findPlug("matrix");
	MPlug stretchMeshMatrixPlug;
	int itr = 0;
	foundElement = false;
	while(!foundElement){
		if(!stretchMeshMatrixPlugArray[itr].isConnected()){
			foundElement = true;
			stretchMeshMatrixPlug = stretchMeshMatrixPlugArray.elementByLogicalIndex(itr);
			continue;
		}
		itr++;
	}			
	// Connect the matrix attribute 
	dgModifier.connect(curveColliderXformPlug, stretchMeshMatrixPlug);
	
	// Create the collider locator, initialize its radius values according to the number of spans on the curve, 
	// and connect those radius attributes to the corresponding radius attributes on the stretchMesh deformer node
	MStringArray crvColliderResult;
	MGlobal::executeCommand( MString("createNode curveColliderLocator "), crvColliderResult);
	shapesList.clear();
	shapesList.add(crvColliderResult[0]);
	MDagPath crvColliderLocatorPath;
	shapesList.getDagPath(0, crvColliderLocatorPath);
	MFnDependencyNode curveColliderLocatorDepNode;
	curveColliderLocatorDepNode.setObject(crvColliderLocatorPath.node());
	MPlug locatorInputCurvePlug = curveColliderLocatorDepNode.findPlug("colliderCurve");
	dgModifier.connect(colliderCurvePlug, locatorInputCurvePlug);
	cColliderFn.setObject(curveColliderShapeDepNode.object());

	int crvColliderSpans = cColliderFn.numSpans();
	int radiusItr;
	for(radiusItr = 0; radiusItr < crvColliderSpans; radiusItr++){
		dgModifier.commandToExecute("setAttr " + curveColliderLocatorDepNode.name() +".radius[" + radiusItr + "] 1.0");
		MPlug locatorRadiusArrayPlug = curveColliderLocatorDepNode.findPlug("radius");
		MPlug locatorRadiusPlug = locatorRadiusArrayPlug.elementByLogicalIndex(radiusItr);
		MPlug smRadiusArrayPlug =  deformerFnDepNode.findPlug("crvColliderRadiusList");
		MPlug smRadiusPlug = smRadiusArrayPlug.elementByLogicalIndex(curveColliderIndex);
		smRadiusPlug = smRadiusPlug.child(0);
		smRadiusPlug = smRadiusPlug.elementByLogicalIndex(radiusItr);
		dgModifier.connect(locatorRadiusPlug, smRadiusPlug);	
	}
	
	//We need to initialize the collider per-vertex mults to one,
	MPlug smBPlug = deformerFnDepNode.findPlug("b");
	int numVerts = smBPlug.numElements();
	int vrtItr;
	for(vrtItr = 0; vrtItr < numVerts; vrtItr++){
		MPlug smColliderMultPlug = deformerFnDepNode.findPlug("crvColliderVrtMultList");
		smColliderMultPlug = smColliderMultPlug.elementByLogicalIndex(vrtItr);
		smColliderMultPlug = smColliderMultPlug.child(0);
		smColliderMultPlug = smColliderMultPlug.elementByLogicalIndex(curveColliderIndex);
		smColliderMultPlug.setValue(1.0);
	}
	
	return true;
}




bool stretchMeshCmd::addSphereCollider()
{
	int sphereColliderIndex = -1;
	// Make sure the collider exists first:
	MCommandResult cmdResult;
	int colliderExists;
	MGlobal::executeCommand("objExists " + addSphereColliderFlag, cmdResult);
	cmdResult.getResult(colliderExists);
	if(!colliderExists){
		MGlobal::displayError("Specified collider transform does not exist");
		return false;
	}
	
	// Get the sphere collider's dependency node
	MSelectionList selList;
	selList.clear();
	selList.add(addSphereColliderFlag);
	MDagPath sphereColliderPath;
	selList.getDagPath(0, sphereColliderPath);
	MFnDependencyNode sphereColliderDepNode;
	sphereColliderDepNode.setObject(sphereColliderPath.node());
	
	// Add the colliderMult attribute to the curve if it hasn't already been added
	if(!sphereColliderDepNode.hasAttribute("colliderMult")){
		dgModifier.commandToExecute("addAttr -ln \"colliderMult\"  -at double  -min 0 -max 1 -dv 1 -keyable true " + addSphereColliderFlag);
		dgModifier.doIt();
	}
	
	
	//  connect locator's worldMatrix[0] to sm node's primSphrCollider[colliderIndex]
	MPlug stretchMeshSphereColliderPlugArray = deformerFnDepNode.findPlug("primSphrCollider");
	MPlug stretchMeshSphereColliderPlug;
	// Find the first unconnected element of this array plug, use that to connect
	int sphrItr = 0;
	bool foundElement = false;
	while(!foundElement){
		if(!stretchMeshSphereColliderPlugArray[sphrItr].isConnected()){
			foundElement = true;
			stretchMeshSphereColliderPlug = stretchMeshSphereColliderPlugArray.elementByLogicalIndex(sphrItr);
			sphereColliderIndex = sphrItr;
			continue;
		}
		sphrItr++;
	}			
	MPlug sphereColliderXformPlug = sphereColliderDepNode.findPlug("worldMatrix");
	sphereColliderXformPlug = sphereColliderXformPlug.elementByLogicalIndex(0);
	dgModifier.connect(sphereColliderXformPlug, stretchMeshSphereColliderPlug);

	
	
	//  connect locator's colliderMult to sm node's primSphrColliderMult[colliderIndex]
	MPlug colliderMultPlug = sphereColliderDepNode.findPlug("colliderMult");
	MPlug smColliderMultPlug = deformerFnDepNode.findPlug("primSphrColliderMult");
	smColliderMultPlug = smColliderMultPlug.elementByLogicalIndex(sphereColliderIndex);
	dgModifier.connect(colliderMultPlug, smColliderMultPlug);
	
	
	// connect locator's worldMatrix[0] to sm node's matrix[] attribute.  Note: the matrix[] index we 
	// connect to might be different than the colliderIndex above because there might be other 
	// colliders attached.
	MPlug stretchMeshMatrixPlugArray = deformerFnDepNode.findPlug("matrix");
	MPlug stretchMeshMatrixPlug;
	int itr = 0;
	foundElement = false;
	while(!foundElement){
		if(!stretchMeshMatrixPlugArray[itr].isConnected()){
			foundElement = true;
			stretchMeshMatrixPlug = stretchMeshMatrixPlugArray.elementByLogicalIndex(itr);
			continue;
		}
		itr++;
	}			
	// Connect the matrix attribute 
	dgModifier.connect(sphereColliderXformPlug, stretchMeshMatrixPlug);
	
	
	
	//We need to initialize the collider per-vertex mults to one...
	MPlug smBPlug = deformerFnDepNode.findPlug("b");  // hack to determine the number of verts on the sm geo
	int numVerts = smBPlug.numElements();
	int vrtItr;
	for(vrtItr = 0; vrtItr < numVerts; vrtItr++){
		MPlug smColliderMultPlug = deformerFnDepNode.findPlug("primSphrColliderVrtMultList");
		smColliderMultPlug = smColliderMultPlug.elementByLogicalIndex(vrtItr);
		smColliderMultPlug = smColliderMultPlug.child(0);
		smColliderMultPlug = smColliderMultPlug.elementByLogicalIndex(sphereColliderIndex);
		smColliderMultPlug.setValue(1.0);
	}
	
	
	return true;
}



bool stretchMeshCmd::addAttractor()
{
	// Make sure the attractor exists first:
	MCommandResult cmdResult;
	int attractorExists;
	MGlobal::executeCommand("objExists " + addAttractorFlag, cmdResult);
	cmdResult.getResult(attractorExists);
	if(!attractorExists){
		MGlobal::displayError("Specified attractor does not exist");
		return false;
	}
	
	// Get the plug representing the attractor xform output
	MSelectionList selList;
	selList.clear();
	selList.add(addAttractorFlag);
	MDagPath attractorPath;
	selList.getDagPath(0, attractorPath);
	MFnDependencyNode attractorDepNode;
	attractorDepNode.setObject(attractorPath.node());
	if(!attractorDepNode.hasAttribute("worldMatrix")){
		MGlobal::displayError("Specified object does not have a world matrix attribute.  Please specify a transform node.");
		return false;
	}
	MPlug attractorLocatorPlugArray = attractorDepNode.findPlug("worldMatrix");
	// The world xform we're interested in is the first element of the list
	MPlug attractorLocatorPlug = attractorLocatorPlugArray.elementByLogicalIndex(0);
	
	// Add the attractor strength attribute to the locator if it hasn't already been added
	if(!attractorDepNode.hasAttribute("attractorStrength")){
	   dgModifier.commandToExecute("addAttr -ln \"attractorStrength\"  -at double  -min 0 -max 1 -dv 0 -keyable true " + addAttractorFlag);
	   dgModifier.doIt();
	}
	   
	// get the plug representing the stretchMesh matrix attribute
	MPlug stretchMeshAttractorPlugArray = deformerFnDepNode.findPlug("matrix");
	MPlug stretchMeshAttractorPlug;
	MPlug stretchMeshStrengthPlugArray = deformerFnDepNode.findPlug("attrctrStrength");
	MPlug stretchMeshStrengthPlug;
	// Find the first unconnected element of this array plug, use that to connect
	int itr = 0;
	bool foundElement = false;
	while(!foundElement){
		if(!stretchMeshAttractorPlugArray[itr].isConnected()){
			foundElement = true;
			stretchMeshAttractorPlug = stretchMeshAttractorPlugArray.elementByLogicalIndex(itr);
			stretchMeshStrengthPlug = stretchMeshStrengthPlugArray.elementByLogicalIndex(itr);
			continue;
		}
		itr++;
	}			
	
	// Connect the xform attribute 
	dgModifier.connect(attractorLocatorPlug, stretchMeshAttractorPlug);
	
	// Connect the attractor strength attribute
	MPlug attractorStrengthPlug = attractorDepNode.findPlug("attractorStrength");
	dgModifier.connect(attractorStrengthPlug, stretchMeshStrengthPlug);

	return true;
	
}

bool stretchMeshCmd::addCurveAttractor()
{
	MPointArray inputPts;
	MFnNurbsCurve cAttractorFn;
	MStatus status;
	int numShapes;
	
	// Make sure the attractor exists first:
	MCommandResult cmdResult;
	int attractorExists;
	MGlobal::executeCommand("objExists " + addCurveAttractorFlag, cmdResult);
	cmdResult.getResult(attractorExists);
	if(!attractorExists){
		MGlobal::displayError("Specified attractor curve does not exist");
		return false;
	}
	
	// Get the plug representing the attractor xform output
	MSelectionList selList;
	selList.clear();
	selList.add(addCurveAttractorFlag);
	MDagPath curveAttractorPath;
	selList.getDagPath(0, curveAttractorPath);
	MMatrix curveAttractorInverseMatrix;
	curveAttractorInverseMatrix = curveAttractorPath.inclusiveMatrixInverse();
	MFnDependencyNode curveAttractorDepNode;
	curveAttractorDepNode.setObject(curveAttractorPath.node());
	MPlug curveAttractorXformPlugArray = curveAttractorDepNode.findPlug("worldMatrix");
	// The world xform we're interested in is the first element of the list
	MPlug curveAttractorXformPlug = curveAttractorXformPlugArray.elementByLogicalIndex(0);
	
	MDagPath crvShapePath;
	MStringArray crvShapesResult;
	MGlobal::executeCommand( MString("listRelatives -pa -shapes -ni ") + curveAttractorPath.fullPathName(), crvShapesResult);
	numShapes = crvShapesResult.length();
	MSelectionList shapesList;
	
	for(unsigned int i = 0; i < numShapes; i++){
		shapesList.clear();
		shapesList.add(crvShapesResult[i]);
		shapesList.getDagPath(0, crvShapePath);
		
		if(crvShapePath.apiType() != MFn::kNurbsCurve){
			MGlobal::displayError("Must supply a nurbs curve as a curve attractor.");
			continue;
		}
	}
	MFnDependencyNode curveAttractorShapeDepNode;
	curveAttractorShapeDepNode.setObject(crvShapePath.node());

	MPlug attractorCurvePlugArray = curveAttractorShapeDepNode.findPlug("worldSpace");
	// The world curve we're interested in is the first element of the list
	MPlug attractorCurvePlug = attractorCurvePlugArray.elementByLogicalIndex(0);
	
	// Add the attractor strength attribute to the curve if it hasn't already been added
	if(!curveAttractorDepNode.hasAttribute("attractorStrength")){
		dgModifier.commandToExecute("addAttr -ln \"attractorStrength\"  -at double  -min 0 -max 1 -dv 0 -keyable true " + addCurveAttractorFlag);
		dgModifier.doIt();
	}
	
	// get the plug representing the stretchMesh curve and strength attribute
	MPlug stretchMeshCurveAttractorPlugArray = deformerFnDepNode.findPlug("crvAttrctrCurve");
	MPlug stretchMeshCurveAttractorPlug;
	MPlug stretchMeshStrengthPlugArray = deformerFnDepNode.findPlug("crvAttrctrStrength");
	MPlug stretchMeshStrengthPlug;
	MPlug stretchMeshMatrixPlugArray = deformerFnDepNode.findPlug("matrix");
	MPlug stretchMeshMatrixPlug;
	
	// Find the first unconnected element of this array plug, use that to connect
	int crvItr = 0;
	bool foundElement = false;
	while(!foundElement){
		if(!stretchMeshCurveAttractorPlugArray[crvItr].isConnected()){
			foundElement = true;
			stretchMeshCurveAttractorPlug = stretchMeshCurveAttractorPlugArray.elementByLogicalIndex(crvItr);
			stretchMeshStrengthPlug = stretchMeshStrengthPlugArray.elementByLogicalIndex(crvItr);
			continue;
		}
		crvItr++;
	}			
	// Connect the curve attribute 
	dgModifier.connect(attractorCurvePlug, stretchMeshCurveAttractorPlug);
	
	// Connect the attractor strength attribute
	MPlug attractorStrengthPlug = curveAttractorDepNode.findPlug("attractorStrength");
	dgModifier.connect(attractorStrengthPlug, stretchMeshStrengthPlug);
	
	// now we need to find the first unconnected matrix plug (might not be the same index
	// as the curve attributes because there could be point attractors)
	int itr = 0;
	foundElement = false;
	while(!foundElement){
		if(!stretchMeshMatrixPlugArray[itr].isConnected()){
			foundElement = true;
			stretchMeshMatrixPlug = stretchMeshMatrixPlugArray.elementByLogicalIndex(itr);
			continue;
		}
		itr++;
	}			
	// Connect the matrix attribute 
	dgModifier.connect(curveAttractorXformPlug, stretchMeshMatrixPlug);

	
	// Find the closest point on the curve for each vertex, populate the array attribute representing
	// these UVs
	if(selected.length() != 1){
		MGlobal::displayError("Must specify a single mesh to add the curve attractor to.");
		return false;
	}
	
	MDagPath stretchMeshPath;
	MStringArray shapesResult;
	MStringArray selectionStrings;
	status = selected.getSelectionStrings(0, selectionStrings);
	if (!status){
		MGlobal::displayError(MString("Error: ") + status.errorString());
		return false;
	}

	MGlobal::executeCommand( MString("listConnections -shapes on ") + selectionStrings[0] + MString(".outputGeometry[0]"), shapesResult);
	numShapes = shapesResult.length();
	
	for(unsigned int i = 0; i < numShapes; i++){
		shapesList.clear();
		shapesList.add(shapesResult[i]);
		shapesList.getDagPath(0, stretchMeshPath);
		
		if(!stretchMeshPath.hasFn(MFn::kMesh)){
			MGlobal::displayError("StretchMesh must be applied to a polygonal mesh");
			continue;
		}
		// Populate an array with the input point positions 
		MItMeshVertex vertIter( stretchMeshPath );
		int vertCount = vertIter.count();
		inputPts.clear();
		inputPts.setLength(vertCount);
		for(vertIter.reset(); !vertIter.isDone(); vertIter.next() ){
			unsigned int vertIndex = vertIter.index();
			inputPts[vertIter.index()] = vertIter.position( MSpace::kWorld );
			// get the closest UV on the curve to the current point, and store this in the
			// deformer array
			MPoint closestPt;
			double param;
			MObject curveObj = curveAttractorShapeDepNode.object();
			cAttractorFn.setObject(curveObj);
			// because cAttractorFn is a non-transformed version of the curve, we have to multiply out point
			// by the inverse of the curve's matrix so that we find the closest point on the transformed curve.
			closestPt = cAttractorFn.closestPoint(inputPts[vertIndex]*curveAttractorInverseMatrix, &param, 0.0001, MSpace::kObject, &status);
			if(!status){
				MGlobal::displayInfo(MString("Error getting closest point... ") + status.errorString());
				status.perror("Couldn't get closest point: ");
			}
																
			MPlug crvAttachUVList = deformerFnDepNode.findPlug("crvAttrctrAttchUVList");
			MPlug crvAttachUVVert = crvAttachUVList.elementByLogicalIndex(vertIndex, &status);
			MPlug crvAttachUV;
			crvAttachUV = crvAttachUVVert.child(0, &status);
			MPlug crvAttachUVIndex = crvAttachUV.elementByLogicalIndex(crvItr);
			crvAttachUVIndex.setValue(param);
		}
	}
	
	return true;
	
}

MStatus stretchMeshCmd::buildstretchMeshCmdMenu()
{
	MString buildMenuCmd;
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Build Menu 
	// using -da 1 to let us know these menuItems are part of SM
	// future menu items can decide how to remove/manipulate this
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd = "global proc stretchMeshCmdMenu()\n";
	buildMenuCmd += "{\n";
	buildMenuCmd += "global string $stretchMeshCmdMenuCtrl;\n";
	buildMenuCmd += "string $name = \"Kickstand\";\n";
	buildMenuCmd += "global string $gMainWindow;\n";
	buildMenuCmd += "if (!`menu -exists $stretchMeshCmdMenuCtrl`)\n";
	buildMenuCmd += "	$stretchMeshCmdMenuCtrl = `menu -p $gMainWindow -to true -l $name -aob true`;\n";
	//buildMenuCmd += "deleteUI $stretchMeshCmdMenuCtrl;\n";
	buildMenuCmd += "menuItem -p $stretchMeshCmdMenuCtrl -da 1 -l \"Create New StretchMesh\" -c (\"stretchMesh()\") -ann \"Select items to deform\";\n";
	buildMenuCmd += "menuItem -p $stretchMeshCmdMenuCtrl -optionBox true -c (\"stretchMeshOption()\");\n";
    buildMenuCmd += "menuItem -p $stretchMeshCmdMenuCtrl -subMenu true -label \"Colliders\";\n";
	buildMenuCmd += "menuItem -da 1 -l \"Add Mesh Collider\" -c (\"addstretchMeshCmdCollisionObj()\") -ann \"Select StretchMesh surface followed by collision object\";\n";
	buildMenuCmd += "menuItem -da 1 -l \"Add Curve Collider\" -c (\"addStretchMeshCurveCollider()\") -ann \"Select StretchMesh surface followed by collision curve\";\n";
	buildMenuCmd += "menuItem -da 1 -l \"Add Sphere Collider\" -c (\"addStretchMeshSphereCollider()\") -ann \"Select StretchMesh surface followed by collision locator\";\n";
	buildMenuCmd += "setParent -menu ..;\n";
	buildMenuCmd += "menuItem -p $stretchMeshCmdMenuCtrl -subMenu true -label \"Attractors\";\n";
	buildMenuCmd += "menuItem -da 1 -l \"Add Point Attractor To Selected\" -c (\"addNewAttrctr()\") -ann \"Select vertices on StretchMesh object\";\n";
	buildMenuCmd += "menuItem -da 1 -l \"Add Curve Attractor To Selected\" -c (\"addNewCrvAttrctr()\") -ann \"Select vertices on StretchMesh object\";\n";
	buildMenuCmd += "setParent -menu ..;\n";
	buildMenuCmd += "menuItem -p $stretchMeshCmdMenuCtrl -subMenu true -label \"Paint\";\n";
	buildMenuCmd += "menuItem -da 1 -l \"Paint Attractor Influence\" -c (\"artAttrStretchMeshToolScript(3, \\\"attractors\\\")\");\n";
	buildMenuCmd += "menuItem -da 1 -l \"Paint Collider Influence\" -c (\"artAttrStretchMeshToolScript(3, \\\"colliders\\\")\");\n";
	buildMenuCmd += "menuItem -da 1 -l \"Paint Stiffness\" -c (\"paintStiffness()\");\n";
	buildMenuCmd += "setParent -menu ..;\n";
	buildMenuCmd += "menuItem -d true;\n";
	buildMenuCmd += "menuItem -p $stretchMeshCmdMenuCtrl -da 1 -l \"Display MAC Address\" -c (\"ksLicenseRequest()\");\n";
	buildMenuCmd += "menuItem -p $stretchMeshCmdMenuCtrl -da 1 -l \"Help\" -c (\"stretchMeshHelpLaunch()\");\n";
	
	buildMenuCmd += "}\n";
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Remove Menu
	// removes only SM menu items (-da 1)
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd += "global proc stretchMeshCmdMenuRemove()\n";
	buildMenuCmd += "{\n";
	buildMenuCmd += "global string $stretchMeshCmdMenuCtrl;\n";
	buildMenuCmd += "if (size(`menu -q -ia $stretchMeshCmdMenuCtrl`) > 0 )\n";
	buildMenuCmd += "	for ( $menu in `menu -q -ia $stretchMeshCmdMenuCtrl`)\n";
	buildMenuCmd += "	{\n";
	buildMenuCmd += "			deleteUI -mi $menu;\n";
	buildMenuCmd += "	}\n";
	buildMenuCmd += "if (size(`menu -q -ia $stretchMeshCmdMenuCtrl`) < 1 )\n";
	buildMenuCmd += "	deleteUI -m $stretchMeshCmdMenuCtrl;\n";
	buildMenuCmd += "}\n";

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Launch Help Docs
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd += "global proc stretchMeshHelpLaunch()\n";
	buildMenuCmd += "{\n";
	buildMenuCmd += "	showHelp -absolute \"http://kickstandlabs.com/tools/stretchmesh/doc\";\n";
	buildMenuCmd += "}\n";
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// StretchMeshOptions... builds the stretchMesh option UI
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd += "global proc stretchMeshOption()\n";
	buildMenuCmd += "{\n";
	buildMenuCmd += "		global string $stretchMeshOptionsWindow;\n";
	buildMenuCmd += "		global string $stretchMeshCollisionStepUI;\n";
	buildMenuCmd += "		global string $stretchMeshIterationsUI;\n";
	buildMenuCmd += "		global string $stretchMeshStiffnessUI;\n";
	buildMenuCmd += "		global string $stretchMeshCollisionsUI;\n";
	buildMenuCmd += "		global string $stretchMeshScaleSafeUI;\n";

	
	buildMenuCmd += "		$stretchMeshOptionsWindow = `window -title \"Create StretchMesh Options\"\n";
	buildMenuCmd += "		-iconName \"StretchMesh\"\n";
	buildMenuCmd += "		-widthHeight 300 300`;\n";
	buildMenuCmd += "		string $form = `formLayout -numberOfDivisions 100`;\n";
	buildMenuCmd += "		string $b1 = `button -label \"Create \" -command (\"stretchMesh -cs (`intFieldGrp -q -value1 $stretchMeshCollisionStepUI`) -i (`intFieldGrp -q -value1 $stretchMeshIterationsUI`) -c (`checkBoxGrp -q -value1 $stretchMeshCollisionsUI`) -s (`floatFieldGrp -q -value1 $stretchMeshStiffnessUI`) -ess (`checkBoxGrp -q -value1 $stretchMeshScaleSafeUI`) \")`;\n";
	buildMenuCmd += "		string $b2 = `button -label \"Close \" -command (\"deleteUI -window \" + $stretchMeshOptionsWindow)`;\n";
	buildMenuCmd += "		string $column = `columnLayout -adjustableColumn true`;\n";
	buildMenuCmd += "		$stretchMeshCollisionStepUI = `intFieldGrp -numberOfFields 1\n";
	buildMenuCmd += "		-label \"Collision Step: \" \n";
	buildMenuCmd += "		-value1 1`;\n";
	buildMenuCmd += "		$stretchMeshIterationsUI = `intFieldGrp -numberOfFields 1\n";
	buildMenuCmd += "		-label \"Iterations: \"  \n";
	buildMenuCmd += "		-value1 10`;\n";
	buildMenuCmd += "		$stretchMeshStiffnessUI = `floatFieldGrp -numberOfFields 1\n";
	buildMenuCmd += "		-label \"Stiffness: \"  \n";
	buildMenuCmd += "		-value1 0.3`;\n";
	buildMenuCmd += "		$stretchMeshCollisionsUI = `checkBoxGrp -numberOfCheckBoxes 1\n";
	buildMenuCmd += "		-label \"Collisions \" \n";
	buildMenuCmd += "		-value1 true`;\n";
	buildMenuCmd += "		$stretchMeshScaleSafeUI = `checkBoxGrp -numberOfCheckBoxes 1\n";
	buildMenuCmd += "		-label \"Enable Scale Safe \" \n";
	buildMenuCmd += "		-value1 true`;\n";
	
	buildMenuCmd += "		formLayout -edit\n";
	buildMenuCmd += "		 -attachNone     $b1     \"top\"\n";
	buildMenuCmd += "		-attachForm     $b1     \"left\"   5\n";
	buildMenuCmd += "		-attachForm     $b1     \"bottom\" 5 \n";
	buildMenuCmd += "		-attachPosition $b1     \"right\"  5 50\n";
	
	buildMenuCmd += "		-attachNone     $b2     \"top\"\n";
	buildMenuCmd += "		-attachPosition $b2     \"left\"  0 50\n";
	buildMenuCmd += "		-attachForm     $b2     \"bottom\" 5 \n";
	buildMenuCmd += "		-attachForm $b2     \"right\"  5\n";
	
	buildMenuCmd += "		-attachForm     $column \"top\"    15\n";
	buildMenuCmd += "		-attachForm     $column \"left\"   5\n";
	buildMenuCmd += "		-attachForm     $column \"right\" 5 \n";
	buildMenuCmd += "		-attachNone     $column \"bottom\"  \n";
	buildMenuCmd += "		$form;\n";
	
	buildMenuCmd += "		showWindow $stretchMeshOptionsWindow;\n";
	buildMenuCmd += "}\n";
	

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Add collision object
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd += "global proc addstretchMeshCmdCollisionObj(){\n";
	buildMenuCmd += "	int $i;\n";
	buildMenuCmd += "	string $shapeNode;\n";
	buildMenuCmd += "	string $selected[] = `ls -sl -fl`;\n";
	buildMenuCmd += "	string $stretchMeshCmdDfrmr;\n";
	buildMenuCmd += "	int $numColliders;\n";
	
	buildMenuCmd += "	if (`size $selected` != 2){\n";
	buildMenuCmd += "		error \"Select the stretchMeshCmd geometry and the collision transform\";\n";
	buildMenuCmd += "		return;\n";
	buildMenuCmd += "	}\n";
	// From the selected objects, the first should have a stretch mesh deformer.  Find that deformer..
	buildMenuCmd += "	string $history[] = `listHistory($selected[0])`;\n";
	buildMenuCmd += "	for($i = 0; $i < `size($history)`; $i++){\n";
	buildMenuCmd += "		if(`nodeType($history[$i])` == \"stretchMesh\"){\n";
	buildMenuCmd += "			$stretchMeshCmdDfrmr = $history[$i];\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	
	// Temporarily disable the stretchMesh node so it isn't calculating as we go
	buildMenuCmd += "	setAttr (($stretchMeshCmdDfrmr + \".nodeState\"), 1);\n";

	buildMenuCmd += "	if($stretchMeshCmdDfrmr == \"\"){\n";
	buildMenuCmd += "		error \"Select the stretchMeshCmd geometry and the collision transform\";\n";
	buildMenuCmd += "		return;\n";
	buildMenuCmd += "	}\n";
	
	// Next, determine the shape node of the second selected object
	buildMenuCmd += "	string $history[] = `listHistory($selected[1])`;\n";
	buildMenuCmd += "	for($i = 0; $i < `size($history)`; $i++){\n";
		// collision object is a polymesh...
	buildMenuCmd += "		if(`nodeType($history[$i])` == \"mesh\"){\n";
	buildMenuCmd += "			$shapeNode = $history[$i];\n";
	buildMenuCmd += "			$numColliders = `getAttr -size ($stretchMeshCmdDfrmr + \".mshCollider\")`;\n";
			//search through existing .mshCollider attrs looking for an unconnected attr, use it if we 
			//find one (this prevents buildup of attrs if we repeatedly create and delete colliders).
	buildMenuCmd += "			int $madeConnection = 0;\n";
	buildMenuCmd += "			for($j = 0; $j < $numColliders; $j++){\n";
	buildMenuCmd += "				if( !`connectionInfo -isDestination ($stretchMeshCmdDfrmr + \".mshCollider[\" + $j + \"]\")`){\n";
	buildMenuCmd += "					connectAttr -f ($shapeNode + \".worldMesh[0]\") ($stretchMeshCmdDfrmr + \".mshCollider[\" + $j + \"]\");\n";
					//set the corresponding pad attribute
	buildMenuCmd += "					setAttr ($stretchMeshCmdDfrmr + \".mshColliderPad[\" + $j + \"]\") 0;\n";
	buildMenuCmd += "					$madeConnection = 1;\n";
	buildMenuCmd += "					break;\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
			
			//if we didn't find an existing unconnected attr, connect it to a new attribute
	buildMenuCmd += "			if( !$madeConnection ){\n";
	buildMenuCmd += "				connectAttr -f ($shapeNode + \".worldMesh[0]\") ($stretchMeshCmdDfrmr + \".mshCollider[\" + $numColliders + \"]\");\n";
				//set the corresponding pad attribute
	buildMenuCmd += "				setAttr ($stretchMeshCmdDfrmr + \".mshColliderPad[\" + $numColliders + \"]\") 0;\n";
	buildMenuCmd += "			}\n";
	// We search the existing matrix array elements for one that is not connected. 
	// if we find it, we connect our new attractor to that element. 
	// otherwise, we connect it at a new element
	buildMenuCmd += "			$numColliders = `getAttr -size ($stretchMeshCmdDfrmr + \".matrix\")`;\n";
	buildMenuCmd += "			int $emptyElement = $numColliders;\n";
	buildMenuCmd += "			for($i = 0; $i < $numColliders; $i++){\n";
	buildMenuCmd += "				if(size(`listConnections ($stretchMeshCmdDfrmr + \".matrix[\" + $i + \"]\")`) == 0){\n";
	buildMenuCmd += "					$emptyElement = $i;\n";
	buildMenuCmd += "					break;\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "			connectAttr(($shapeNode + \".worldMatrix[0]\"), ($stretchMeshCmdDfrmr + \".matrix[\" + $emptyElement + \"]\"));\n";	
	buildMenuCmd += "		}else if(`nodeType($history[$i])` == \"nurbsSurface\"){\n";
	buildMenuCmd += "			$shapeNode = $history[$i];\n";
	buildMenuCmd += "			$numColliders = `getAttr -size ($stretchMeshCmdDfrmr + \".nrbsCollider\")`;\n";
			//search through existing .mshCollider attrs looking for an unconnected attr, use it if we 
			//find one (this prevents buildup of attrs if we repeatedly create and delete colliders).
	buildMenuCmd += "			int $madeConnection = 0;\n";
	buildMenuCmd += "			for($j = 0; $j < $numColliders; $j++){\n";
	buildMenuCmd += "				if( !`connectionInfo -isDestination ($stretchMeshCmdDfrmr + \".nrbsCollider[\" + $j + \"]\")`){\n";
	buildMenuCmd += "					connectAttr -f ($shapeNode + \".worldSpace[0]\") ($stretchMeshCmdDfrmr + \".nrbsCollider[\" + $j + \"]\");\n";
	buildMenuCmd += "					$madeConnection = 1;\n";
	buildMenuCmd += "					break;\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
			
			//if we didn't find an existing unconnected attr, connect it to a new attribute
	buildMenuCmd += "			if( !$madeConnection ){\n";
	buildMenuCmd += "				connectAttr -f ($shapeNode + \".worldSpace[0]\") ($stretchMeshCmdDfrmr + \".nrbsCollider[\" + $numColliders + \"]\");\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";

	
	buildMenuCmd += "	addAttr -ln \"colliderMult\"  -at double  -min 0 -max 1 -dv 1 -keyable true $selected[1];\n";
	
	// Search for unconnected mshColliderMult attribute
	buildMenuCmd += "	int $numColliderMults = `getAttr -size ($stretchMeshCmdDfrmr + \".mshColliderMult\")`;\n";
	buildMenuCmd += "	int $emptyMshCollider = $numColliderMults;\n";
	buildMenuCmd += "	for($i = 0; $i < $numColliderMults; $i++){\n";
	buildMenuCmd += "		if(size(`listConnections ($stretchMeshCmdDfrmr + \".mshColliderMult[\" + $i + \"]\")`) == 0){\n";
	buildMenuCmd += "			$emptyMshCollider = $i;\n";
	buildMenuCmd += "			break;\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	buildMenuCmd += "	connectAttr (($selected[1] + \".colliderMult\"), ($stretchMeshCmdDfrmr + \".mshColliderMult[\" + $emptyMshCollider + \"]\"));\n";
	
	//We need to initialize the collider per-vertex mults to one,
	buildMenuCmd += "	int $numVerts[];\n";
	buildMenuCmd += "	$numVerts = `polyEvaluate -vertex $selected[0]`;\n";
	buildMenuCmd += "	for($i = 0; $i < $numVerts[0]; $i++){\n";
	buildMenuCmd += "		setAttr(($stretchMeshCmdDfrmr + \".mshColliderVrtMultList[\" + $i + \"].mshColliderVrtMult[\" + $emptyMshCollider + \"]\"), 1.0);\n";	
	buildMenuCmd += "	}\n";	
	// Enable the stretchMeshCmd now that we're done
	buildMenuCmd += "	setAttr (($stretchMeshCmdDfrmr + \".nodeState\"), 0);\n";
	buildMenuCmd += "}\n";
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Add curve collider
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd += "global proc addStretchMeshCurveCollider(){\n";
	buildMenuCmd += "	int $i;\n";
	buildMenuCmd += "	string $shapeNode;\n";
	buildMenuCmd += "	string $selected[] = `ls -sl -fl`;\n";
	buildMenuCmd += "	string $stretchMeshCmdDfrmr;\n";
	buildMenuCmd += "	int $numColliders;\n";
	buildMenuCmd += " int $crvColliderSpans;\n";
	buildMenuCmd += "	string $crvColliderLoc;\n";
	
	buildMenuCmd += "	if (`size $selected` != 2){\n";
	buildMenuCmd += "		error \"Select the stretchMeshCmd geometry and the collision transform\";\n";
	buildMenuCmd += "		return;\n";
	buildMenuCmd += "	}\n";
	// From the selected objects, the first should have a stretch mesh deformer.  Find that deformer..
	buildMenuCmd += "	string $history[] = `listHistory($selected[0])`;\n";
	buildMenuCmd += "	for($i = 0; $i < `size($history)`; $i++){\n";
	buildMenuCmd += "		if(`nodeType($history[$i])` == \"stretchMesh\"){\n";
	buildMenuCmd += "			$stretchMeshCmdDfrmr = $history[$i];\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	
	buildMenuCmd += "	if($stretchMeshCmdDfrmr == \"\"){\n";
	buildMenuCmd += "		error \"Select the stretchMeshCmd geometry and the collision transform\";\n";
	buildMenuCmd += "		return;\n";
	buildMenuCmd += "	}\n";
	
	// Temporarily disable the stretchMesh node so it isn't calculating as we go
	buildMenuCmd += "	setAttr (($stretchMeshCmdDfrmr + \".nodeState\"), 1);\n";
	
	// Next, determine the shape node of the second selected object
	buildMenuCmd += "	string $history[] = `listHistory($selected[1])`;\n";
	buildMenuCmd += "	for($i = 0; $i < `size($history)`; $i++){\n";
	// collision object is a polymesh...
	buildMenuCmd += "		if(`nodeType($history[$i])` == \"nurbsCurve\"){\n";
	buildMenuCmd += "			$shapeNode = $history[$i];\n";
	buildMenuCmd += "			$crvColliderSpans = `getAttr ($shapeNode + \".spans\")`;\n";
	buildMenuCmd += "			$crvColliderSpans = $crvColliderSpans + 1;\n";
	buildMenuCmd += "			$numColliders = `getAttr -size ($stretchMeshCmdDfrmr + \".primCrvCollider\")`;\n";
	//search through existing .mshCollider attrs looking for an unconnected attr, use it if we 
	//find one (this prevents buildup of attrs if we repeatedly create and delete colliders).
	buildMenuCmd += "			int $madeConnection = 0;\n";
	buildMenuCmd += "			for($j = 0; $j < $numColliders; $j++){\n";
	buildMenuCmd += "				if( !`connectionInfo -isDestination ($stretchMeshCmdDfrmr + \".primCrvCollider[\" + $j + \"]\")`){\n";
	buildMenuCmd += "					connectAttr -f ($shapeNode + \".worldSpace[0]\") ($stretchMeshCmdDfrmr + \".primCrvCollider[\" + $j + \"]\");\n";
	// Plug the crve into the curve collider locator 
	buildMenuCmd += "					$crvColliderLoc = `createNode curveColliderLocator`;\n";
	buildMenuCmd += "					connectAttr -f ($shapeNode + \".worldSpace[0]\") ($crvColliderLoc + \".colliderCurve\");\n";
	buildMenuCmd += "					int $crvItr;\n";
	buildMenuCmd += "					for($crvItr = 0; $crvItr < $crvColliderSpans; $crvItr++){\n";
	buildMenuCmd += "						setAttr ($crvColliderLoc + \".radius[\" + $crvItr + \"]\") 1.0;\n";
	buildMenuCmd += "					}\n";
	buildMenuCmd += "					$madeConnection = 1;\n";
	buildMenuCmd += "					break;\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
	
	//if we didn't find an existing unconnected attr, connect it to a new attribute
	buildMenuCmd += "			if( !$madeConnection ){\n";
	buildMenuCmd += "				connectAttr -f ($shapeNode + \".worldSpace[0]\") ($stretchMeshCmdDfrmr + \".primCrvCollider[\" + $numColliders + \"]\");\n";
	// Plug the crve into the curve collider locator 
	buildMenuCmd += "				$crvColliderLoc = `createNode curveColliderLocator`;\n";
	buildMenuCmd += "				connectAttr -f ($shapeNode + \".worldSpace[0]\") ($crvColliderLoc + \".colliderCurve\");\n";
	buildMenuCmd += "				int $crvItr;\n";
	buildMenuCmd += "				for($crvItr = 0; $crvItr < $crvColliderSpans; $crvItr++){\n";
	buildMenuCmd += "					setAttr ($crvColliderLoc + \".radius[\" + $crvItr + \"]\") 1.0;\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";	
	// We search the existing matrix array elements for one that is not connected. 
	// if we find it, we connect our new attractor to that element. 
	// otherwise, we connect it at a new element
	buildMenuCmd += "			int $numColliders = `getAttr -size ($stretchMeshCmdDfrmr + \".matrix\")`;\n";
	buildMenuCmd += "			int $emptyElement = $numColliders;\n";
	buildMenuCmd += "			for($i = 0; $i < $numColliders; $i++){\n";
	buildMenuCmd += "				if(size(`listConnections ($stretchMeshCmdDfrmr + \".matrix[\" + $i + \"]\")`) == 0){\n";
	buildMenuCmd += "					$emptyElement = $i;\n";
	buildMenuCmd += "					break;\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "			connectAttr(($shapeNode + \".worldMatrix[0]\"), ($stretchMeshCmdDfrmr + \".matrix[\" + $emptyElement + \"]\"));\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	buildMenuCmd += " if (!`attributeExists \"colliderMult\" $selected[1]`) {\n";
	buildMenuCmd += "		addAttr -ln \"colliderMult\"  -at double  -min 0 -max 1 -dv 1 -keyable true $selected[1];\n";
	buildMenuCmd += "	}\n";
	
	// Search for unconnected crvColliderMult attribute
	buildMenuCmd += "	int $numColliderMults = `getAttr -size ($stretchMeshCmdDfrmr + \".crvColliderMult\")`;\n";
	buildMenuCmd += "	int $emptyCrvCollider = $numColliderMults;\n";
	buildMenuCmd += "	for($i = 0; $i < $numColliderMults; $i++){\n";
	buildMenuCmd += "		if(size(`listConnections ($stretchMeshCmdDfrmr + \".crvColliderMult[\" + $i + \"]\")`) == 0){\n";
	buildMenuCmd += "			$emptyCrvCollider = $i;\n";
	buildMenuCmd += "			break;\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	buildMenuCmd += "	connectAttr (($selected[1] + \".colliderMult\"), ($stretchMeshCmdDfrmr + \".crvColliderMult[\" + $emptyCrvCollider + \"]\"));\n";

	//We need to initialize the collider per-vertex mults to one,
	buildMenuCmd += "	int $numVerts[];\n";
	buildMenuCmd += "	$numVerts = `polyEvaluate -vertex $selected[0]`;\n";
	buildMenuCmd += "	for($i = 0; $i < $numVerts[0]; $i++){\n";
	buildMenuCmd += "		setAttr(($stretchMeshCmdDfrmr + \".crvColliderVrtMultList[\" + $i + \"].crvColliderVrtMult[\" + $emptyCrvCollider + \"]\"), 1.0);\n";	
	buildMenuCmd += "	}\n";	
	
	//We need to connect the crvColliderLocator radius attributes to the SM deformer
	buildMenuCmd += "	for($crvItr = 0; $crvItr < $crvColliderSpans; $crvItr++){\n";
	buildMenuCmd += "		connectAttr -f ($crvColliderLoc + \".radius[\" + $crvItr + \"]\") ($stretchMeshCmdDfrmr + \".crvColliderRadiusList[\" + $emptyCrvCollider + \"].crvColliderRadius[\" + $crvItr + \"]\");\n";
	buildMenuCmd += "	}\n";	
	
	// Enable the stretchMeshCmd now that we're done
	buildMenuCmd += "	setAttr (($stretchMeshCmdDfrmr + \".nodeState\"), 0);\n";
	buildMenuCmd += "}\n";
	

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Add primitive sphere collider
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd += "global proc addStretchMeshSphereCollider(){\n";
	buildMenuCmd += "	int $i;\n";
	buildMenuCmd += "	string $shapeNode;\n";
	buildMenuCmd += "	string $selected[] = `ls -sl -fl`;\n";
	buildMenuCmd += "	string $stretchMeshCmdDfrmr;\n";
	buildMenuCmd += "	int $numColliders;\n";
	
	buildMenuCmd += "	if (`size $selected` != 2){\n";
	buildMenuCmd += "		error \"Select the stretchMeshCmd geometry and the collision transform\";\n";
	buildMenuCmd += "		return;\n";
	buildMenuCmd += "	}\n";
	// From the selected objects, the first should have a stretch mesh deformer.  Find that deformer..
	buildMenuCmd += "	string $history[] = `listHistory($selected[0])`;\n";
	buildMenuCmd += "	for($i = 0; $i < `size($history)`; $i++){\n";
	buildMenuCmd += "		if(`nodeType($history[$i])` == \"stretchMesh\"){\n";
	buildMenuCmd += "			$stretchMeshCmdDfrmr = $history[$i];\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	
	buildMenuCmd += "	if($stretchMeshCmdDfrmr == \"\"){\n";
	buildMenuCmd += "		error \"Select the stretchMeshCmd geometry and the collision transform\";\n";
	buildMenuCmd += "		return;\n";
	buildMenuCmd += "	}\n";
	
	// Temporarily disable the stretchMesh node so it isn't calculating as we go
	buildMenuCmd += "	setAttr (($stretchMeshCmdDfrmr + \".nodeState\"), 1);\n";
	
	// Next, determine the shape node of the second selected object
	// collision object is a polymesh...
	buildMenuCmd += "	if(`nodeType($selected[1])` == \"transform\"){\n";
	buildMenuCmd += "		$numColliders = `getAttr -size ($stretchMeshCmdDfrmr + \".primSphrCollider\")`;\n";
	//search through existing .primSphrCollider attrs looking for an unconnected attr, use it if we 
	//find one (this prevents buildup of attrs if we repeatedly create and delete colliders).
	buildMenuCmd += "		int $madeConnection = 0;\n";
	buildMenuCmd += "		for($j = 0; $j < $numColliders; $j++){\n";
	buildMenuCmd += "			if( !`connectionInfo -isDestination ($stretchMeshCmdDfrmr + \".primSphrCollider[\" + $j + \"]\")`){\n";
	buildMenuCmd += "				connectAttr -f ($selected[1] + \".worldMatrix[0]\") ($stretchMeshCmdDfrmr + \".primSphrCollider[\" + $j + \"]\");\n";
	buildMenuCmd += "				$madeConnection = 1;\n";
	buildMenuCmd += "				break;\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";
	
	//if we didn't find an existing unconnected attr, connect it to a new attribute
	buildMenuCmd += "		if( !$madeConnection ){\n";
	buildMenuCmd += "			connectAttr -f ($selected[1] + \".worldMatrix[0]\") ($stretchMeshCmdDfrmr + \".primSphrCollider[\" + $numColliders + \"]\");\n";
	buildMenuCmd += "		}\n";	
	// We search the existing matrix array elements for one that is not connected. 
	// if we find it, we connect our new attractor to that element. 
	// otherwise, we connect it at a new element
	buildMenuCmd += "		int $numColliders = `getAttr -size ($stretchMeshCmdDfrmr + \".matrix\")`;\n";
	buildMenuCmd += "		int $emptyElement = $numColliders;\n";
	buildMenuCmd += "		for($i = 0; $i < $numColliders; $i++){\n";
	buildMenuCmd += "			if(size(`listConnections ($stretchMeshCmdDfrmr + \".matrix[\" + $i + \"]\")`) == 0){\n";
	buildMenuCmd += "				$emptyElement = $i;\n";
	buildMenuCmd += "				break;\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "		connectAttr(($selected[1] + \".worldMatrix[0]\"), ($stretchMeshCmdDfrmr + \".matrix[\" + $emptyElement + \"]\"));\n";
	buildMenuCmd += "	}\n";
	
	buildMenuCmd += "	addAttr -ln \"colliderMult\"  -at double  -min 0 -max 1 -dv 1 -keyable true $selected[1];\n";
	
	// Search for unconnected mshColliderMult attribute
	buildMenuCmd += "	int $numColliderMults = `getAttr -size ($stretchMeshCmdDfrmr + \".primSphrColliderMult\")`;\n";
	buildMenuCmd += "	int $emptySphereCollider = $numColliderMults;\n";
	buildMenuCmd += "	for($i = 0; $i < $numColliderMults; $i++){\n";
	buildMenuCmd += "		if(size(`listConnections ($stretchMeshCmdDfrmr + \".primSphrColliderMult[\" + $i + \"]\")`) == 0){\n";
	buildMenuCmd += "			$emptySphereCollider = $i;\n";
	buildMenuCmd += "			break;\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	buildMenuCmd += "	connectAttr (($selected[1] + \".colliderMult\"), ($stretchMeshCmdDfrmr + \".primSphrColliderMult[\" + $emptySphereCollider + \"]\"));\n";
	
	//We need to initialize the collider per-vertex mults to one,
	buildMenuCmd += "	int $numVerts[];\n";
	buildMenuCmd += "	$numVerts = `polyEvaluate -vertex $selected[0]`;\n";
	buildMenuCmd += "	for($i = 0; $i < $numVerts[0]; $i++){\n";
	buildMenuCmd += "		setAttr(($stretchMeshCmdDfrmr + \".primSphrColliderVrtMultList[\" + $i + \"].primSphrColliderVrtMult[\" + $emptySphereCollider + \"]\"), 1.0);\n";	
	buildMenuCmd += "	}\n";	

	// Enable the stretchMeshCmd now that we're done
	buildMenuCmd += "	setAttr (($stretchMeshCmdDfrmr + \".nodeState\"), 0);\n";
	buildMenuCmd += "}\n";
	

	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Paint stiffness values
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd += "global proc paintStiffness(){\n";
	buildMenuCmd += "		string $sel[];\n";
	buildMenuCmd += "		$sel = `ls -sl -fl`;\n";
	buildMenuCmd += "		int $i;\n";
	buildMenuCmd += "		int $j;\n";	
	buildMenuCmd += "		string $stretchMesh;\n";		
	buildMenuCmd += "		for($i = 0; $i < `size($sel)`; $i++){\n";
	
	buildMenuCmd += "			string $history[] = `listHistory($sel[$i])`;\n";
	buildMenuCmd += "			for($j = 0; $j < `size($history)`; $j++){\n";
	buildMenuCmd += "				if(`nodeType($history[$j])` == \"stretchMesh\"){\n";
	buildMenuCmd += "					$stretchMesh = $history[$j];\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "		if($stretchMesh != \"\"){\n";
	buildMenuCmd += "			ArtPaintAttrToolOptions;\n";
	buildMenuCmd += "			artSetToolAndSelectAttr( \"artAttrCtx\", \"weightGeometryFilter.\" + $stretchMesh + \".stiffness\" );\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "}\n";
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Add attractor to selected verts
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	
	buildMenuCmd += "global proc addNewAttrctr(){\n";
	// One of the selected objects should have a stretchMeshCmd, find it...
	
	buildMenuCmd += "	string $sel[];\n";
	buildMenuCmd += "	$sel = `ls -sl -fl`;\n";
	buildMenuCmd += "	int $i;\n";
	buildMenuCmd += "	int $j;\n";	
	buildMenuCmd += "	string $stretchMesh;\n";		
	buildMenuCmd += "	for($i = 0; $i < `size($sel)`; $i++){\n";
	
	buildMenuCmd += "		string $history[] = `listHistory($sel[$i])`;\n";
	buildMenuCmd += "		for($j = 0; $j < `size($history)`; $j++){\n";
	buildMenuCmd += "			if(`nodeType($history[$j])` == \"stretchMesh\"){\n";
	buildMenuCmd += "				$stretchMesh = $history[$j];\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "}\n";
	
	buildMenuCmd += "	if($stretchMesh == \"\"){\n";
	buildMenuCmd += "		error \"Select vertices on a mesh that has a stretchMeshCmd deformer\";\n";
	buildMenuCmd += "		return;\n";
	buildMenuCmd += "	}\n";

	// Temporarily disable the stretchMeshCmd so it isn't evaluating as 
	// as we go
	buildMenuCmd += "	setAttr (($stretchMesh + \".nodeState\"), 1);\n";
	
	// First, we need to find out if there is a locator selected. If so, we'll use that to 
	// connect the vertices to.  If not, we'll create a new locator.
	buildMenuCmd += "	string $locator = \"\";\n";
	buildMenuCmd += "	for($i = 0; $i < `size($sel)`; $i++){\n";
	buildMenuCmd += "		if (`nodeType($sel[$i])` == \"transform\"){\n";
	buildMenuCmd += "			$locator = $sel[$i];\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";

	// Create attractor locator and make connections
	buildMenuCmd += "	if ($locator == \"\"){\n";
	buildMenuCmd += "		string $locatorArray[];\n";
	buildMenuCmd +="		float $locatorPosition[] = `getSelectionCenter($sel)`;\n";
	buildMenuCmd += "		$locatorArray = `spaceLocator -name attractor`;\n";
	buildMenuCmd += "		$locator = $locatorArray[0];\n";
	buildMenuCmd += "		xform -t $locatorPosition[0] $locatorPosition[1] $locatorPosition[2] $locator;\n";
	buildMenuCmd += "		addAttr -ln \"attractorStrength\"  -at double  -min 0 -max 1 -dv 0 -keyable true $locator;\n";
	buildMenuCmd += "	}\n";
	buildMenuCmd += "	int $numAttrctrs = `getAttr -size ($stretchMesh + \".matrix\")`;\n";

	//First we search the existing matrix array elements for one that is not connected. 
	// if we find it, we connect our new attractor to that element. 
	// otherwise, we connect it at a new element
	buildMenuCmd += "int $emptyElement = $numAttrctrs;\n";
	buildMenuCmd += "for($i = 0; $i < $numAttrctrs; $i++){\n";
	buildMenuCmd += "		if(size(`listConnections ($stretchMesh + \".matrix[\" + $i + \"]\")`) == 0){\n";
	buildMenuCmd += "			$emptyElement = $i;\n";
	buildMenuCmd += "			break;\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";

	//We need to initialize the attractor mults to zero, in case any previously created/deleted attractors have old values in there
	buildMenuCmd += "	int $numVerts[];\n";
	buildMenuCmd += "	for($i = 0; $i < `size($sel)`; $i++){\n";
	buildMenuCmd += "		if (`nodeType($sel[$i])` == \"mesh\"){\n";
	buildMenuCmd += "			$numVerts = `polyEvaluate -vertex $sel[$i]`;\n";
	buildMenuCmd += "			break;\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	buildMenuCmd += "	for($i = 0; $i < $numVerts[0]; $i++){\n";
	buildMenuCmd += "		setAttr(($stretchMesh + \".attrctrVrtMultList[\" + $i + \"].attrctrVrtMult[\" + $emptyElement + \"]\"), 0.0);\n";	
	buildMenuCmd += "}\n";
	
	buildMenuCmd += "int $locatorConnected = 0;\n";
	buildMenuCmd += "	for($i = 0; $i < `size($sel)`; $i++){\n";
	buildMenuCmd += "		if (`nodeType($sel[$i])` == \"transform\"){\n";
	buildMenuCmd += "			continue;\n";
	buildMenuCmd += "		}\n";
	// The \[ may be an issue for linux...maybe just assume [ and ]...or figure out what's with linux! heh
	buildMenuCmd += "		string $buf[];\n";
	buildMenuCmd += "		int $s = `tokenize $sel[$i] \"[]\" $buf`;\n";
	buildMenuCmd += "		string $match = $buf[$s-1];\n";
	//buildMenuCmd += "		string $match = `match \"\[[0-9]+\]\" $sel[$i]`;\n";
	//buildMenuCmd += "		$match = `substitute \"\\\\[\" $match \"\"`;\n";
	//buildMenuCmd += "		$match = `substitute \"\\\\]\" $match \"\"`;\n";

	buildMenuCmd += "		if ($match != \"\"){\n";
	buildMenuCmd += "			int $vert_id = $match;\n";
	buildMenuCmd += "			setAttr(($stretchMesh + \".attrctrVrtMultList[\" + $vert_id + \"].attrctrVrtMult[\" + $emptyElement + \"]\"), 1.0);\n";
	buildMenuCmd += "			if(!$locatorConnected){\n";
	buildMenuCmd += "				connectAttr(($locator + \".worldMatrix[0]\"), ($stretchMesh + \".matrix[\" + $emptyElement + \"]\"));\n";
	buildMenuCmd += "				connectAttr (($locator + \".attractorStrength\"), ($stretchMesh + \".attrctrStrength[\" + $emptyElement + \"]\"));\n";
	buildMenuCmd += "				$locatorConnected = 1;";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	
	// Enable the stretchMeshCmd now that we're done
	buildMenuCmd += "	setAttr (($stretchMesh + \".nodeState\"), 0);\n";
	buildMenuCmd += "}\n";
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// End Add attractor to selected verts
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	

	
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Add curve attractor to selected verts
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	
	buildMenuCmd += "global proc addNewCrvAttrctr(){\n";
	// One of the selected objects should have a stretchMesh node, find it...
	
	buildMenuCmd += "	string $sel[];\n";
	buildMenuCmd += "	$sel = `ls -sl -fl`;\n";
	buildMenuCmd += "	int $i;\n";
	buildMenuCmd += "	int $j;\n";	
	buildMenuCmd += "	int $numCrvAttractors;\n";	
	buildMenuCmd += "	string $stretchMesh;\n";		
	buildMenuCmd += "	string $stretchMeshPoly;\n";
	buildMenuCmd += "	for($i = 0; $i < `size($sel)`; $i++){\n";
	
	buildMenuCmd += "		string $history[] = `listHistory($sel[$i])`;\n";
	buildMenuCmd += "		for($j = 0; $j < `size($history)`; $j++){\n";
	buildMenuCmd += "			if(`nodeType($history[$j])` == \"stretchMesh\"){\n";
	buildMenuCmd += "				$stretchMesh = $history[$j];\n";
	buildMenuCmd += "				$stretchMeshPoly = `substitute \".vtx\\\\[.*\\\\]\" $sel[$i] \"\"`;\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	
	buildMenuCmd += "	if($stretchMesh == \"\"){\n";
	buildMenuCmd += "		error \"Select vertices on a mesh that has a stretchMeshCmd deformer\";\n";
	buildMenuCmd += "		return;\n";
	buildMenuCmd += "	}\n";
	
	// Temporarily disable the stretchMeshCmd so it isn't evaluating as 
	// as we go
	buildMenuCmd += "	setAttr (($stretchMesh + \".nodeState\"), 1);\n";
	
	// First, we need to find out if there is a curve selected. If not, we need to exit with an error message
	buildMenuCmd += "	string $curve = \"\";\n";
	buildMenuCmd += "	for($i = 0; $i < `size($sel)`; $i++){\n";
	// Next, determine the shape node of the selected curve
	buildMenuCmd += "		string $history[] = `listRelatives($sel[$i])`;\n";
	buildMenuCmd += "		int $hist;\n";
	buildMenuCmd += "		for($hist = 0; $hist < `size($history)`; $hist++){\n";
	// collision object is a polymesh...
	buildMenuCmd += "			if(`nodeType($history[$hist])` == \"nurbsCurve\"){\n";
	buildMenuCmd += "				$curve = $sel[$i];\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	
	// Create attractor curve and make connections
	buildMenuCmd += "	if ($curve == \"\"){\n";
	buildMenuCmd += "		error \"You must select a curve and some StretchMesh vertices before running this command\";\n";
	buildMenuCmd += "		return;\n";
	buildMenuCmd += "	}\n";

	// We search the existing curve attractor array elements for one that is not connected. 
	// if we find it, we connect our new attractor to that element. 
	// otherwise, we connect it at a new element
	buildMenuCmd += "	int $numAttrctrs = `getAttr -size ($stretchMesh + \".crvAttrctrCurve\")`;\n";
	buildMenuCmd += "	int $emptyElement = $numAttrctrs;\n";
	buildMenuCmd += "	for($i = 0; $i < $numAttrctrs; $i++){\n";
	buildMenuCmd += "		if(size(`listConnections ($stretchMesh + \".crvAttrctrCurve[\" + $i + \"]\")`) == 0){\n";
	buildMenuCmd += "			$emptyElement = $i;\n";
	buildMenuCmd += "			break;\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";

	// Call the stretchMesh command to build the SM connections, and calculate the closest UV points to the curve
	buildMenuCmd += "stretchMesh -edit -addCurveAttractor $curve $stretchMesh;\n";
		
	//We need to initialize the attractor mults to zero, in case any previously created/deleted attractors have old values in there
	buildMenuCmd += "	int $numVerts[];\n";
	buildMenuCmd += "	for($i = 0; $i < `size($sel)`; $i++){\n";
	buildMenuCmd += "		if (`nodeType($sel[$i])` == \"mesh\"){\n";
	buildMenuCmd += "			$numVerts = `polyEvaluate -vertex $sel[$i]`;\n";
	buildMenuCmd += "			break;\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	buildMenuCmd += "	for($i = 0; $i < $numVerts[0]; $i++){\n";
	buildMenuCmd += "		setAttr(($stretchMesh + \".crvAttrctrVrtMultList[\" + $i + \"].crvAttrctrVrtMult[\" + $emptyElement + \"]\"), 0.0);\n";	
	buildMenuCmd += "	}\n";
	
	buildMenuCmd += "	for($i = 0; $i < `size($sel)`; $i++){\n";
	buildMenuCmd += "		if (`nodeType($sel[$i])` == \"transform\"){\n";
	buildMenuCmd += "			continue;\n";
	buildMenuCmd += "		}\n";
	// The \[ may be an issue for linux...maybe just assume [ and ]...or figure out what's with linux! heh
	buildMenuCmd += "		string $buf[];\n";
	buildMenuCmd += "		int $s = `tokenize $sel[$i] \"[]\" $buf`;\n";
	buildMenuCmd += "		string $match = $buf[$s-1];\n";
	//buildMenuCmd += "		string $match = `match \"\[[0-9]+\]\" $sel[$i]`;\n";
	//buildMenuCmd += "		$match = `substitute \"\\\\[\" $match \"\"`;\n";
	//buildMenuCmd += "		$match = `substitute \"\\\\]\" $match \"\"`;\n";
	
	buildMenuCmd += "		if ($match != \"\"){\n";
	buildMenuCmd += "			int $vert_id = $match;\n";
	buildMenuCmd += "			setAttr(($stretchMesh + \".crvAttrctrVrtMultList[\" + $vert_id + \"].crvAttrctrVrtMult[\" + $emptyElement + \"]\"), 1.0);\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";
	
	// Enable the stretchMeshCmd now that we're done
	buildMenuCmd += "	setAttr (($stretchMesh + \".nodeState\"), 0);\n";
	buildMenuCmd += "}\n";
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// End Add curve attractor to selected verts
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Helper function that returns the center of the selection list (passed in as argument)
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd += "	proc float[] getSelectionCenter(string $selection[]){\n";
	buildMenuCmd += "		int $numSelected = size($selection);\n";
	buildMenuCmd += "		float $translate[] = `xform -q -t -ws`;\n";
	buildMenuCmd += "		float $selectionCenter[] = {0, 0, 0};\n";
	buildMenuCmd += "		int $i;\n";

	buildMenuCmd += "		for ($i = 0; $i < $numSelected; $i++){\n";
	buildMenuCmd += "			$selectionCenter[0] = $selectionCenter[0] + $translate[$i*3];\n";
	buildMenuCmd += "			$selectionCenter[1] = $selectionCenter[1] + $translate[($i*3)+1];\n";
	buildMenuCmd += "			$selectionCenter[2] = $selectionCenter[2] + $translate[($i*3)+2];\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "		$selectionCenter[0] = $selectionCenter[0]/$numSelected;\n";
	buildMenuCmd += "		$selectionCenter[1] = $selectionCenter[1]/$numSelected;\n";
	buildMenuCmd += "		$selectionCenter[2] = $selectionCenter[2]/$numSelected;\n";

	buildMenuCmd += "		return $selectionCenter;\n";
	buildMenuCmd += "	}\n";

#ifndef MAYA_2011
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Paint per vertex attractor mults
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd += "	proc string artSkinHoldSubstring(string $infl)\n";
//
// Return the influence name with the \\" (Hold)\\" removed.
//
	buildMenuCmd += "	{\n";
	buildMenuCmd += "		string	$subInf = $infl;	\n";	
	buildMenuCmd += "		int 	$sizeInf = size($infl);\n";	
	buildMenuCmd += "		if ( $sizeInf > 7 ) {\n";
	buildMenuCmd += "			string $hasHold = substring( $infl, ($sizeInf-6), $sizeInf );\n";
	buildMenuCmd += "			if ($hasHold == (\" (Hold)\")) {\n";
	buildMenuCmd += "				$subInf = substring( $infl, 1, $sizeInf-7 );\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "		return $subInf;\n";
	buildMenuCmd += "	}\n";

	buildMenuCmd += "	global proc string [] artSkinFindSkinClusterNodes()\n";
//
// Return an array with currently active skin cluster nodes.
//
	buildMenuCmd += "	{\n";
	buildMenuCmd += "		string $cmd = \"artAttrSkinPaintCtx -q -objattrArray \" + `currentCtx`;\n";
	buildMenuCmd += "		string $paintClusters = `eval $cmd`;\n";

	buildMenuCmd += "		string $buffer[];\n";
	buildMenuCmd += "		tokenize( $paintClusters, \" \", $buffer );	\n";

	buildMenuCmd += "		string $skinClusters[];\n";
	buildMenuCmd += "		int    $skinIdx = 0;\n";
	buildMenuCmd += "		for ( $item in $buffer ) {\n";
	buildMenuCmd += "			string $itemElems[];\n";
	buildMenuCmd += "			tokenize( $item, \".\", $itemElems );\n";
	buildMenuCmd += "			int $nbElem = size($itemElems);\n";
	buildMenuCmd += "			if ( ( $nbElem < 3 ) || ( $itemElems[2] != \"paintWeights\") ) \n";
	buildMenuCmd += "				continue; \n";

		// We have the skinCluster node.
	buildMenuCmd += "			$skinClusters[$skinIdx] = $itemElems[1];\n";
	buildMenuCmd += "			$skinIdx ++;\n";
	buildMenuCmd += "		}\n";

	buildMenuCmd += "		return $skinClusters;\n";
	buildMenuCmd += "	}\n";

	buildMenuCmd += "	global proc string[] artGetSkinInfluenceObjects(string $inSkinClusterName)\n";
//
// Return array of influence objects of a skinCluster-alike node
// Handles StretchMesh nodes as well.
//
	buildMenuCmd += "	{\n";
	// For normal skinCluster nodes, do old behavior
	buildMenuCmd += "		if (nodeType($inSkinClusterName) == \"skinCluster\")\n";
	buildMenuCmd += "		{\n";
	buildMenuCmd += "			return `skinCluster -q -inf $inSkinClusterName`;\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "		else\n";
	buildMenuCmd += "		{\n";
		// Not of skinCluster type. Just return \"matrix\" connections
	
	buildMenuCmd += "			string $stringResult[];\n";
	buildMenuCmd += "			if(`currentCtx` == \"artAttrAttractorContext\"){\n";
	buildMenuCmd += "				string $crvAttractors[] = `listConnections ($inSkinClusterName + \".crvAttrctrCurve\")`;\n";
	buildMenuCmd += "				string $attractors[] = `listConnections ($inSkinClusterName + \".attrctrStrength\")`;\n";
	buildMenuCmd += "				$stringResult = stringArrayCatenate($crvAttractors, $attractors);\n";
	buildMenuCmd += "			}else if(`currentCtx` == \"artAttrColliderContext\"){\n";
	buildMenuCmd += "				string $mshColliders[] = `listConnections ($inSkinClusterName + \".mshCollider\")`;\n";
	buildMenuCmd += "				string $crvColliders[] = `listConnections ($inSkinClusterName + \".primCrvCollider\")`;\n";
	buildMenuCmd += "				string $sphrColliders[] = `listConnections ($inSkinClusterName + \".primSphrCollider\")`;\n";
	buildMenuCmd += "				$stringResult = stringArrayCatenate($mshColliders, $crvColliders);\n";
	buildMenuCmd += "				$stringResult = stringArrayCatenate($stringResult, $sphrColliders);\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "			return $stringResult;\n";
	
//	buildMenuCmd += "			return `listConnections ($inSkinClusterName + \".crvAttrctrCurve\")`;\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";

	buildMenuCmd += "	global proc string artSkinInflNameFromShortName(\n";
	buildMenuCmd += "		string $shortName \n";
	buildMenuCmd += "	)\n";
// 
// Returns a long name of the influence object.
	buildMenuCmd += "	{\n";
	buildMenuCmd += "		string $hasHold = \"\";\n";
	buildMenuCmd += "		int $sizeInf = size($shortName);\n";
	buildMenuCmd += "		if ($sizeInf > 7) {\n";
	buildMenuCmd += "			string $hasHold = substring($shortName,($sizeInf-6),$sizeInf);\n";
	buildMenuCmd += "			if ($hasHold == (\" (Hold)\")) {\n";
	buildMenuCmd += "				$shortName = substring($shortName,1,$sizeInf-7);\n";
	buildMenuCmd += "			} else {\n";
	buildMenuCmd += "				$hasHold = \"\";\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";

	buildMenuCmd += "		string $buffer[];\n";
	buildMenuCmd += "		int $numTokens = tokenize($shortName,\"|\",$buffer);\n";
	buildMenuCmd += "		if ($numTokens == 1) {\n";
	buildMenuCmd += "			string $endName = $buffer[$numTokens-1];\n";

	buildMenuCmd += "			string $clusters[] = artSkinFindSkinClusterNodes();\n";
	buildMenuCmd += "			for ($cluster in $clusters) {\n";
	buildMenuCmd += "				string $infs[] = artGetSkinInfluenceObjects($cluster);\n";
	buildMenuCmd += "				string $inf;\n";
	buildMenuCmd += "				for ($inf in $infs) {\n";
	buildMenuCmd += "					if ($inf == $shortName) {\n";
	buildMenuCmd += "						return ($inf+$hasHold);\n";
	buildMenuCmd += "					}\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "				for ($inf in $infs) {\n";
	buildMenuCmd += "					clear($buffer);\n";
	buildMenuCmd += "					$numTokens = tokenize($inf,\"|\",$buffer);\n";
	buildMenuCmd += "					if ($buffer[$numTokens-1] == $endName) {\n";
	buildMenuCmd += "						return ($inf+$hasHold);\n";
	buildMenuCmd += "					}\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";

	buildMenuCmd += "		$shortName += $hasHold;\n";
	buildMenuCmd += "		return $shortName;\n";
	buildMenuCmd += "	}\n";


	buildMenuCmd += "	global proc string artAttrSkinShortName(\n";
	buildMenuCmd += "		string 	$name\n";
	buildMenuCmd += "	)\n";
//
//	Returns the name that represents a given joint in the artisan
//  textScrollListUI.
//
	buildMenuCmd += "	{\n";
	buildMenuCmd += "		if ( !`textScrollList -q -ex skinClusterInflList`)\n";
	buildMenuCmd += "			return $name;\n";


	// Make sure this is a unique name - if the whole name is
	// on the art manu, then either this name is already short,
	// or we could not make it unique.
	buildMenuCmd += "		string  $menuItems[] = `textScrollList -q -ai skinClusterInflList`;\n";
	buildMenuCmd += "		string  $label;\n";
	buildMenuCmd += "		for ($label in $menuItems ) {\n";

		// Fixup the label (remove \HOLD\)
	buildMenuCmd += "			string $tmp[];\n";
	buildMenuCmd += "			tokenize( $label, \" \", $tmp );\n";

	buildMenuCmd += "			if ( $tmp[0] == $name ) \n";
	buildMenuCmd += "				return $label;\n";
	buildMenuCmd += "		}\n";
		
	buildMenuCmd += "		string 	$shortName = $name;\n";

	buildMenuCmd += "		string	$buffer[];\n";
	buildMenuCmd += "		int 	$numTokens = tokenize( $name, \"|\", $buffer );\n";
	buildMenuCmd += "		if (0 != $numTokens ) {\n";
	buildMenuCmd += "			$shortName = $buffer[$numTokens-1];\n";
	buildMenuCmd += "			if (`getAttr ($name+\".liw\")`) {\n";
	buildMenuCmd += "				string $holdString = (\" (Hold)\");\n";
	buildMenuCmd += "				$shortName += $holdString;\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";

	buildMenuCmd += "		return $shortName;\n";
	buildMenuCmd += "	}\n";


	buildMenuCmd += "	proc string [] artSkinFindInfluenceNodes(\n";
	buildMenuCmd += "		string 	$skinCluster,\n";
	buildMenuCmd += "		string	$inflList[]\n";
	buildMenuCmd += "	)\n";
//
// Return an array with all influence nodes 
// connected to the passed skin cluster node.
//
	buildMenuCmd += "	{\n";
	// Get all the influence objects.
	buildMenuCmd += "		string $connections[] = artGetSkinInfluenceObjects($skinCluster);\n";

	buildMenuCmd += "		string	$infl, $conn;\n";
	buildMenuCmd += "		int 	$numInfls = 0;\n";
	buildMenuCmd += "		for ( $conn in $connections ) {\n";
		// Check and see if the influence is 
		// already in the current list.
	buildMenuCmd += "			int $found = 0;\n";
	buildMenuCmd += "			for( $infl in $inflList ) {\n";
	buildMenuCmd += "				if ( $conn == $infl ) {\n";
	buildMenuCmd += "					$found = 1;		\n";
	buildMenuCmd += "					break;\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";

	buildMenuCmd += "			if ( 0 == $found ) {\n";
	buildMenuCmd += "				$inflList[$numInfls++] = $conn;\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";

	// Now try to make a short name to make the 
	// names easier for the user to read.
	buildMenuCmd += "		string	$buffer[];\n";
	buildMenuCmd += "		string	$inflListShortName[];\n";
	buildMenuCmd += "		int 	$usingShortNames = 0;\n";
	buildMenuCmd += "		for ($ii = 0; $ii < $numInfls; $ii++) {\n";
	buildMenuCmd += "			clear($buffer);\n";
	buildMenuCmd += "			int $numTokens = tokenize($inflList[$ii],\"|\",$buffer);\n";
	buildMenuCmd += "			if (0 == $numTokens ) continue;\n";

	buildMenuCmd += "			$inflListShortName[$ii] = $buffer[$numTokens-1];\n";
	buildMenuCmd += "			$usingShortNames++;\n";
	buildMenuCmd += "		}\n";

	// Deal with names which are now duplicated 
	// because they got shortened - basically copy
	// the long name to resolve it.
	buildMenuCmd += "		int $badIndexList[];\n";
	buildMenuCmd += "		int $badIndexCount = 0;\n";
	buildMenuCmd += "		for ($ii = 0; $ii < $numInfls; $ii++) {\n";
	buildMenuCmd += "			for ($jj=0;$jj<$numInfls;$jj++) {\n";
	buildMenuCmd += "				if ($ii == $jj) continue;\n";
			
			// Check if they are the same.
	buildMenuCmd += "				if ($inflListShortName[$ii] == $inflListShortName[$jj]) {\n";
	buildMenuCmd += "					$badIndexList[$badIndexCount++] = $ii;\n";
	buildMenuCmd += "					break;	\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";

	buildMenuCmd += "		int $badIndex;\n";
	buildMenuCmd += "		for ($badIndex in $badIndexList) {\n";
	buildMenuCmd += "			$inflListShortName[$badIndex] = $inflList[$badIndex];\n";
	buildMenuCmd += "			$usingShortNames--;\n";
	buildMenuCmd += "		}\n";
	
	// Append the word \"Hold\" to the influence 
	// nodes which are in LockWeights mode.
	buildMenuCmd += "		for ($ii = 0; $ii < $numInfls; $ii++) {\n";
	buildMenuCmd += "			if (`attributeQuery -n $inflList[$ii] -ex liw`) {\n";
	buildMenuCmd += "				if (`getAttr ($inflList[$ii]+\".liw\")`) {\n";
	buildMenuCmd += "					string $holdString = (\" (Hold)\");\n";
	buildMenuCmd += "					$inflList[$ii] = ($inflList[$ii] + $holdString);\n";
	buildMenuCmd += "					$inflListShortName[$ii] = ($inflListShortName[$ii] + $holdString);\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";

	// Sort the list of influences if the  
	// optionVar 'sortSkinPaintList' is true.
	buildMenuCmd += "		int $sortList = `optionVar -q sortSkinPaintList`;\n";
	buildMenuCmd += "		if ( 1 == $sortList ) {\n";
	buildMenuCmd += "			string $sortedL[]  = sort($inflListShortName);\n";
	buildMenuCmd += "			$inflListShortName = $sortedL;\n";
	buildMenuCmd += "			if ( $usingShortNames ) {\n";
	buildMenuCmd += "				for ($ii = 0; $ii < $numInfls; $ii++) {\n";
	buildMenuCmd += "					$inflList[$ii] = artSkinInflNameFromShortName($sortedL[$ii]);\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";

	buildMenuCmd += "		return $inflListShortName;\n";
	buildMenuCmd += "	}\n";


///////////////////////////////////////////////////////////////////
// Main procedure - creates a popup menu listing all active joints.
///////////////////////////////////////////////////////////////////

	buildMenuCmd += "	global proc artAttrSkinJointMenu(\n";
	buildMenuCmd += "		string 	$parent,\n";
	buildMenuCmd += "		string 	$artCommand\n";
	buildMenuCmd += "	)\n";
//
//	Description:
// 		Creates a menu that shows all the paintable joints.
// 
	buildMenuCmd += "	{\n";
	buildMenuCmd += "		global string $artSkinCurrentInfluence;\n";

	// Find all the skin cluster nodes.
	buildMenuCmd += "		string $skinClusters[] = artSkinFindSkinClusterNodes();\n";
	buildMenuCmd += "		if ( size($skinClusters) <= 0 ) {\n";
		//cleanup the list if it exists.
	buildMenuCmd += "			if ( `textScrollList -q -ex skinClusterInflList` )\n";
	buildMenuCmd += "				textScrollList -e -ra skinClusterInflList;\n";

	buildMenuCmd += "			return;\n";
	buildMenuCmd += "		}\n";

	buildMenuCmd += "		if (! `textScrollList -q -ex skinClusterInflList`) {\n";
		// If the influence list does not exist, there is no menu to build
		// but we'll still set the current influence if we can find one to
		// set.
		//
	buildMenuCmd += "			if (size($skinClusters) > 0) {\n";
	buildMenuCmd += "				string $infToSet;\n";
	buildMenuCmd += "				int $currentExists = 0;\n";

			// See if the current influence is valid for this skinCluster.
			// If it is, we'll use that. Otherwise, we'll use the first
			// influence of the first skin.
			//
	buildMenuCmd += "				for ($sc in $skinClusters) {\n";
	buildMenuCmd += "					string $infs[] = artGetSkinInfluenceObjects($sc);\n";
	buildMenuCmd += "					for ($inf in $infs) {\n";
	buildMenuCmd += "						if (size($infToSet) == 0) {\n";
						// if the former influence is not found, we'll
						// default to use the first one
						//
	buildMenuCmd += "							$infToSet = $inf;\n";
	buildMenuCmd += "						}\n";
	buildMenuCmd += "						if (size($artSkinCurrentInfluence) == 0) {\n";
						// there is no current influence so just use
						// the first one
						//
	buildMenuCmd += "							source artAttrSkinCallback;\n";
	buildMenuCmd += "							break;\n";
	buildMenuCmd += "						}\n";
	buildMenuCmd += "						if ($inf == $artSkinCurrentInfluence) {\n";
	buildMenuCmd += "							$currentExists = 1;\n";
	buildMenuCmd += "							$infToSet = $artSkinCurrentInfluence;\n";
	buildMenuCmd += "							break;\n";
	buildMenuCmd += "						}\n";
	buildMenuCmd += "					}\n";
	buildMenuCmd += "					if ($currentExists) \n";
	buildMenuCmd += "						break;\n";
	buildMenuCmd += "				}\n";

			// We found an influence to set.
			//
	buildMenuCmd += "				if (size($infToSet) != 0) {\n";
	buildMenuCmd += "					artSkinSelectInfluence($artCommand,$infToSet,$infToSet);\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "			return;\n";
	buildMenuCmd += "		}\n";

	// Clean up the existing list
	buildMenuCmd += "		textScrollList -e -ra skinClusterInflList;\n";
	
	// Create a list of all influence objects names.
	buildMenuCmd += "		string	$inflList[], $inflListShortNames[];\n";
	buildMenuCmd += "		int		$inflIdx = 0;\n";
	buildMenuCmd += "		for ( $sCluster in $skinClusters ) {\n";
	buildMenuCmd += "			if ( $sCluster == \" \")\n";
	buildMenuCmd += "				continue;\n";

		// Find the influence objects (joins) connected
		// to the current skin cluster node.
	buildMenuCmd += "			string	$tmpInflList[];\n";
	buildMenuCmd += "			string	$tmpListShortNames[] = artSkinFindInfluenceNodes($sCluster,$tmpInflList);\n";
	buildMenuCmd += "			int		$nbTmpList = size($tmpInflList);\n";
	buildMenuCmd += "			if ( $nbTmpList <= 0 )\n";
	buildMenuCmd += "				continue;\n";

		// Copy the temp lists.
	buildMenuCmd += "			int $i;\n";
	buildMenuCmd += "			for ( $i = 0; $i < $nbTmpList; $i++ ) {\n";
	buildMenuCmd += "				$inflList[$inflIdx] 		  = $tmpInflList[$i];\n";
	buildMenuCmd += "				$inflListShortNames[$inflIdx] = $tmpListShortNames[$i];\n";
	buildMenuCmd += "				$inflIdx ++;	\n";
	buildMenuCmd += "			} \n";
	buildMenuCmd += "		}\n";


	// =================================
	// Create a popup menu now.
	// =================================
	buildMenuCmd += "		int $numInfls = size($inflList);\n";
	buildMenuCmd += "		int $ii;\n";
	buildMenuCmd += "		for ($ii = 0; $ii < $numInfls; $ii++) {\n";
	buildMenuCmd += "			string $infl = artSkinHoldSubstring($inflList[$ii]);\n";
	buildMenuCmd += "			string $inflShortName = $inflListShortNames[$ii];\n";

	buildMenuCmd += "			textScrollList -e -append $inflShortName\n";
	buildMenuCmd += "				skinClusterInflList;\n";
	buildMenuCmd += "		}\n";

	buildMenuCmd += "		textScrollList -e -sc (\"artSkinSelectInfluence \"+$artCommand+\" \\\"\\\" \\\"\\\"\") skinClusterInflList;\n";

	
	
	// =================================
	// Set the selection 
	// =================================

	// First check if the previously selected influence 
	// object is valid for the selected surfaces and 
	// if that's the case, select it again. Otherwise 
	// use the influence that was last used for the 
	// first of the surfaces.
	buildMenuCmd += "		for ($ii = 0; $ii < $numInfls; $ii++) {\n";
	buildMenuCmd += "			string	$infl 	 = artSkinHoldSubstring($inflList[$ii]);\n";
	buildMenuCmd += "			if ( ( $infl == $artSkinCurrentInfluence ) ||\n";
	buildMenuCmd += "				 ( $inflList[$ii] == $artSkinCurrentInfluence ) )\n";
	buildMenuCmd += "			{\n";
			// Make the connection bewteen the influence 
			// object and the corresponding skin cluster.
	buildMenuCmd += "				artSkinSelectInfluence( $artCommand, $infl, $inflListShortNames[$ii] );\n";
	buildMenuCmd += "				return;\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";

	// Since the selected influence has not been found
	// Find what influence object is currently used 
	// for the first surface and make it the current 
	// object for all the other surfaces too.

	buildMenuCmd += "		string $skinCluster = $skinClusters[0];\n";
	buildMenuCmd += "		if( $skinCluster != \"\" && $skinCluster != \" \" ) {\n";
	buildMenuCmd += "			string $skinClusterPlug = $skinCluster + \".ptt\";\n";
	buildMenuCmd += "			string $connections[]   = `listConnections $skinClusterPlug`;\n";
	buildMenuCmd += "			string $inf;\n";
	buildMenuCmd += "			if ((size($connections) != 0) && ($connections[0] != \"\")) {\n";
	buildMenuCmd += "				$inf = $connections[0];\n";
	buildMenuCmd += "			} else {\n";
			// If no influence was found, just use the first one.
			//
	buildMenuCmd += "				if ($numInfls > 0) {\n";
	buildMenuCmd += "					$inf = $inflList[0];\n";
	buildMenuCmd += "				}\n";
	buildMenuCmd += "			}\n";

	buildMenuCmd += "			if (size($inf) > 0) {\n";
	buildMenuCmd += "				if (`attributeQuery -n $inf -ex liw`) {\n";
	buildMenuCmd += "					if (`getAttr ($inf+\".liw\")`) {\n";
	buildMenuCmd += "						string $holdString = (\" (Hold)\");\n";
	buildMenuCmd += "						$inf = ($inf + $holdString);\n";
	buildMenuCmd += "					}\n";
	buildMenuCmd += "				}\n";

			// Set the selection in the list to that one.
	buildMenuCmd += "				string $shortName = \"\";\n";
	buildMenuCmd += "				for ($ii = 0; $ii < $numInfls; $ii++) {\n";
	buildMenuCmd += "					if ($inf == $inflList[$ii]) {\n";
	buildMenuCmd += "						$shortName = $inflListShortNames[$ii];\n";
	buildMenuCmd += "						break;\n";
	buildMenuCmd += "					}\n";
	buildMenuCmd += "				}\n";

			// Make the connection bewteen the influence 
			// object and the corresponding skin cluster.
	buildMenuCmd += "				artSkinSelectInfluence( $artCommand, $inf, $shortName );\n";
	buildMenuCmd += "				return;\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "	}\n";

#endif
	
//
// Same as artAttrSkinToolScript() but for StretchMesh nodes
//
	buildMenuCmd += "	global proc string artAttrStretchMeshToolScript(int $setToTool, string $whatToPaint)\n";
	buildMenuCmd += "	{\n";
	// Deformer Paint Weight Tool.
//	buildMenuCmd += "		string $tool = \"artAttrAttractorContext\";\n";
	buildMenuCmd += "		string $tool;\n";
	buildMenuCmd += "		if ($whatToPaint == \"attractors\"){\n";
	buildMenuCmd += "			$tool = \"artAttrAttractorContext\";\n";
	buildMenuCmd += "		}else if($whatToPaint == \"colliders\"){\n";
	buildMenuCmd += "			$tool = \"artAttrColliderContext\";\n";
	buildMenuCmd += "		}\n";
							
	buildMenuCmd += "		makePaintable -activateAll false;\n";
	buildMenuCmd += "		makePaintable -activate true \"skinCluster\" \"*\";\n";
	buildMenuCmd += "		makePaintable \"stretchMesh\" \"paintWeights\";\n";

	// Create a tool if it does not exists.
	buildMenuCmd += "		if (! `artAttrSkinPaintCtx -exists $tool` )\n";
	buildMenuCmd += "		{\n";
	buildMenuCmd += "			if($whatToPaint == \"attractors\"){\n";
	buildMenuCmd += "				rememberCtxSettings `artAttrSkinPaintCtx -i1 \"paintSkinWeights.xpm\" -whichTool \"skinWeights\" $tool`;\n";
	buildMenuCmd += "			}else if($whatToPaint == \"colliders\"){\n";
	buildMenuCmd += "				rememberCtxSettings `artAttrSkinPaintCtx -i1 \"paintSkinWeights.xpm\" -whichTool \"skinWeights\" $tool`;\n";
	buildMenuCmd += "			}\n";
	buildMenuCmd += "		}\n";

	buildMenuCmd += "		setToolTo $tool;\n";

	buildMenuCmd += "		if( 3 == $setToTool ) {\n";
	buildMenuCmd += "			toolPropertyWindow;\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "		else if( 4 != $setToTool ) {\n";
	buildMenuCmd += "			warning( \"Wrong input for artAttrStretchMeshToolScript\" );\n";
	buildMenuCmd += "		}\n";
	buildMenuCmd += "		return $tool;\n";
	buildMenuCmd += "	}\n";

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	buildMenuCmd += "stretchMeshCmdMenu();\n";
	
	MGlobal::executeCommand(buildMenuCmd);
	
	return MS::kSuccess;
}

//
// Register command to plugin
//
MStatus stretchMeshCmd::Register(MFnPlugin& ioPlugin)
{
	MStatus status = ioPlugin.registerCommand(MAYA_stretchMeshCMD_NAME, &creator, stretchMeshCmd::newSyntax);

	if (MFAIL(status)) 
		return MReturnStatus(status, "Failed to register " MAYA_stretchMeshCMD_NAME " command");
	else
		Registered = true;

	return status;
}

//
// Deregister command to plugin
//
MStatus stretchMeshCmd::Deregister(MFnPlugin& ioPlugin)
{
	MStatus status = ioPlugin.deregisterCommand(MAYA_stretchMeshCMD_NAME);

	if (MFAIL(status)) 
		return MReturnStatus(status, "Failed to deregister " MAYA_stretchMeshCMD_NAME " command");
	else
		Registered = false;

	return status;
}

