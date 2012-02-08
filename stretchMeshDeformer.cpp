#include "stretchMeshDeformer.h"

#include <string.h>
#include <vector>
#include <maya/MIOStream.h>
#include <math.h>

#include <maya/MItGeometry.h>
#include <maya/MGlobal.h>

#include <maya/MTypeId.h> 
#include <maya/MDagPath.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnNurbsSurface.h>
#include <maya/MDistance.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MMeshIntersector.h>
#include <maya/MTimer.h>

#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MFloatPoint.h>
#include <maya/MFloatPointArray.h>
#include <maya/MMatrix.h>
#include <maya/MMatrixArray.h>

#include <maya/MItMeshVertex.h>
#include <maya/MPxLocatorNode.h> 

#include "CRSpline.h"

#include "ksUtils.h"
#include "IncludeMFnPluginClass.h"


MTypeId     stretchMeshDeformer::id( 0x00113000 );

////////////////////////
// stretchMeshDeformer attributes  //
////////////////////////

MObject     stretchMeshDeformer::stretchMeshVersion;
MObject     stretchMeshDeformer::collisionStep;
MObject     stretchMeshDeformer::iterations;
MObject	  stretchMeshDeformer::collisions;
MObject     stretchMeshDeformer::meanWeightsList;
MObject     stretchMeshDeformer::meanWeights;
MObject     stretchMeshDeformer::connVrtIdList;
MObject     stretchMeshDeformer::connVrtId;
MObject     stretchMeshDeformer::connVrtIdNrmlOrderList;
MObject     stretchMeshDeformer::connVrtIdNrmlOrder;
MObject	  stretchMeshDeformer::enableScaleSafe;
MObject     stretchMeshDeformer::b;
MObject     stretchMeshDeformer::bScalableList;
MObject     stretchMeshDeformer::bScalable;
MObject     stretchMeshDeformer::stiffnessList;
MObject     stretchMeshDeformer::stiffness;
MObject     stretchMeshDeformer::attrWorldMatrixList;
MObject     stretchMeshDeformer::attrctrStrength;
MObject     stretchMeshDeformer::attrctrVrtMultList;
MObject     stretchMeshDeformer::attrctrVrtMult;
MObject     stretchMeshDeformer::attrPaintWeights;
MObject     stretchMeshDeformer::attrPaintTrans;
MObject     stretchMeshDeformer::attrPaintArrDirty;
// curve attractors
MObject		stretchMeshDeformer::crvAttractorCurve;
MObject		stretchMeshDeformer::crvAttractorStrength;
MObject		stretchMeshDeformer::crvAttractorVrtMultList;
MObject		stretchMeshDeformer::crvAttractorVrtMult;
MObject		stretchMeshDeformer::crvAttractorAttachUVList;
MObject		stretchMeshDeformer::crvAttractorAttachUV;
// end curve attractors
MObject     stretchMeshDeformer::mshCollider;
MObject     stretchMeshDeformer::mshColliderPad;
MObject     stretchMeshDeformer::mshColliderInflated;
MObject     stretchMeshDeformer::mshColliderMult;
MObject	  stretchMeshDeformer::mshColliderVrtMultList;
MObject	  stretchMeshDeformer::mshColliderVrtMult;
MObject     stretchMeshDeformer::nrbsCollider;
MObject     stretchMeshDeformer::primSphrColliderList;
MObject     stretchMeshDeformer::primSphrColliderMult;
MObject	  stretchMeshDeformer::primSphrColliderVrtMultList;
MObject	  stretchMeshDeformer::primSphrColliderVrtMult;
MObject     stretchMeshDeformer::primCrvColliderList;
MObject     stretchMeshDeformer::primCrvColliderMult;
MObject	  stretchMeshDeformer::crvColliderVrtMultList;
MObject	  stretchMeshDeformer::crvColliderVrtMult;
MObject	  stretchMeshDeformer::crvColliderRadiusList;
MObject	  stretchMeshDeformer::crvColliderRadius;

bool		stretchMeshDeformer::Registered = false;
bool		stretchMeshDeformer::Licensed = false;

using namespace std;

struct smConnectedVerts {
	vector<int> connectedVerts;
};

struct smMeanWeights {
	vector<double> meanWeights;
};

struct smBScalable {
	vector<double> bScalable;
};

struct smAttractorMults {
	vector<double> attractorMults;
};

struct smCrvColliderRadius {
	vector<double> colliderRadius;
};

struct smCrvAttractorUV {
	vector<double> attractorUV;
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// mWeightListCallbackId is necessary for per-vertex mult painting
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Description:
//		constructor
//
stretchMeshDeformer::stretchMeshDeformer() : mWeightListCallbackId(0)
{
}

stretchMeshDeformer::~stretchMeshDeformer()
//
//	Description:
//		destructor
//
{}

void* stretchMeshDeformer::creator()
//
//	Description:
//		create the stretchMeshDeformer
//
{
	return new stretchMeshDeformer();
}

MStatus stretchMeshDeformer::initialize()
//
//	Description:
//		initialize the attributes
//
{
	// local attribute initialization
	//
	MFnNumericAttribute nAttr;
	MFnMatrixAttribute mAttr;
	MFnCompoundAttribute cmpAttr;
	MFnUnitAttribute unitFn;    
	MFnTypedAttribute typedAttr;
	MFnNumericAttribute stfAttr;

	stretchMeshVersion=nAttr.create( "stretchMeshVersion", "smv", MFnNumericData::kDouble );
	nAttr.setDefault(1.0);
	nAttr.setKeyable(false);
	nAttr.setReadable(false);
	nAttr.setStorable(true);
	nAttr.setHidden(true);
	addAttribute( stretchMeshVersion ); 

	collisionStep=nAttr.create( "collisionStep", "collS", MFnNumericData::kInt );
	nAttr.setDefault(1);
	nAttr.setMin(1);
	nAttr.setKeyable(true);
	addAttribute( collisionStep); 
	
	iterations=nAttr.create( "iterations", "itr", MFnNumericData::kInt );
	// iterations is defaulted to 0 so that the deformer effectively doesn't run during the setup process. The stretchMeshDeformer() 
	// python script will set the iterations attribute after the initialization of all the relevant deformer attributes (mean_weights, 
	// conn_vrt_ids and b). 
	nAttr.setDefault(0);
	nAttr.setMin(0);
	nAttr.setKeyable(true);
	addAttribute( iterations); 

	collisions = nAttr.create( "collisions", "coll", MFnNumericData::kBoolean, true );
	nAttr.setKeyable(true);
	addAttribute(collisions);

	meanWeights=nAttr.create( "meanWeights", "mw", MFnNumericData::kDouble );
	nAttr.setDefault(-1.0);
	nAttr.setHidden(true);
	nAttr.setKeyable(false);
	nAttr.setArray(true);
	nAttr.setReadable(true);
	nAttr.setUsesArrayDataBuilder(true); 
	nAttr.setStorable(true);
	addAttribute( meanWeights ); 

	meanWeightsList=cmpAttr.create( "meanWeightsList", "mwl" );
	cmpAttr.addChild(meanWeights);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	cmpAttr.setUsesArrayDataBuilder(true);
	addAttribute( meanWeightsList ); 

	connVrtId=nAttr.create( "connVrtId", "cvid", MFnNumericData::kInt );
	nAttr.setDefault(-1);
	nAttr.setKeyable(false);
	nAttr.setArray(true);
	nAttr.setReadable(true);
	//nAttr.setUsesArrayDataBuilder(true); 
	nAttr.setStorable(true);
	nAttr.setHidden(true);
	addAttribute( connVrtId ); 

	connVrtIdList=cmpAttr.create( "connVrtIdList", "cvidl" );
	cmpAttr.addChild(connVrtId);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	//cmpAttr.setUsesArrayDataBuilder(true);
	cmpAttr.setStorable(true);
	addAttribute( connVrtIdList ); 

	connVrtIdNrmlOrder=nAttr.create( "connVrtIdNrmlOrder", "cvidN", MFnNumericData::kInt );
	nAttr.setDefault(-1);
	nAttr.setKeyable(false);
	nAttr.setArray(true);
	nAttr.setReadable(true);
	//nAttr.setUsesArrayDataBuilder(true); 
	nAttr.setStorable(true);
	nAttr.setHidden(true);
	addAttribute( connVrtIdNrmlOrder ); 

	connVrtIdNrmlOrderList=cmpAttr.create( "connVrtIdNrmlOrderList", "cvidnl" );
	cmpAttr.addChild(connVrtIdNrmlOrder);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	//cmpAttr.setUsesArrayDataBuilder(true);
	cmpAttr.setStorable(true);
	addAttribute( connVrtIdNrmlOrderList ); 
	
	b=nAttr.create( "b", "b", MFnNumericData::kDouble );
	nAttr.setDefault(-1.0);
	nAttr.setKeyable(false);
	nAttr.setArray(true);
	nAttr.setReadable(true);
	nAttr.setUsesArrayDataBuilder(true); 
	nAttr.setStorable(true);
	nAttr.setHidden(true);
	addAttribute( b ); 

	enableScaleSafe = nAttr.create( "enableScaleSafe", "ess", MFnNumericData::kBoolean, false );
	nAttr.setKeyable(true);
//	nAttr.setDefault(0);
	addAttribute(enableScaleSafe);

	bScalable=nAttr.create( "bScalable", "bs", MFnNumericData::kDouble );
	nAttr.setDefault(-1.0);
	nAttr.setHidden(true);
	nAttr.setKeyable(false);
	nAttr.setArray(true);
	nAttr.setReadable(true);
	nAttr.setUsesArrayDataBuilder(true); 
	nAttr.setStorable(true);
	addAttribute( bScalable ); 
	
	bScalableList=cmpAttr.create( "bScalableList", "bsl" );
	cmpAttr.addChild(bScalable);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	cmpAttr.setUsesArrayDataBuilder(true);
	addAttribute( bScalableList ); 
	
	stiffness=stfAttr.create( "stiffness", "stf", MFnNumericData::kFloat, 0.0);
	stfAttr.setHidden(true);
	stfAttr.setStorable(true);
	stfAttr.setArray(true);
	stfAttr.setUsesArrayDataBuilder(true); 
	addAttribute( stiffness );
	
	stiffnessList=cmpAttr.create( "stiffnessList", "stfl" );
	cmpAttr.addChild(stiffness);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	cmpAttr.setUsesArrayDataBuilder(true);
	addAttribute( stiffnessList ); 
	
	attrWorldMatrixList=mAttr.create("matrix", "ma");
	mAttr.setArray(true); 
	mAttr.setReadable(false); 
	mAttr.setKeyable(false); 
	mAttr.setConnectable(true);
	mAttr.setHidden(true);
	addAttribute( attrWorldMatrixList ); 
	
	attrctrStrength=nAttr.create( "attrctrStrength", "aStr", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true); 
	nAttr.setReadable(false); 
	nAttr.setKeyable(false); 
	nAttr.setConnectable(true);
	nAttr.setHidden(true);
	addAttribute( attrctrStrength ); 
	
	attrctrVrtMult=nAttr.create( "attrctrVrtMult", "aVMlt", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true);
	nAttr.setUsesArrayDataBuilder(true);
	nAttr.setHidden(true);
	addAttribute( attrctrVrtMult ); 
	
	attrctrVrtMultList=cmpAttr.create( "attrctrVrtMultList", "aVMltL" );
	cmpAttr.addChild(attrctrVrtMult);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	cmpAttr.setUsesArrayDataBuilder(true);
	addAttribute( attrctrVrtMultList ); 
	
	// Curve attractors
	crvAttractorCurve=typedAttr.create("crvAttrctrCurve", "cACrv", MFnData::kNurbsCurve);
	typedAttr.setArray(true);
	typedAttr.setReadable(true);
	typedAttr.setHidden(true);
	addAttribute( crvAttractorCurve ); 
	
	crvAttractorStrength=nAttr.create( "crvAttrctrStrength", "cAStr", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true); 
	nAttr.setReadable(false); 
	nAttr.setKeyable(false); 
	nAttr.setConnectable(true);
	addAttribute( crvAttractorStrength ); 
	
	crvAttractorVrtMult=nAttr.create( "crvAttrctrVrtMult", "cAVM", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true);
	nAttr.setHidden(true);
	nAttr.setUsesArrayDataBuilder(true);
	addAttribute( crvAttractorVrtMult ); 
	
	crvAttractorVrtMultList=cmpAttr.create( "crvAttrctrVrtMultList", "aVML" );
	cmpAttr.addChild(crvAttractorVrtMult);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	cmpAttr.setUsesArrayDataBuilder(true);
	addAttribute( crvAttractorVrtMultList ); 

	crvAttractorAttachUV=nAttr.create( "crvAttrctrAttchUV", "cAUV", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true);
	nAttr.setUsesArrayDataBuilder(true);
	nAttr.setHidden(true);
	addAttribute( crvAttractorAttachUV ); 
	
	crvAttractorAttachUVList=cmpAttr.create( "crvAttrctrAttchUVList", "cAUVL" );
	cmpAttr.addChild(crvAttractorAttachUV);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	cmpAttr.setUsesArrayDataBuilder(true);
	addAttribute( crvAttractorAttachUVList ); 
	// End curve attractors
	
	// Make sure the <kDoubleArray> attribute has a default value. Behaves weird otherwise.
	MDoubleArray	defaultDoubleArray;
	MFnDoubleArrayData defaultDoubleArrayData;
	MObject defaultDoubleArrayAttr;
	defaultDoubleArrayAttr = defaultDoubleArrayData.create(defaultDoubleArray);

	attrPaintWeights = typedAttr.create("paintWeights", "ptw", MFnDoubleArrayData::kDoubleArray);
	typedAttr.setReadable(false); 
	typedAttr.setKeyable(false); 
	typedAttr.setConnectable(true);
	typedAttr.setDefault(defaultDoubleArrayAttr);
	addAttribute(attrPaintWeights);

	MFnMessageAttribute msgAttr;
	attrPaintTrans = msgAttr.create("paintTrans", "ptt"); 
	msgAttr.setReadable(false); 
	addAttribute(attrPaintTrans);
	attrPaintArrDirty = msgAttr.create("paintArrDirty", "pad");
	msgAttr.setReadable(false); 
	addAttribute(attrPaintArrDirty);

	mshCollider = typedAttr.create( "mshCollider", "mcldr", MFnData::kMesh );
	typedAttr.setArray(true);
	typedAttr.setReadable(true);
	typedAttr.setCached(true);
	addAttribute( mshCollider ); 

	mshColliderPad = nAttr.create( "mshColliderPad", "mcldrp", MFnNumericData::kDouble );
	nAttr.setDefault(0);
	nAttr.setKeyable(true);
	nAttr.setArray(true);
	addAttribute( mshColliderPad ); 

	mshColliderInflated = typedAttr.create( "mshColliderInflated", "mcldri", MFnData::kMesh );
	typedAttr.setArray(true);
	typedAttr.setReadable(true);
	typedAttr.setCached(true);
	typedAttr.setUsesArrayDataBuilder(true); 
	typedAttr.setHidden(true);
	addAttribute( mshColliderInflated ); 

	mshColliderMult=nAttr.create( "mshColliderMult", "mcm", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true); 
	nAttr.setReadable(false); 
	nAttr.setKeyable(false); 
	nAttr.setConnectable(true);
	nAttr.setHidden(true);
	addAttribute( mshColliderMult ); 
	
	mshColliderVrtMult=nAttr.create( "mshColliderVrtMult", "mCVM", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true);
	nAttr.setUsesArrayDataBuilder(true);
	nAttr.setHidden(true);
	addAttribute( mshColliderVrtMult ); 
	
	mshColliderVrtMultList=cmpAttr.create( "mshColliderVrtMultList", "mCVML" );
	cmpAttr.addChild(mshColliderVrtMult);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	cmpAttr.setUsesArrayDataBuilder(true);
	addAttribute( mshColliderVrtMultList ); 

	nrbsCollider = typedAttr.create( "nrbsCollider", "ncldr", MFnData::kNurbsSurface );
	typedAttr.setArray(true);
	typedAttr.setReadable(true);
	typedAttr.setCached(true);
	typedAttr.setHidden(true);
	addAttribute( nrbsCollider ); 
	
	primSphrColliderList = mAttr.create("primSphrCollider", "psc");
	mAttr.setArray(true); 
	mAttr.setReadable(false); 
	mAttr.setKeyable(false); 
	mAttr.setConnectable(true);
	mAttr.setHidden(true);
	addAttribute( primSphrColliderList ); 
	
	primSphrColliderMult=nAttr.create( "primSphrColliderMult", "scm", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true); 
	nAttr.setReadable(false); 
	nAttr.setKeyable(false); 
	nAttr.setConnectable(true);
	nAttr.setHidden(true);
	addAttribute( primSphrColliderMult ); 
	
	primSphrColliderVrtMult=nAttr.create( "primSphrColliderVrtMult", "sCVM", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true);
	nAttr.setUsesArrayDataBuilder(true);
	nAttr.setHidden(true);
	addAttribute( primSphrColliderVrtMult ); 
	
	primSphrColliderVrtMultList=cmpAttr.create( "primSphrColliderVrtMultList", "sCVML" );
	cmpAttr.addChild(primSphrColliderVrtMult);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	cmpAttr.setUsesArrayDataBuilder(true);
	addAttribute( primSphrColliderVrtMultList ); 
	
	primCrvColliderList = typedAttr.create("primCrvCollider", "pcc", MFnData::kNurbsCurve);
	typedAttr.setArray(true);
	typedAttr.setReadable(true);
	typedAttr.setHidden(true);
	addAttribute( primCrvColliderList ); 
	
	primCrvColliderMult=nAttr.create( "crvColliderMult", "ccm", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true); 
	nAttr.setReadable(false); 
	nAttr.setKeyable(false); 
	nAttr.setConnectable(true);
	nAttr.setHidden(true);
	addAttribute( primCrvColliderMult ); 
	
	crvColliderVrtMult=nAttr.create( "crvColliderVrtMult", "cCVM", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true);
	nAttr.setHidden(true);
	nAttr.setUsesArrayDataBuilder(true);
	addAttribute( crvColliderVrtMult ); 
	
	crvColliderVrtMultList=cmpAttr.create( "crvColliderVrtMultList", "cCVML" );
	cmpAttr.addChild(crvColliderVrtMult);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	cmpAttr.setUsesArrayDataBuilder(true);
	addAttribute( crvColliderVrtMultList ); 

	crvColliderRadius=nAttr.create( "crvColliderRadius", "cCR", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true);
	nAttr.setHidden(true);
	nAttr.setUsesArrayDataBuilder(true);
	addAttribute( crvColliderRadius ); 
	
	crvColliderRadiusList=cmpAttr.create( "crvColliderRadiusList", "cCRL" );
	cmpAttr.addChild(crvColliderRadius);
	cmpAttr.setHidden(true);
	cmpAttr.setArray(true);
	cmpAttr.setUsesArrayDataBuilder(true);
	addAttribute( crvColliderRadiusList ); 
	
	// affects
	//
	attributeAffects( stretchMeshDeformer::collisionStep, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::iterations, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::collisions, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::meanWeights, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::connVrtId, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::connVrtIdNrmlOrder, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::b, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::enableScaleSafe, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::bScalable, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::stiffness, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::attrWorldMatrixList, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::attrctrStrength, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::attrctrVrtMult, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::attrPaintWeights, stretchMeshDeformer::outputGeom);
	attributeAffects( stretchMeshDeformer::attrPaintTrans, stretchMeshDeformer::outputGeom);
	attributeAffects( stretchMeshDeformer::attrPaintArrDirty, stretchMeshDeformer::outputGeom);
	attributeAffects( stretchMeshDeformer::crvAttractorCurve, stretchMeshDeformer::outputGeom);
	attributeAffects( stretchMeshDeformer::crvAttractorStrength, stretchMeshDeformer::outputGeom);
	attributeAffects( stretchMeshDeformer::crvAttractorVrtMult, stretchMeshDeformer::outputGeom);
	attributeAffects( stretchMeshDeformer::mshCollider, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::mshColliderPad, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::mshColliderInflated, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::mshColliderMult, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::mshColliderVrtMult, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::mshColliderVrtMultList, stretchMeshDeformer::outputGeom);
	attributeAffects( stretchMeshDeformer::nrbsCollider, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::primSphrColliderList, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::primSphrColliderMult, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::primSphrColliderVrtMult, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::primSphrColliderVrtMultList, stretchMeshDeformer::outputGeom);
	attributeAffects( stretchMeshDeformer::primCrvColliderList, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::primCrvColliderMult, stretchMeshDeformer::outputGeom );
	attributeAffects( stretchMeshDeformer::crvColliderVrtMult, stretchMeshDeformer::outputGeom);
	attributeAffects( stretchMeshDeformer::crvColliderRadius, stretchMeshDeformer::outputGeom);

	return MS::kSuccess;
}

void stretchMeshDeformer::postConstructor()
{
	MPxDeformerNode::postConstructor();
	
	MObject this_mobj		= thisMObject();
	mWeightListCallbackId	= MNodeMessage::addAttributeChangedCallback(this_mobj, sAttrChangedCallback, this);
}

void stretchMeshDeformer::RemoveCallbacks()
{
	if (mWeightListCallbackId!=0)
	{
		MNodeMessage::removeCallback(mWeightListCallbackId);
		mWeightListCallbackId = 0;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


MStatus
stretchMeshDeformer::deform( MDataBlock& block,
				MItGeometry& iter,
				const MMatrix& inMatrix,
				unsigned int multiIndex)
//
// Method: deform
//
// Description:   Deform the point with a stretchMeshDeformer algorithm
//
// Arguments:
//   block		: the datablock of the node
//	 iter		: an iterator for the geometry to be deformed
//   m    		: matrix to transform the point into world space
//	 multiIndex : the index of the geometry that we are deforming
//
//
{
	// If not licensed, pass through
	if ( !Licensed )
		return MStatus::kSuccess;

	// do this if we are using an OpenMP implementation that is not the same as Maya's.
	// Even if it is the same, it does no harm to make this call. testing svn on ubuntu
//	MThreadUtils::syncNumOpenMPThreads();
	
	MStatus status = MS::kSuccess;
	MObject thisNode = thisMObject();
	
	// determine the stretch factor
	//
	MDataHandle stretchMeshVersionData = block.inputValue(stretchMeshVersion,&status);
	McheckErr(status, "Error getting stretchMeshVersion data handle\n");
	double stretchMeshVers = stretchMeshVersionData.asDouble();
	
	MDataHandle collisionStepData = block.inputValue(collisionStep,&status);
	McheckErr(status, "Error getting collisionStep data handle\n");
	int collisionStep = collisionStepData.asInt();
	
	MDataHandle iterationsData = block.inputValue(iterations,&status);
	McheckErr(status, "Error getting iterations data handle\n");
	int iterations = iterationsData.asInt();

	MDataHandle collisionsData;
	collisionsData = block.inputValue(collisions );
	bool doCollisions = collisionsData.asBool();
	
	MDataHandle enableScaleSafeHndl;
	enableScaleSafeHndl = block.inputValue(enableScaleSafe );
	bool doScaleSafe = enableScaleSafeHndl.asBool();

	// Get the output geometry as a polygon mesh.
	// Currently, only poly meshes are supported.
	MStatus outGeomStatus;
	MArrayDataHandle outputData = block.outputArrayValue( outputGeom, &outGeomStatus );
	MDataHandle outMesh = outputData.outputValue( &outGeomStatus );
	MObject meshObj;
	meshObj = outMesh.asMesh();
	// Create a vertex iterator from the output mesh
	MStatus stat;
	MItMeshVertex vertIter(meshObj, &status);
	if( !status ){
		status.perror( "Stretch mesh couldn't initialize vertex iterator for output poly mesh: " );
		MGlobal::displayError( "Stretch mesh couldn't initialize vertex iterator for output poly mesh: " );
		return status;
	}
	
	MArrayDataHandle weightListHndl = block.inputArrayValue(meanWeightsList, &stat);
	if(!stat){stat.perror("weights array handle construction failed\n");}

	MArrayDataHandle connVrtListHndl = block.inputArrayValue(connVrtIdList, &stat);
	if(!stat){stat.perror("connVrtIdList array handle construction failed\n");}

	MArrayDataHandle connVrtNrmlOrderListHndl = block.inputArrayValue(connVrtIdNrmlOrderList, &stat);
	if(!stat){stat.perror("connVrtIdList array handle construction failed\n");}
	
	MArrayDataHandle b_hndl = block.inputArrayValue(b, &stat);
	if(!stat){stat.perror("b array handle construction failed\n");}

	MArrayDataHandle bScalableListHndl = block.inputArrayValue(bScalableList, &stat);
	if(!stat){stat.perror("bScalable array handle construction failed\n");}

	MArrayDataHandle stfListHndl = block.inputArrayValue(stiffnessList, &stat);
	if(!stat){stat.perror("stiffnessList array handle construction failed\n");}

	MArrayDataHandle attrctrXformsHndl = block.inputValue(attrWorldMatrixList, &stat);
	if(!stat){stat.perror("attrctrXformsHndl handle construction failed\n");}

	MArrayDataHandle attrctrStrengthsHndl = block.inputValue(attrctrStrength, &stat);
	if(!stat){stat.perror("attrctrStrengthsHndl handle construction failed\n");}

	MArrayDataHandle aVrtMultListHndl = block.inputArrayValue(attrctrVrtMultList, &stat);
	if(!stat){stat.perror("attrctrVrtMultList handle construction failed\n");}

	MArrayDataHandle attrctrCrvsHndl = block.inputValue(crvAttractorCurve, &stat);
	if(!stat){stat.perror("attrctrCrvsHndl handle construction failed\n");}
	
	MArrayDataHandle crvAttrctrStrengthsHndl = block.inputValue(crvAttractorStrength, &stat);
	if(!stat){stat.perror("crvAttrctrStrengthsHndl handle construction failed\n");}
	
	MArrayDataHandle cAVrtMultListHndl = block.inputArrayValue(crvAttractorVrtMultList, &stat);
	if(!stat){stat.perror("attrctrVrtMultList handle construction failed\n");}

	MArrayDataHandle cAttachUVListHndl = block.inputArrayValue(crvAttractorAttachUVList, &stat);
	if(!stat){stat.perror("crvAttractorAttachUV handle construction failed\n");}

	MArrayDataHandle mColliderArrayHandle = block.inputArrayValue(mshCollider, &stat);
	if(!stat){stat.perror("Mesh collider array handle construction failed\n");}

	MArrayDataHandle mColliderPadArrayHandle = block.inputArrayValue(mshColliderPad, &stat);
	if(!stat){stat.perror("Mesh collider pad array handle construction failed\n");}

	MArrayDataHandle mColliderInflatedArrayHandle = block.inputArrayValue(mshColliderInflated, &stat);
	if(!stat){stat.perror("Mesh colliderInflated array handle construction failed\n");}

	MArrayDataHandle mColliderMultHandle = block.inputValue(mshColliderMult, &stat);
	if(!stat){stat.perror("mColliderMultHandle handle construction failed\n");}
	
	MArrayDataHandle mCVrtMultListHndl = block.inputArrayValue(mshColliderVrtMultList, &stat);
	if(!stat){stat.perror("mCVrtMultListHndl handle construction failed\n");}
	
	MArrayDataHandle nColliderArrayHandle = block.inputArrayValue(nrbsCollider, &stat);
	if(!stat){stat.perror("Nurbs collider array handle construction failed\n");}

	MArrayDataHandle pSColliderArrayHandle = block.inputArrayValue(primSphrColliderList, &stat);
	if(!stat){stat.perror("Primitive sphere collider array handle construction failed\n");}
	
	MArrayDataHandle pSColliderMultHandle = block.inputValue(primSphrColliderMult, &stat);
	if(!stat){stat.perror("pSColliderMultHandle handle construction failed\n");}
	
	MArrayDataHandle pSVrtMultListHndl = block.inputArrayValue(primSphrColliderVrtMultList, &stat);
	if(!stat){stat.perror("pSVrtMultListHndl handle construction failed\n");}

	MArrayDataHandle pCColliderArrayHandle = block.inputArrayValue(primCrvColliderList, &stat);
	if(!stat){stat.perror("Primitive curve collider array handle construction failed\n");}

	MArrayDataHandle pCColliderMultHandle = block.inputValue(primCrvColliderMult, &stat);
	if(!stat){stat.perror("pCColliderMultHandle handle construction failed\n");}
	
	MArrayDataHandle cCVrtMultListHndl = block.inputArrayValue(crvColliderVrtMultList, &stat);
	if(!stat){stat.perror("crvColliderVrtMultList handle construction failed\n");}

	MArrayDataHandle cCRadiusListHndl = block.inputArrayValue(crvColliderRadiusList, &stat);
	if(!stat){stat.perror("crvColliderRadiusList handle construction failed\n");}
	
	// determine the envelope (this is a global scale factor)
	//
	MDataHandle envData = block.inputValue(envelope,&status);
	McheckErr(status, "Error getting envelope data handle\n");	
	double envelope = envData.asFloat();	
	if(envelope == 0.0){return MS::kSuccess;}
	MPlug colliderDestPlugArray(thisNode, mshCollider); 
	MPlug attractorStrengthPlugArray(thisNode, attrctrStrength);
	MPlug crvAttractorStrengthPlugArray(thisNode, crvAttractorStrength);
	MDataHandle cAttractorHandle;
	MObject cAttractorObj;
	MObjectArray cAttractorObjArray;
	MDataHandle mColliderInflatedHandle;
	MObject mClldrInfltdObj;
	MPointArray mClldrPts;
	MPointArray clldrMinPts;
	MPointArray clldrMaxPts;
	MVector mClldrNrml;
	MBoundingBox colliderBoundBox;
	MDataHandle nColliderHandle;
	MObject nColliderObj;
	MFnNurbsSurface nColliderFn;
	MPlug clldrDestPlug;
	MPlugArray clldrSrcPlgArray;
	MObject clldrSrcObj;
	MFnDagNode clldrDagNode;
	MDagPath clldrDagPath;
	MMatrix clldrXform;
	MDataHandle pColliderHandle;
	// curve colliders
	MDataHandle cColliderHandle;
	MObject cColliderObj;
	MPlug crvColliderMultPlugArray(thisNode, primCrvColliderMult);
	// mesh colliders
	MDataHandle mColliderHandle;
	MObject mColliderObj;
	MFnMesh mColliderFn;
	MPlug mshColliderMultPlugArray(thisNode, mshColliderMult);	
	// SPhere colliders
	MPlug sphereColliderDestPlugArray(thisNode, primSphrColliderList ); 
	MPlug sphereColliderMultPlugArray(thisNode, primSphrColliderMult);	
	MBoundingBox inMeshBoundBox;
	MVector searchVector;
	MIntArray connected_vrts;
	MVector nrml;
	MPoint pt;
	MPoint conn_pt;
	MPoint conn1;
	MPoint conn2;
	double searchRadius;
	MFloatPoint hitPoint;
	MFloatPointArray hitPoints;
	MPointArray nHitPoints;
	MDoubleArray intersectUArray;
	MDoubleArray intersectVArray;
	MDoubleArray intersectDistances;
	double intersectU;
	double intersectV;
	bool wasExactHit;
	unsigned int numHits;
	MPoint closestPoint;
	MFloatPoint raySource; 
	MVector rayDirVec;
	MFloatVector rayDir;
	MMeshIsectAccelParams intersectAccel;
	MVector conn_pt_t[MAX_NUM_CONN_VRTS];
	MVectorArray vrt_snapshot;
	MVector currVertSnapshot;
	MVector curr_pt_t;
	MVector conn_pt_avg;
	MVector inputToResult;
	MVector vrt_prj_sum;
	MVector prj_edge;
	MIntArray conn_vrts;
	unsigned int num_conn_vrts;
	MPointArray verts;
	MPlug crvColliderRadiusListPlug(thisNode, crvColliderRadiusList);
	
	int conn_vrt_id;
	int prev_vrt_id;

	MPointArray inputPts;  // The point array that represents the initial position of the points.  It is used to interpolate
					    // with the resultPts array depending on env/weight values.
	MPointArray resultPts;  // The point array used throughout the deform method.  
					     // It's the working copy of the output points ultimately used to set the point positions
	inputPts.setLength(iter.count());
	resultPts.setLength(iter.count());

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//                                                                    INITIALIZATION                                                                                     //
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
	// The following for loop initializes several arrays that represent the array attributes of the SM node (stiffness, 
	// mean value weights, connected verts, etc).  This is an optimization, creating MArrayDataHandle objects is 
	// expensive, so we do it just once, here
	// Fill arrays representing the attractor strengths and xforms... optimization so we're not checking it later
	int count = attrctrStrengthsHndl.elementCount();
	// fill an array representing the indices of the connected attractors... optimization
	vector<int> connectedAttractorIds;
	connectedAttractorIds.clear();
	for(int attrctrItr = 0; attrctrItr <  count; attrctrItr++){
		// If this attractor's xform attribute is unconnected, continue to the next attractor
		MPlug attractorStrengthPlug = attractorStrengthPlugArray.elementByPhysicalIndex(attrctrItr);
		if(attractorStrengthPlug.isConnected()){
			connectedAttractorIds.push_back(attractorStrengthPlug.logicalIndex());
		}
	}	
	int lastElementId = -1;
	if(connectedAttractorIds.size() != 0){ lastElementId = connectedAttractorIds.back(); }
	vector<double> attractorStrengthArray;
	attractorStrengthArray.clear();
	MMatrixArray attractorXfmArray;
	attractorXfmArray.clear();
	for (int attrctrItr = 0; attrctrItr <=  lastElementId; attrctrItr++){
		status = attrctrStrengthsHndl.jumpToElement(attrctrItr);
		if(status){
			attractorStrengthArray.push_back(attrctrStrengthsHndl.inputValue().asDouble());
		}else{
			attractorStrengthArray.push_back(0.0);
		}
		
		status = attrctrXformsHndl.jumpToElement(attrctrItr);
		if(status){
			attractorXfmArray.append( attrctrXformsHndl.inputValue().asMatrix());
		}else{
			MMatrix identity;
			attractorXfmArray.append(identity);
		}
	}
	
	// CURVE ATTRACTOR INITIALIZATION
	count = crvAttrctrStrengthsHndl.elementCount();
	// fill an array representing the indices of the connected attractors... optimization
	vector<int> connectedCrvAttractorIds;
	connectedCrvAttractorIds.clear();
	for(int attrctrItr = 0; attrctrItr <  count; attrctrItr++){
		// If this attractor's xform attribute is unconnected, continue to the next attractor
		MPlug crvAttractorStrengthPlug = crvAttractorStrengthPlugArray.elementByPhysicalIndex(attrctrItr);
		if(crvAttractorStrengthPlug.isConnected()){
			connectedCrvAttractorIds.push_back(crvAttractorStrengthPlug.logicalIndex());
		}
	}	
	int lastCrvAttrctrId = -1;
	if(connectedCrvAttractorIds.size() != 0){ lastCrvAttrctrId = connectedCrvAttractorIds.back(); }
	vector<double> crvAttractorStrengthArray;
	crvAttractorStrengthArray.clear();
	for (int attrctrItr = 0; attrctrItr <=  lastCrvAttrctrId; attrctrItr++){
		status = crvAttrctrStrengthsHndl.jumpToElement(attrctrItr);
		if(status){
			crvAttractorStrengthArray.push_back(crvAttrctrStrengthsHndl.inputValue().asDouble());
		}else{
			crvAttractorStrengthArray.push_back(0.0);
		}
	}
	
	cAttractorObjArray.clear();
	for(int attrctrItr = 0; attrctrItr <  connectedCrvAttractorIds.size(); attrctrItr++){
		int attractorId = connectedCrvAttractorIds[attrctrItr];
		attrctrCrvsHndl.jumpToArrayElement(attractorId);
		cAttractorObjArray.append(attrctrCrvsHndl.inputValue().asNurbsCurveTransformed());
	}
	
	// CURVE COLLIDER INITIALIZATION
	count = pCColliderArrayHandle.elementCount();
	// fill an array representing the indices of the connected attractors... optimization
	vector<int> connectedCrvColliderIds;
	connectedCrvColliderIds.clear();
	for(int colliderItr = 0; colliderItr <  count; colliderItr++){
		// If this attractor's mshCollider attribute is unconnected, continue to the next attractor
		MPlug crvColliderPlug = crvColliderMultPlugArray.elementByPhysicalIndex(colliderItr);
		if(crvColliderPlug.isConnected()){
			connectedCrvColliderIds.push_back(crvColliderPlug.logicalIndex());
		}
	}		
	int lastCrvColliderId = -1;
	if(connectedCrvColliderIds.size() != 0){ lastCrvColliderId = connectedCrvColliderIds.back(); }
	vector<double> crvColliderMultArray;
	crvColliderMultArray.clear();
	for (int colliderItr = 0; colliderItr <=  lastCrvColliderId; colliderItr++){
		status = pCColliderMultHandle.jumpToElement(colliderItr);
		if(status){
			crvColliderMultArray.push_back(pCColliderMultHandle.inputValue().asDouble());
		}else{
			crvColliderMultArray.push_back(1.0);
		}
	}
	
	// CURVE COLLIDER RADIUS INITIALIZATION
	MPointArray crvColliderRadiusPts;
	MPoint crvColliderRadiusPt;
	crvColliderRadiusPts.clear();
	vector<MPointArray> crvColliderRadiusPtsArray;
	crvColliderRadiusPts.clear();
	int colliderCount = cCRadiusListHndl.elementCount();
	for(int colliderItr = 0; colliderItr < colliderCount; colliderItr++){
		status = cCRadiusListHndl.jumpToElement(colliderItr);
		if(!status){
			MGlobal::displayError(MString("Could not access curve collider array element"));
			continue;
		}
		MArrayDataHandle cCRadiusHndl = cCRadiusListHndl.inputValue(&status).child(crvColliderRadius);
		int radiusCount = cCRadiusHndl.elementCount();
		
		// iterate through the radius list, storing an array of the connected radius IDs
		MPlug crvColliderRadiusPlug = crvColliderRadiusListPlug.elementByLogicalIndex(colliderItr);
		crvColliderRadiusPlug = crvColliderRadiusPlug.child(0);
		vector<int> connectedRadiusIds;
		for(int radiusPlugItr = 0; radiusPlugItr < crvColliderRadiusPlug.numElements(); radiusPlugItr++){
			if(crvColliderRadiusPlug[radiusPlugItr].isConnected()){
				connectedRadiusIds.push_back(radiusPlugItr);
			}
		}
		
		crvColliderRadiusPts.clear();
		crvColliderRadiusPts.setLength(connectedRadiusIds.size());
		for (int radiusItr = 0; radiusItr < connectedRadiusIds.size(); radiusItr++){
			status = cCRadiusHndl.jumpToElement(connectedRadiusIds[radiusItr]);
			if(!status){
				MGlobal::displayError(MString("Could not access curve collider radius array element"));
				continue;
			}
			crvColliderRadiusPt.x = cCRadiusHndl.inputValue().asDouble(); crvColliderRadiusPt.y = 0.0; crvColliderRadiusPt.z = 0.0; 
			crvColliderRadiusPts[radiusItr] = crvColliderRadiusPt;
		}
		crvColliderRadiusPtsArray.push_back(crvColliderRadiusPts);
	}
		
		
	// MESH COLLIDER INITIALIZATION (for per-vert mult)
	count = mColliderArrayHandle.elementCount();
	// fill an array representing the indices of the connected colliders... optimization
	vector<int> connectedMshColliderIds;
	connectedMshColliderIds.clear();
	for(int colliderItr = 0; colliderItr <  count; colliderItr++){
		// If this colliders's xform attribute is unconnected, continue to the next attractor
		MPlug mshColliderPlug = colliderDestPlugArray.elementByPhysicalIndex(colliderItr);
		if(mshColliderPlug.isConnected()){
			connectedMshColliderIds.push_back(mshColliderPlug.logicalIndex());
		}
	}		
	int lastMshColliderId = -1;
	if(connectedMshColliderIds.size() != 0){ lastMshColliderId = connectedMshColliderIds.back(); }
	vector<double> mshColliderMultArray;
	mshColliderMultArray.clear();
	for (int colliderItr = 0; colliderItr <=  lastMshColliderId; colliderItr++){
		status = mColliderMultHandle.jumpToElement(colliderItr);
		if(status){
			mshColliderMultArray.push_back(mColliderMultHandle.inputValue().asDouble());
		}else{
			mshColliderMultArray.push_back(1.0);
		}
	}
	
	// SHERE COLLIDER INITIALIZATION (for per-vert mult)
	count = pSColliderArrayHandle.elementCount();
	// fill an array representing the indices of the connected sphere colliders... optimization
	vector<int> connectedSphereColliderIds;
	connectedSphereColliderIds.clear();
	for(int colliderItr = 0; colliderItr <  count; colliderItr++){
		// If this colliders's xform attribute is unconnected, continue to the next attractor
		MPlug sphereColliderPlug = sphereColliderDestPlugArray.elementByPhysicalIndex(colliderItr);
		if(sphereColliderPlug.isConnected()){
			connectedSphereColliderIds.push_back(sphereColliderPlug.logicalIndex());
		}
	}		
	int lastSphereColliderId = -1;
	if(connectedSphereColliderIds.size() != 0){ lastSphereColliderId = connectedSphereColliderIds.back(); }
	vector<double> sphereColliderMultArray;
	sphereColliderMultArray.clear();
	for (int colliderItr = 0; colliderItr <=  lastSphereColliderId; colliderItr++){
		status = pSColliderMultHandle.jumpToElement(colliderItr);
		if(status){
			sphereColliderMultArray.push_back(pSColliderMultHandle.inputValue().asDouble());
		}else{
			sphereColliderMultArray.push_back(1.0);
		}
	}

	// Initialize variables containing the connected sphere primitive collider matrices
//	int spherePrimCount = pSColliderArrayHandle.elementCount();
	
	vector<smConnectedVerts> connectedVertArray;
	connectedVertArray.clear();
	smConnectedVerts currVert;
	vector<smConnectedVerts> connectedVertArrayNrmlOrder;
	connectedVertArrayNrmlOrder.clear();
	smConnectedVerts currVertNrmlOrder;
	vector<float> stiffnessArray;
	stiffnessArray.clear();
	vector<double> bArray;
	bArray.clear();

	vector<smBScalable> bScalableArray;
	bScalableArray.clear();
	smBScalable currBScalable;

	vector<smMeanWeights> meanWeightsArray;
	meanWeightsArray.clear();
	smMeanWeights currMeanWeight;

	vector<smAttractorMults> attractorMultsArray;
	attractorMultsArray.clear();
	smAttractorMults currAttractorMult;
	
	vector<smAttractorMults> crvAttractorMultsArray;
	crvAttractorMultsArray.clear();
	smAttractorMults currCrvAttractorMult;

	vector<smAttractorMults> crvColliderMultsArray;
	crvColliderMultsArray.clear();
	smAttractorMults currCrvColliderMult;

	vector<smAttractorMults> mshColliderMultsArray;
	mshColliderMultsArray.clear();
	smAttractorMults currMshColliderMult;
	
	vector<smAttractorMults> sphereColliderMultsArray;
	sphereColliderMultsArray.clear();
	smAttractorMults currSphereColliderMult;
	
	vector<smCrvAttractorUV> crvAttractorUVArray;
	crvAttractorUVArray.clear();
	smCrvAttractorUV currCrvAttractorUV;
	
	for (iter.reset(); !iter.isDone(); iter.next()) {
		int curr_vrt_index = iter.index();
		pt = iter.position(MSpace::kObject);
		pt *= inMatrix; //Put point in world space
		inputPts[iter.index()] = pt;
		resultPts[iter.index()] = pt;
		
		status = stfListHndl.jumpToArrayElement(0);
		if(!status){continue;}
		MArrayDataHandle stfHndl = stfListHndl.inputValue(&status).child(stiffness);
		McheckErr(status, "StiffnessList handle evaluation failed\n");
		status = stfHndl.jumpToArrayElement(curr_vrt_index);
		McheckErr(status, "stfHndl handle evaluation failed\n");
		stiffnessArray.push_back(stfHndl.inputValue(&status).asFloat());
		McheckErr(status, "Error getting stiffness float value\n");
		
		status = b_hndl.jumpToElement(curr_vrt_index);
		McheckErr(status, "b_hndl handle evaluation failed\n");
		bArray.push_back(b_hndl.inputValue(&status).asDouble());
		McheckErr(status, "Error getting b value\n");
		
		currVert.connectedVerts.clear();
		status = connVrtListHndl.jumpToArrayElement(curr_vrt_index);
		McheckErr(status, "Jump to array element failed\n");
		MArrayDataHandle connVrtHndl = connVrtListHndl.inputValue(&status).child(connVrtId);
		num_conn_vrts = connVrtHndl.elementCount();
		for(int i = 0; i < num_conn_vrts; i++){
			connVrtHndl.jumpToArrayElement( i );
			currVert.connectedVerts.push_back(connVrtHndl.inputValue().asInt());
		}
		connectedVertArray.push_back(currVert);		
		
		if(stretchMeshVers >= SM_POLAR_FIX){
			currVertNrmlOrder.connectedVerts.clear();
			status = connVrtNrmlOrderListHndl.jumpToArrayElement(curr_vrt_index);
			McheckErr(status, "Jump to array element failed\n");
			MArrayDataHandle connVrtNrmOrderHndl = connVrtNrmlOrderListHndl.inputValue(&status).child(connVrtIdNrmlOrder);
			num_conn_vrts = connVrtNrmOrderHndl.elementCount();
			for(int i = 0; i < num_conn_vrts; i++){
				connVrtNrmOrderHndl.jumpToArrayElement( i );
				currVertNrmlOrder.connectedVerts.push_back(connVrtNrmOrderHndl.inputValue().asInt());
			}
			connectedVertArrayNrmlOrder.push_back(currVertNrmlOrder);
		}
		
		currMeanWeight.meanWeights.clear();
		status = weightListHndl.jumpToArrayElement( curr_vrt_index );
		McheckErr(status, "Jump to mean weight list array element failed\n");
		MArrayDataHandle weightHndl = weightListHndl.inputValue(&status).child(meanWeights);
		num_conn_vrts = weightHndl.elementCount();
		for(int i = 0; i < num_conn_vrts; i++){
			weightHndl.jumpToElement( i );
			currMeanWeight.meanWeights.push_back(weightHndl.inputValue().asDouble());
		}
		meanWeightsArray.push_back(currMeanWeight);		
		
		currBScalable.bScalable.clear();
		// check that the bScalable array has actually been populated (won't be the case with older nodes)
		if(bScalableListHndl.elementCount() > curr_vrt_index){
			status = bScalableListHndl.jumpToArrayElement( curr_vrt_index );
			McheckErr(status, "Jump to bScalable list array element failed\n");
			MArrayDataHandle bScalableHndl = bScalableListHndl.inputValue(&status).child(bScalable);
			num_conn_vrts = bScalableHndl.elementCount();
			for(int i = 0; i < num_conn_vrts; i++){
				bScalableHndl.jumpToElement( i );
				currBScalable.bScalable.push_back(bScalableHndl.inputValue().asDouble());
			}
			bScalableArray.push_back(currBScalable);	
		}
		
		currAttractorMult.attractorMults.clear();
		status = aVrtMultListHndl.jumpToElement(curr_vrt_index);
		if(status){
			MArrayDataHandle attrctrMults = aVrtMultListHndl.inputValue(&status).child(attrctrVrtMult);
			for(int attrctrItr = 0; attrctrItr <=  lastElementId; attrctrItr++){
				status = attrctrMults.jumpToElement(attrctrItr);
				if(!status){
					currAttractorMult.attractorMults.push_back(0.0);
				}else{
					currAttractorMult.attractorMults.push_back(attrctrMults.inputValue().asDouble());
				}
			}
			attractorMultsArray.push_back(currAttractorMult);						
		}else{
			// This is necessary for v1.0... aVrtMultListHndl.jumpToElement() fails if the specific vert
			// wasn't initialized during the attractor creation
			for(int attrctrItr = 0; attrctrItr <=  lastElementId; attrctrItr++){
				currAttractorMult.attractorMults.push_back(0.0);
			}
			attractorMultsArray.push_back(currAttractorMult);		
		}
		
		// CURVE ATTRACTORS 
		currCrvAttractorMult.attractorMults.clear();
		status = cAVrtMultListHndl.jumpToElement(curr_vrt_index);
		if(status){
			MArrayDataHandle crvAttrctrMults = cAVrtMultListHndl.inputValue(&status).child(crvAttractorVrtMult);
			for(int attrctrItr = 0; attrctrItr <=  lastCrvAttrctrId; attrctrItr++){
				status = crvAttrctrMults.jumpToElement(attrctrItr);
				if(!status){
					currCrvAttractorMult.attractorMults.push_back(0.0);
				}else{
					currCrvAttractorMult.attractorMults.push_back(crvAttrctrMults.inputValue().asDouble());
				}
			}
			crvAttractorMultsArray.push_back(currCrvAttractorMult);						
		}else{
			// This is necessary for v1.0... aVrtMultListHndl.jumpToElement() fails if the specific vert
			// wasn't initialized during the attractor creation
			for(int attrctrItr = 0; attrctrItr <=  lastCrvAttrctrId; attrctrItr++){
				currCrvAttractorMult.attractorMults.push_back(0.0);
			}
			crvAttractorMultsArray.push_back(currCrvAttractorMult);		
		}
		// CURVE ATTRACTOR ATTACH UV
		currCrvAttractorUV.attractorUV.clear();
		status = cAttachUVListHndl.jumpToElement(curr_vrt_index);
		if(status){
			MArrayDataHandle cAttachUVHndl = cAttachUVListHndl.inputValue(&status).child(crvAttractorAttachUV);
			for(int attrctrItr = 0; attrctrItr <=  lastCrvAttrctrId; attrctrItr++){
				status = cAttachUVHndl.jumpToElement(attrctrItr);
				if(!status){
					currCrvAttractorUV.attractorUV.push_back(0.0);
				}else{
					currCrvAttractorUV.attractorUV.push_back(cAttachUVHndl.inputValue().asDouble());
				}
			}
			crvAttractorUVArray.push_back(currCrvAttractorUV);						
		}else{
			// This is necessary for v1.0... aVrtMultListHndl.jumpToElement() fails if the specific vert
			// wasn't initialized during the attractor creation
			for(int attrctrItr = 0; attrctrItr <=  lastCrvAttrctrId; attrctrItr++){
				currCrvAttractorUV.attractorUV.push_back(0.0);
			}
			crvAttractorUVArray.push_back(currCrvAttractorUV);		
		}
		// CURVE COLLIDERS
		currCrvColliderMult.attractorMults.clear();
		status = cCVrtMultListHndl.jumpToElement(curr_vrt_index);
		if(status){
			MArrayDataHandle crvColliderMults = cCVrtMultListHndl.inputValue(&status).child(crvColliderVrtMult);
			for(int colliderItr = 0; colliderItr <= pCColliderArrayHandle.elementCount(); colliderItr++){
				status = crvColliderMults.jumpToElement(colliderItr);
				if(!status){
					currCrvColliderMult.attractorMults.push_back(0.0);
				}else{
					currCrvColliderMult.attractorMults.push_back(crvColliderMults.inputValue().asDouble());
				}
			}
			crvColliderMultsArray.push_back(currCrvColliderMult);						
		}else{
			// This is necessary for v1.0... aVrtMultListHndl.jumpToElement() fails if the specific vert
			// wasn't initialized during the attractor creation
			for(int colliderItr = 0; colliderItr <=  pCColliderArrayHandle.elementCount(); colliderItr++){
				currCrvColliderMult.attractorMults.push_back(0.0);
			}
			crvColliderMultsArray.push_back(currCrvColliderMult);		
		}
	
		// MESH COLLIDERS
		currMshColliderMult.attractorMults.clear();
		status = mCVrtMultListHndl.jumpToElement(curr_vrt_index);
		if(status){
			MArrayDataHandle mshColliderMults = mCVrtMultListHndl.inputValue(&status).child(mshColliderVrtMult);
			for(int colliderItr = 0; colliderItr <= mColliderArrayHandle.elementCount(); colliderItr++){
				status = mshColliderMults.jumpToElement(colliderItr);
				if(!status){
					currMshColliderMult.attractorMults.push_back(0.0);
				}else{
					currMshColliderMult.attractorMults.push_back(mshColliderMults.inputValue().asDouble());
				}
			}
			mshColliderMultsArray.push_back(currMshColliderMult);						
		}else{
			// This is necessary for v1.0... aVrtMultListHndl.jumpToElement() fails if the specific vert
			// wasn't initialized during the attractor creation
			for(int colliderItr = 0; colliderItr <=  mColliderArrayHandle.elementCount(); colliderItr++){
				currMshColliderMult.attractorMults.push_back(0.0);
			}
			mshColliderMultsArray.push_back(currMshColliderMult);		
		}
		
		// SPHERE COLLIDERS
		currSphereColliderMult.attractorMults.clear();
		status = pSVrtMultListHndl.jumpToElement(curr_vrt_index);
		if(status){
			MArrayDataHandle sphereColliderMults = pSVrtMultListHndl.inputValue(&status).child(primSphrColliderVrtMult);
			for(int colliderItr = 0; colliderItr <= pSColliderArrayHandle.elementCount(); colliderItr++){
				status = sphereColliderMults.jumpToElement(colliderItr);
				if(!status){
					currSphereColliderMult.attractorMults.push_back(0.0);
				}else{
					currSphereColliderMult.attractorMults.push_back(sphereColliderMults.inputValue().asDouble());
				}
			}
			sphereColliderMultsArray.push_back(currSphereColliderMult);						
		}else{
			// This is necessary for v1.0... aVrtMultListHndl.jumpToElement() fails if the specific vert
			// wasn't initialized during the attractor creation
			for(int colliderItr = 0; colliderItr <=  pSColliderArrayHandle.elementCount(); colliderItr++){
				currSphereColliderMult.attractorMults.push_back(0.0);
			}
			sphereColliderMultsArray.push_back(currSphereColliderMult);		
		}
		
	}
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//                                                  0.5  Inflate Colliders, calculate bounding boxes                                                       //
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
	//Determine the bounding box of the input Geometry.  This will be used for collisions
	//It has to be done here, because we only want to do it once, instead of for each relax 
	//iteration.
	MPoint bBoxMin;	
	MPoint bBoxMax;
	if(mColliderArrayHandle.elementCount() > 0 && doCollisions){
		//Initialize the min/max storage variables to the first point of the output mesh iterator
		iter.reset();
		pt = iter.position(MSpace::kObject);
		pt *= inMatrix;  //Put point in world space
		
		bBoxMin = pt;
		bBoxMax = pt;
		// iterate through the mesh verts to find the min/max bounding box points
		for (iter.reset(); !iter.isDone(); iter.next()) {
			pt = iter.position(MSpace::kObject);
			pt *= inMatrix; //Put point in world space
			if(pt.x < bBoxMin.x){bBoxMin.x = pt.x;} 
			if(pt.y < bBoxMin.y){bBoxMin.y = pt.y;}
			if(pt.z < bBoxMin.z){bBoxMin.z = pt.z;}

			if(pt.x > bBoxMax.x){bBoxMax.x = pt.x;} 
			if(pt.y > bBoxMax.y){bBoxMax.y = pt.y;} 
			if(pt.z > bBoxMax.z){bBoxMax.z = pt.z;} 
		}
		
		// iterate through attractor xforms to expand the min/max bounding box points.
		for(int j = 0; j < attrctrXformsHndl.elementCount(); j++){
			attrctrXformsHndl.jumpToArrayElement(j);
			MMatrix attrctrMat = attrctrXformsHndl.inputValue(&status).asMatrix();
			if(attrctrMat[3][0] < bBoxMin.x){bBoxMin.x = attrctrMat[3][0];} 
			if(attrctrMat[3][1] < bBoxMin.y){bBoxMin.y = attrctrMat[3][1];} 
			if(attrctrMat[3][2] < bBoxMin.z){bBoxMin.z = attrctrMat[3][2];} 
			
			if(attrctrMat[3][0] > bBoxMax.x){bBoxMax.x = attrctrMat[3][0];} 
			if(attrctrMat[3][1] > bBoxMax.y){bBoxMax.y = attrctrMat[3][1];} 
			if(attrctrMat[3][2] > bBoxMax.z){bBoxMax.z = attrctrMat[3][2];} 
		}

		//resize the bounding box
		inMeshBoundBox = MBoundingBox(bBoxMin, bBoxMax);

		//we need to "inflate" each collider mesh according to its pad value (we also calculate each collider's 
		//min and max point, for use when creating bounding boxes.) We store the inflated mesh in a separate
		//attribute because we don't want to inflate the input collider mesh every time this deformer is evaluated.
		clldrMinPts.setLength(mColliderArrayHandle.elementCount());
		clldrMaxPts.setLength(mColliderArrayHandle.elementCount());
		for(int clldrItr = 0; clldrItr < mColliderArrayHandle.elementCount(); clldrItr++){
			//first, make sure this collider attribute is actually connected to something (we don't want to 
			//bother inflating meshes that were once connected, but have been deleted). 
			clldrDestPlug = colliderDestPlugArray.elementByPhysicalIndex(clldrItr);
			if(clldrDestPlug.isConnected()){
				mColliderArrayHandle.jumpToArrayElement(clldrItr);
				mColliderObj = mColliderArrayHandle.inputValue().asMesh(); 
				MItMeshVertex mClldrVrtItr(mColliderObj, &status);
				
				//get the collider pad value, but first check the array length
				double colliderPad = 0;
				if(mColliderPadArrayHandle.elementCount() > clldrItr){
					mColliderPadArrayHandle.jumpToArrayElement(clldrItr);
					colliderPad = mColliderPadArrayHandle.inputValue().asDouble();
				}
				
				MArrayDataBuilder mClldrInfltdBldr = mColliderInflatedArrayHandle.builder();
				mClldrInfltdBldr.addElement(clldrItr);
				mColliderInflatedArrayHandle.set(mClldrInfltdBldr);
				mColliderInflatedArrayHandle.jumpToArrayElement(clldrItr);
				mColliderInflatedHandle = mColliderInflatedArrayHandle.inputValue();
				//Copy the collider mesh from collider attr to colliderInflated attr:
				mColliderInflatedHandle.copy(mColliderArrayHandle.inputValue());
				
				mClldrPts.clear();
				mClldrPts.setLength(mClldrVrtItr.count());
				clldrMinPts[clldrItr] = mClldrVrtItr.position(MSpace::kWorld);
				clldrMaxPts[clldrItr] = mClldrVrtItr.position(MSpace::kWorld);
				for(mClldrVrtItr.reset(); !mClldrVrtItr.isDone(); mClldrVrtItr.next()){
					//store the current point position in MPointArray (we're not modifying the actual points
					//as we go, because when you modify a point you affect neighboring point normals, which would give
					//bad results when we inflate those points.
					mClldrPts[mClldrVrtItr.index()] = mClldrVrtItr.position(MSpace::kWorld);
					
					//add pad to the point in the point snapshot array 
					if(colliderPad != 0){
						mClldrVrtItr.getNormal(mClldrNrml, MSpace::kWorld);
						mClldrNrml.normalize();
						mClldrNrml = mClldrNrml * colliderPad;
						mClldrPts[mClldrVrtItr.index()].x += mClldrNrml.x;
						mClldrPts[mClldrVrtItr.index()].y += mClldrNrml.y;
						mClldrPts[mClldrVrtItr.index()].z += mClldrNrml.z;
					}
					
					//update min and max points (used for creating bounding boxes)
					if(mClldrPts[mClldrVrtItr.index()].x < clldrMinPts[clldrItr].x){clldrMinPts[clldrItr].x = mClldrPts[mClldrVrtItr.index()].x;}
					if(mClldrPts[mClldrVrtItr.index()].y < clldrMinPts[clldrItr].y){clldrMinPts[clldrItr].y = mClldrPts[mClldrVrtItr.index()].y;}
					if(mClldrPts[mClldrVrtItr.index()].z < clldrMinPts[clldrItr].z){clldrMinPts[clldrItr].z = mClldrPts[mClldrVrtItr.index()].z;}
					
					if(mClldrPts[mClldrVrtItr.index()].x > clldrMaxPts[clldrItr].x){clldrMaxPts[clldrItr].x = mClldrPts[mClldrVrtItr.index()].x;}
					if(mClldrPts[mClldrVrtItr.index()].y > clldrMaxPts[clldrItr].y){clldrMaxPts[clldrItr].y = mClldrPts[mClldrVrtItr.index()].y;}
					if(mClldrPts[mClldrVrtItr.index()].z > clldrMaxPts[clldrItr].z){clldrMaxPts[clldrItr].z = mClldrPts[mClldrVrtItr.index()].z;}
				}
				//iterate through the inflated collider points and set positions (see comment above for why we do this)... 
				if(colliderPad != 0){
					mClldrInfltdObj = mColliderInflatedHandle.asMesh();
					MItMeshVertex mClldrInfltdVrtItr(mClldrInfltdObj, &status);
					for(mClldrInfltdVrtItr.reset(); !mClldrInfltdVrtItr.isDone(); mClldrInfltdVrtItr.next()){
						mClldrInfltdVrtItr.setPosition(mClldrPts[mClldrInfltdVrtItr.index()], MSpace::kWorld);
					}
				}
			}
		}
	}

//	MTimer timer; timer.beginTimer();
	// for each relax iteration, iterate through each point in the geometry, set pyramid coords, process attractors,
	// and perform collisions.
	//
	for ( int i = 0; i < iterations; i++){

		// We want to position each vert simultaneously for each iteration. In other words, after we place vert 0, we want vert 1 to 
		// "see" the position of vert 0 before it was placed.  So we take a snapshot of all verts at the beginning of each 
		// iteration
		vrt_snapshot.clear();
		for (iter.reset(); !iter.isDone(); iter.next()) {
			currVertSnapshot.x = resultPts[iter.index()].x;
			currVertSnapshot.y = resultPts[iter.index()].y;
			currVertSnapshot.z = resultPts[iter.index()].z;
			vrt_snapshot.append(currVertSnapshot);
		}

		
		iter.reset();
		// get all points at once. Faster to query, and also better for
		// threading than using iterator
		// -----------------------2008 ---------------------------
		int nPoints = iter.count();
		// ---------------------- 2009+ -----------------------
		
#ifdef _OPENMP
#pragma omp parallel for
#endif		
		for ( int vrtItr = 0; vrtItr < nPoints; vrtItr++) {
			// continue to the next vertex if the weight for this vertex is zero (this is an optimization)
			double weight = weightValue(block, multiIndex, vrtItr);
			if(weight == 0.0){
				continue;
			}
			int curr_vrt_index = vrtItr;
			
			MVector currPtT;
			currPtT.x = resultPts[curr_vrt_index].x;
			currPtT.y = resultPts[curr_vrt_index].y;
			currPtT.z = resultPts[curr_vrt_index].z;
			
			float stiff_curr;
			// Small hack here... for some reason the stiffness values are not written to disk if they are zero, which will cause 
			// the call to inputValue to fail.  So we first check the element count, if it's zero, we assume the stiffness value is 
			// zero.
			if (stfListHndl.elementCount() == 0){
				stiff_curr = 0.0;
			}else{
				stiff_curr = stiffnessArray[curr_vrt_index];
				// If the stiffness is 1, the vertex won't be affected by the pyramid coords, so we can bail out early.
				// This is a speed optimization
				if(stiff_curr == 1.0){
					continue;
				}
			}

			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//                                                   1. Compute the current normal n at v                                                                      //
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
			unsigned int numConnVrts;
			numConnVrts = connectedVertArray[curr_vrt_index].connectedVerts.size();
			
			// populate array containing translation of connected verts and translation of the connected verts, and calculate the avg
			// of the connected verts, the "l" term in the paper
			//
			int j;
			MVector connPtAvg;
			connPtAvg.x = 0.0;
			connPtAvg.y = 0.0;
			connPtAvg.z = 0.0;
			int connVrtId;
			MVector connPtT[MAX_NUM_CONN_VRTS];
			for(j=0; j < numConnVrts; j++){
				connVrtId = connectedVertArray[curr_vrt_index].connectedVerts[j];
				connPtT[j].x = vrt_snapshot[connVrtId].x;
				connPtT[j].y = vrt_snapshot[connVrtId].y;
				connPtT[j].z = vrt_snapshot[connVrtId].z;
				connPtAvg.x = connPtAvg.x + conn_pt.x;
				connPtAvg.y = connPtAvg.y + conn_pt.y;
				connPtAvg.z = connPtAvg.z + conn_pt.z;
			}
			
			connPtAvg.x = connPtAvg.x/numConnVrts;
			connPtAvg.y = connPtAvg.y/numConnVrts;
			connPtAvg.z = connPtAvg.z/numConnVrts;

			MVector currNrml;
			currNrml.x = 0.0;
			currNrml.y = 0.0;
			currNrml.z = 0.0;
			for(j=0; j < numConnVrts; j++){
				// Changes to the deformer were made in order to fix degenerate vertices in certain cases.  
				// Because of this change, we now have to maintain code to compute "legacy" SM nodes (those
				// created before vers SM_POLAR_FIX
				if(stretchMeshVers >= SM_POLAR_FIX){
					connVrtId = connectedVertArrayNrmlOrder[curr_vrt_index].connectedVerts[(j+1)%numConnVrts];
				}else{
					connVrtId = connectedVertArray[curr_vrt_index].connectedVerts[(j+1)%numConnVrts];
				}

				MVector vec1;
				MVector vec2;
				vec1.x = vrt_snapshot[connVrtId].x;
				vec1.y = vrt_snapshot[connVrtId].y;
				vec1.z = vrt_snapshot[connVrtId].z;
				vec1 = vec1 - connPtAvg;
				
				if(stretchMeshVers >= SM_POLAR_FIX){
					connVrtId = connectedVertArrayNrmlOrder[curr_vrt_index].connectedVerts[j];
				}else{
					connVrtId = connectedVertArray[curr_vrt_index].connectedVerts[j];
				}
				vec2.x = vrt_snapshot[connVrtId].x;
				vec2.y = vrt_snapshot[connVrtId].y;
				vec2.z = vrt_snapshot[connVrtId].z;
				vec2 = vec2 - connPtAvg;
				
				currNrml = currNrml + (vec1^vec2);
			}
			currNrml = currNrml/currNrml.length();
			
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//                                                   2. compute the "d" term in the definition of the projection plane                             //
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
			double d;
			d = 0.0;
			for(j=0; j < numConnVrts; j++){
				d = d + currNrml*connPtT[j];
			}
			d = -d/numConnVrts;
			
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//                                                   3. determine v_prime                                                                                              //
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
			MVector v_prime;
			v_prime.x = 0.0;
			v_prime.y = 0.0;
			v_prime.z = 0.0;
			for (j=0; j < numConnVrts; j++){
				double weight_curr;
				weight_curr = meanWeightsArray[curr_vrt_index].meanWeights[j];
				v_prime = v_prime + weight_curr*(connPtT[j] - (d + (connPtT[j]*currNrml))*currNrml);
			}
			
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//                                                   4. Determine v from v_prime                                                                                   //
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			
		
			
			MVector v;
			v.x = 0.0;
			v.y = 0.0;
			v.z = 0.0;
			
			if(doScaleSafe){
				// Calculate v from v_prime using the scalable algorithm ....
				// if the bScalableArray is empty, this must be an old (1.0) sm node, so ScaleSafe mode will be invalid
				// in this case, print a warning and bail out
				if(bScalableArray.size() == 0){
					MGlobal::displayWarning(MString("This mesh does not have scale safe mode data, it must have been created with an older version of stretchMesh.  ")
											+ MString("please recreate the stretchMesh deformer with version 1.5 or later.  Exiting"));
					return MStatus::kFailure;
				}
				
				// Populate an array containing the connected verts projected to the projection plane. 
				MVector conn_pt_prj_t[MAX_NUM_CONN_VRTS];
				for(j=0; j < numConnVrts; j++){
					conn_pt_prj_t[j] = connPtT[j] - (d + (connPtT[j]*currNrml))*currNrml;
				}
				
				MVector vPrimeToV;
				for(j=0; j< numConnVrts; j++){
					MVector connToVPrime = v_prime - conn_pt_prj_t[j];
					vPrimeToV = vPrimeToV + meanWeightsArray[curr_vrt_index].meanWeights[j] * ((connToVPrime.length() * bScalableArray[curr_vrt_index].bScalable[j]) + (connPtT[j] - conn_pt_prj_t[j])*currNrml)*currNrml;
				}
				v = v_prime + vPrimeToV;
			}else{
				//b_hndl.jumpToElement(curr_vrt_index);
				double b_curr;
				b_curr = bArray[curr_vrt_index];
				v = v_prime + b_curr*currNrml;
			}
			
			// The final position of the vertex is a blend between v and the initial position according to the stiffness value.
			MVector v_to_curr;
			v_to_curr = currPtT - v;

			// Uncomment the two lines below to use the deformer's weight value instead of the stiffness
			// attribute.  This allows the user to do attribute painting on the weight value. 
			// stiff_curr = weightValue(block, multiIndex, curr_vrt_index);
			// stiff_curr = 1 - stiff_curr;
			v = v + stiff_curr*v_to_curr;

			MPoint currPt;
			currPt.x = v.x;
			currPt.y = v.y;
			currPt.z = v.z;
			resultPts[curr_vrt_index] = currPt;
		}
		
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//                                                    5. Attractors                                                                                                            //
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// 5. For each vertex, find the connected attractors, determine the average attractor vector using a weighted
		// average of the attractor positions times the attractor strength, then find the final position of the vert by adding 
		// the connected attractor influence vectors.
		//
#ifdef _OPENMP
#pragma omp parallel for
#endif		
		for (  int vrtItr = 0; vrtItr < nPoints; vrtItr++) {			
			int curr_vrt_index = vrtItr;
			
			// Determine the sum of the strength of all connected attractors:
			double attrctrStrengthSum = 0;
			
			for(int attrctrItr = 0; attrctrItr <  connectedAttractorIds.size(); attrctrItr++){
				int attractorId = connectedAttractorIds[attrctrItr];
				attrctrStrengthSum = attrctrStrengthSum + (attractorMultsArray[curr_vrt_index].attractorMults[attractorId] * attractorStrengthArray[attractorId]);
			}
			// Add curve attractor strengths
			for(int attrctrItr = 0; attrctrItr <  connectedCrvAttractorIds.size(); attrctrItr++){
				int attractorId = connectedCrvAttractorIds[attrctrItr];
				attrctrStrengthSum = attrctrStrengthSum + (crvAttractorMultsArray[curr_vrt_index].attractorMults[attractorId] * crvAttractorStrengthArray[attractorId]);
			}
			
			if(attrctrStrengthSum < 1){ 
				attrctrStrengthSum = 1.0;
			}
			
			// Start adding the attractor positions (scaled by the strength value) to the current vert position, 
			// Get the xyz coords of the vertex
			MVector currPtT;
			currPtT.x = resultPts[curr_vrt_index].x;
			currPtT.y = resultPts[curr_vrt_index].y;
			currPtT.z = resultPts[curr_vrt_index].z;
			MVector vec_sum;
			vec_sum.x = 0;
			vec_sum.y = 0;
			vec_sum.z = 0;
			
			// loop through the point attractors
			for(int attrctrItr = 0; attrctrItr <  connectedAttractorIds.size(); attrctrItr++){
				int attractorId = connectedAttractorIds[attrctrItr];
				double strengthMult = attractorMultsArray[curr_vrt_index].attractorMults[attractorId];
				
				double strength = attractorStrengthArray[attractorId];
				strength = strength * strengthMult;
				if(strength == 0.0){continue;}

				double x = attractorXfmArray[attractorId][3][0];
				double y = attractorXfmArray[attractorId][3][1];
				double z = attractorXfmArray[attractorId][3][2];
			
				MVector vec;
				vec.x = x - currPtT.x;
				vec.y = y - currPtT.y;
				vec.z = z - currPtT.z;

				// try and get a more linear response to the muscle. Coming up with vec_scale was
				// a lot of trial and error.  Basically the idea is that you want an attractor with a strength
				// of 0.5 to result in a vertex thfffat is roughly halfway between its starting position and the 
				// attractor.  You should get this behavior regardless of the number of iterations. 
				// vec_scale*vec essentially results in the same length vector for each iteration.
				// first, check that iterations are not zero
				if(iterations != 0){
					double vec_scale;
					vec_scale = ( strength/iterations );
					// prevent divide by zero
					if(vec_scale == 0){continue;}
					if(((1/vec_scale) - i) == 0){continue;}
					vec_scale = 1/((1/vec_scale) - i);
					vec_scale = vec_scale/attrctrStrengthSum;
					// One more thing to try... bias the early iterations more heavily, so that the attractor
					// effect has a chance to progagate further across the surface.  This should really be done
					// as a weighted avg rather than a linear interpolation.
					vec_sum = vec_sum + vec_scale*vec;	
				}
			}
			
			// loop through the curve attractors
			for(int attrctrItr = 0; attrctrItr <  connectedCrvAttractorIds.size(); attrctrItr++){
				int attractorId = connectedCrvAttractorIds[attrctrItr];
				double strengthMult = crvAttractorMultsArray[curr_vrt_index].attractorMults[attractorId];
				
				double strength = crvAttractorStrengthArray[attractorId];
				strength = strength * strengthMult;
				if(strength == 0.0){continue;}
				
//				attrctrCrvsHndl.jumpToArrayElement(attractorId);
//				cAttractorHandle = attrctrCrvsHndl.inputValue();
//				cAttractorObj = cAttractorHandle.asNurbsCurveTransformed();  
				MFnNurbsCurve cAttractorFn;
				status = cAttractorFn.setObject(cAttractorObjArray[attractorId]);
				MPoint closestPt;
				double param;
				double attachUV = crvAttractorUVArray[curr_vrt_index].attractorUV[attractorId];
				status = cAttractorFn.getPointAtParam(attachUV, closestPt, MSpace::kObject);
				//closestPt = cAttractorFn.closestPoint(resultPts[curr_vrt_index], &param, 0.0001, MSpace::kObject, &status);
				if(!status){
					MGlobal::displayInfo("Error...");
					status.perror("couldn't get closest point to curve"); 
					//return status;
				}
				
				double x = closestPt.x;
				double y = closestPt.y;
				double z = closestPt.z;
				
				MVector vec;
				vec.x = x - currPtT.x;
				vec.y = y - currPtT.y;
				vec.z = z - currPtT.z;
				
				// try and get a more linear response to the muscle. Coming up with vec_scale was
				// a lot of trial and error.  Basically the idea is that you want an attractor with a strength
				// of 0.5 to result in a vertex thfffat is roughly halfway between its starting position and the 
				// attractor.  You should get this behavior regardless of the number of iterations. 
				// vec_scale*vec essentially results in the same length vector for each iteration.
				// first, check that iterations are not zero
				if(iterations != 0){
					double vec_scale;
					vec_scale = ( strength/iterations );
					// prevent divide by zero
					if(vec_scale == 0){continue;}
					if(((1/vec_scale) - i) == 0){continue;}
					vec_scale = 1/((1/vec_scale) - i);
					vec_scale = vec_scale/attrctrStrengthSum;
					// One more thing to try... bias the early iterations more heavily, so that the attractor
					// effect has a chance to progagate further across the surface.  This should really be done
					// as a weighted avg rather than a linear interpolation.
					vec_sum = vec_sum + vec_scale*vec;	
				}
			}			
 
			MPoint currPt;
			currPt.x = currPtT.x + vec_sum.x;
			currPt.y = currPtT.y + vec_sum.y;
			currPt.z = currPtT.z + vec_sum.z;
			resultPts[curr_vrt_index] = currPt;
		}
		
	
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//                                                   6. Collisions.                                                                                                            //
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//First we check if the collisions switch is on.  If not, we don't do any collision calculations. Also check the 
		//collisionStep attribute, we only do collisions every <collisionStep> iterations, but we always do collisions
		//on the first and last step (this guarantees that the points end up completely outside the collider, 
		//and not relaxed inside).
		if(doCollisions && ((i % collisionStep == 0) || (i == iterations - 1))){
			// Check to see if vertices are inside of any of the mesh collider objects.  If so, push them out. 
			for (int collider_itr = 0; collider_itr < mColliderInflatedArrayHandle.elementCount(); collider_itr++){
				mColliderInflatedArrayHandle.jumpToArrayElement(collider_itr);
				clldrDestPlug = colliderDestPlugArray.elementByPhysicalIndex(collider_itr);
				//bail out if this plug isn't connected to anything, we don't want to perform collision calcutions
				//on collision objects that were connected and then deleted from the mshCollider attribute. 
				if(!clldrDestPlug.isConnected()){ continue; }

				// Determine the search radius:
				// searchRadius = collider.boundingBox + mesh.boundingBox 
				colliderBoundBox = MBoundingBox(clldrMinPts[collider_itr], clldrMaxPts[collider_itr]);
								
				//if the collider and inMesh bounding boxes don't intersect, we can bail out, 
				//we don't do any collisions in this case. 
				if(!colliderBoundBox.intersects(inMeshBoundBox)){ continue; }
				
				searchVector.x = colliderBoundBox.width(); searchVector.y = colliderBoundBox.height(); searchVector.z = colliderBoundBox.depth();
				searchVector.x += inMeshBoundBox.width(); searchVector.y += inMeshBoundBox.height(); searchVector.z += inMeshBoundBox.depth();				
				searchRadius = searchVector.length(); 

				mColliderHandle = mColliderInflatedArrayHandle.inputValue();
				mColliderObj = mColliderHandle.asMeshTransformed();  
				status = mColliderFn.setObject(mColliderObj);
				MMeshIntersector mIntersector;
				mIntersector.create(mColliderObj);
				McheckErr(status, "Error getting collider MFnMesh");
				intersectAccel = mColliderFn.autoUniformGridParams();
				
				if( !mColliderObj.isNull() ){
					for( vertIter.reset(); !vertIter.isDone(); vertIter.next() ) {
						int curr_vrt_index = vertIter.index();
						//optimization: if the vert mult for this vertex is zero, we can continue
						if(mshColliderMultsArray[curr_vrt_index].attractorMults[collider_itr] == 0){
							continue;
						}
						
						pt = resultPts[curr_vrt_index];
						raySource.x = pt.x; raySource.y = pt.y; raySource.z = pt.z;
						vertIter.getNormal(rayDirVec, MSpace::kWorld);
						rayDirVec.normalize();
						rayDir.x = rayDirVec.x; rayDir.y = rayDirVec.y; rayDir.z = rayDirVec.z;
						// to reduce the occurance of single hits being seen as two hits by allIntersections(), we scale the rayDir
						// by the search radius.  this will affect the tolerance used in combining double hits (see allIntersections() docs)
						rayDir = rayDir*searchRadius;
																	
						// Determine if the point is inside the collider mesh, and get the hit points
						hitPoints.clear();
						mColliderFn.allIntersections(raySource, rayDir, NULL, NULL, false,  MSpace::kWorld, searchRadius, false, &intersectAccel, true, hitPoints, NULL, NULL, NULL, NULL, NULL, 0.000001f, &stat);
						numHits = hitPoints.length();
						
						// If the number of mesh intersections is odd, the point is inside the mesh and needs to be pushed out.
						if((numHits % 2) == 1){
							// Get the closest intersection and set the point to that.
							//mColliderFn.getClosestPoint(pt, closestPoint, MSpace::kWorld);
							MPointOnMesh ptOnMesh;
							mIntersector.getClosestPoint(pt, ptOnMesh);
							closestPoint = ptOnMesh.getPoint();
							// if the vert multiplier for this vert is not 1.0, we need to interpolate between the starting position, and the collision point
							if(mshColliderMultsArray[curr_vrt_index].attractorMults[collider_itr] != 1.0){
								
								// get the vector from the starting position to the collision point
								MVector initialToCollided;
								initialToCollided.x = closestPoint.x - resultPts[curr_vrt_index].x;
								initialToCollided.y = closestPoint.y - resultPts[curr_vrt_index].y;
								initialToCollided.z = closestPoint.z - resultPts[curr_vrt_index].z;
								
								// get the length of the vector
								float distToCollided = initialToCollided.length();
								// normalize the vector
								initialToCollided.normalize();
								// multiply the normalized vector by (the original length*vert multiplier)
								initialToCollided = initialToCollided * (distToCollided * mshColliderMultsArray[curr_vrt_index].attractorMults[collider_itr]);
								closestPoint.x = resultPts[curr_vrt_index].x + initialToCollided.x;
								closestPoint.y = resultPts[curr_vrt_index].y + initialToCollided.y;
								closestPoint.z = resultPts[curr_vrt_index].z + initialToCollided.z;
							}
							resultPts[curr_vrt_index] = closestPoint;
						}
					}
				}
			}

		
			//  Check to see if vertices are inside of any of the nurbs collider objects.  If so, push them out. 
			for (int collider_itr = 0; collider_itr < nColliderArrayHandle.elementCount(); collider_itr++){
				nColliderArrayHandle.jumpToArrayElement(collider_itr);
				nColliderHandle = nColliderArrayHandle.inputValue();
				nColliderObj = nColliderHandle.asNurbsSurface();
				status = nColliderFn.setObject(nColliderObj);

				if( !nColliderObj.isNull() ){
					for( vertIter.reset(); !vertIter.isDone(); vertIter.next() ) {
						pt = resultPts[vertIter.index()];
						vertIter.getNormal(rayDirVec, MSpace::kWorld);
						rayDirVec.normalize();
						
						// Determine if the point is inside the collider surface, and get the hit points
						nHitPoints.clear();
						nColliderFn.intersect(pt, rayDirVec, intersectUArray, intersectVArray, nHitPoints, 1e-6, MSpace::kWorld, false, &intersectDistances, false, &wasExactHit, &stat );
						numHits = nHitPoints.length();
						// If the number of mesh intersections is odd, the point is inside the mesh and needs to be pushed out.
						if((numHits % 2) == 1){
							// Get the closest intersection and set the point to that.
							intersectU = intersectUArray[0];
							intersectV = intersectVArray[0];
							closestPoint = nColliderFn.closestPoint(pt, true, &intersectU, &intersectV, true, 1e-6, MSpace::kWorld, &stat );
							resultPts[vertIter.index()] = closestPoint;
						}
					}
				}
			}
			
			// Process the primitive sphere colliders
			for (int collider_itr = 0; collider_itr < connectedSphereColliderIds.size(); collider_itr++){
				pSColliderArrayHandle.jumpToElement(connectedSphereColliderIds[collider_itr]);
				pColliderHandle = pSColliderArrayHandle.inputValue();
				clldrXform = pColliderHandle.asMatrix();
				MMatrix clldrXformInverse = clldrXform.inverse();
					
				int numVerts = vertIter.count();
#ifdef _OPENMP
#pragma omp parallel for
#endif		
				for( int vrtItr = 0; vrtItr < numVerts; vrtItr++ ) {
					int curr_vrt_index = vrtItr;
					//optimization: if the vert mult for this vertex is zero, we can continue
					if(sphereColliderMultsArray[curr_vrt_index].attractorMults[collider_itr] == 0){
						continue;
					}
					
					// determine the distance between the current vert and the current collider.  If it's less than the radius, push the vert
					// out along the vector from the collider to the vert. 
					MPoint currPoint;
					currPoint = resultPts[vrtItr];
					// need to put the pt in the space of the collider xform:
					currPoint = currPoint*clldrXformInverse;
										
					MVector colliderToPt;
					colliderToPt.x = currPoint.x;
					colliderToPt.y = currPoint.y;
					colliderToPt.z = currPoint.z;
					float distanceToCollider = colliderToPt.length();
					//MVector colliderXAxis = clldrXform[0];
					//float colliderScale = colliderXAxis.length();
					if(distanceToCollider < 1.0){
						colliderToPt.normalize();
						MPoint resultPoint;
						// push the point out, then put it back in world space
						resultPoint.x = colliderToPt.x;
						resultPoint.y = colliderToPt.y;
						resultPoint.z = colliderToPt.z;
						resultPoint = resultPoint*clldrXform;
						
						// if the vert multiplier for this vert is not 1.0, we need to interpolate between the starting position, and the collision point
						if(sphereColliderMultsArray[curr_vrt_index].attractorMults[collider_itr] != 1.0){
							
							// get the vector from the starting position to the collision point
							MVector initialToCollided;
							initialToCollided.x = resultPoint.x - resultPts[curr_vrt_index].x;
							initialToCollided.y = resultPoint.y - resultPts[curr_vrt_index].y;
							initialToCollided.z = resultPoint.z - resultPts[curr_vrt_index].z;
							
							// get the length of the vector
							float distToCollided = initialToCollided.length();
							// normalize the vector
							initialToCollided.normalize();
							// multiply the normalized vector by (the original length*vert multiplier)
							initialToCollided = initialToCollided * (distToCollided * sphereColliderMultsArray[curr_vrt_index].attractorMults[collider_itr]);
							resultPoint.x = resultPts[curr_vrt_index].x + initialToCollided.x;
							resultPoint.y = resultPts[curr_vrt_index].y + initialToCollided.y;
							resultPoint.z = resultPts[curr_vrt_index].z + initialToCollided.z;
						}
						
						resultPts[vrtItr] = resultPoint;
					}
				}
			}
			
			// Process the primitive curve colliders
			for (int collider_itr = 0; collider_itr < connectedCrvColliderIds.size(); collider_itr++){
				pCColliderArrayHandle.jumpToElement(connectedCrvColliderIds[collider_itr]);
				cColliderHandle = pCColliderArrayHandle.inputValue();
				cColliderObj = cColliderHandle.asNurbsCurveTransformed();  

				MFnNurbsCurve cColliderFn(cColliderObj, &status);
				if(!status){MGlobal::displayError(MString("Could not initialize MFnNurbsCurve for curve collider."));}
				
				unsigned numSpans;
				numSpans = cColliderFn.numSpans(&status);
				if(!status){MGlobal::displayError(MString("Could not get number of spans on collider curve."));}
				
				int nPoints = vertIter.count();
//#ifdef _OPENMP
//#pragma omp parallel for
//#endif		
				for( int vrtItr = 0; vrtItr < nPoints; vrtItr++ ) {
					// determine the distance between the current vert and the current collider.  If it's less than the radius, push the vert
					// out along the vector from the collider to the vert. 
					int curr_vrt_index = vrtItr;
					//optimization: if the vert mult for this vertex is zero, we can continue
					if(crvColliderMultsArray[curr_vrt_index].attractorMults[collider_itr] == 0){
						continue;
					}
					MPoint currPt;
					currPt = resultPts[curr_vrt_index];
					MVector colliderToPt;
					double param;
					MPoint closestPt;
					closestPt = cColliderFn.closestPoint(currPt, &param, 0.0001, MSpace::kObject, &status);
					colliderToPt.x = currPt.x - closestPt.x;
					colliderToPt.y = currPt.y - closestPt.y;
					colliderToPt.z = currPt.z - closestPt.z;
					float distanceToCollider = colliderToPt.length();
					int numRadiusPts = crvColliderRadiusPtsArray[collider_itr].length();
					float colliderScale;
					if(numRadiusPts == 0){
						colliderScale = 1.0;
					}else{
						// parameter value ranges from 0 to num spans, we normalize it here so it goes from 0 - 1.0:
						param = param/numSpans;
						crvColliderRadiusPt = getInterpolatedSplinePoint(param, crvColliderRadiusPtsArray[collider_itr]);
						colliderScale = crvColliderRadiusPt.x;
					}
					//colliderScale = crvColliderRadiusArray[collider_itr].colliderRadius[0];
					
					if(distanceToCollider < (colliderScale*crvColliderMultsArray[curr_vrt_index].attractorMults[collider_itr])){
						colliderToPt.normalize();
						colliderToPt = colliderToPt*(colliderScale*crvColliderMultsArray[curr_vrt_index].attractorMults[collider_itr]);
						MPoint collidedPt;
						collidedPt.x = closestPt.x + colliderToPt.x;
						collidedPt.y = closestPt.y + colliderToPt.y;
						collidedPt.z = closestPt.z + colliderToPt.z;
						
						resultPts[curr_vrt_index] = collidedPt;
					}
				}
			}
			

		}

	}
//	timer.endTimer(); 
//	MGlobal::displayInfo(MString("Runtime for threaded loop: ") + timer.elapsedTime());

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//                           7. Set final point positions using envelope and weight attributes to blend                                       //
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (iter.reset(); !iter.isDone(); iter.next()) {
		// scale the result vector by evelope*weight (the if statement is just a speed optimization 
		// so you don't do the matrix math if the envelope/weight values are 1.0)
		double weight = weightValue(block, multiIndex, iter.index());
		if(envelope != 1.0 || weight != 1.0){
			inputToResult.x = resultPts[iter.index()].x - inputPts[iter.index()].x;
			inputToResult.y = resultPts[iter.index()].y - inputPts[iter.index()].y;
			inputToResult.z = resultPts[iter.index()].z - inputPts[iter.index()].z;
			inputToResult = (envelope*weight)*inputToResult;
			pt.x = inputPts[iter.index()].x + inputToResult.x;
			pt.y = inputPts[iter.index()].y + inputToResult.y;
			pt.z = inputPts[iter.index()].z + inputToResult.z;
		}else{
			pt = resultPts[iter.index()];
		}
		pt *= inMatrix.inverse(); //Put point in object space
		resultPts[iter.index()] = pt;
		//------------------------ Maya 2008 -------------------------
		iter.setPosition(pt, MSpace::kObject);
	}
	
	//------------------------ Maya 2009+ -------------------------
//	iter.setAllPositions(resultPts, MSpace::kObject);

	return status;
}

//
// Register deformer to plugin
//
MStatus stretchMeshDeformer::Register(MFnPlugin& ioPlugin, bool pLicensed)
{
	MStatus status = ioPlugin.registerNode(MAYA_stretchMeshDEFORMER_NAME, id, &creator,
		&initialize, MPxNode::kDeformerNode );

	if (MFAIL(status)) 
		return MReturnStatus(status, "Failed to register " MAYA_stretchMeshDEFORMER_NAME " deformer");
	else
	{
		Registered = true;
		Licensed = pLicensed;
	}

	return status;
}

//
// Deregister deformer to plugin
//
MStatus stretchMeshDeformer::Deregister(MFnPlugin& ioPlugin)
{
	MStatus status = ioPlugin.deregisterNode(id);

	if (MFAIL(status)) 
		return MReturnStatus(status, "Failed to deregister " MAYA_stretchMeshDEFORMER_NAME " deformer");
	else
	{
		Registered = false;
		Licensed = false;
	}

	return status;
}


MVector stretchMeshDeformer::project_vrt_to_plane(MVector vrt, MVector normal, double d)
{
	MVector result;
	// To find the closest point on the plane to vrt, first I construct a line going through vrt and 
	// (vrt+normal).  Then I find the intersection between the line and the plane.  I'm sure there
	// is a way to find the closest point directly, but this hack works for now...
	MVector vrt2(vrt.x + normal.x, vrt.y + normal.y, vrt.z + normal.z);
	
	// The line is defined by P = P1 + u(P2-P1), we determine the u that gives us the plane intersection point...
	double numerator = normal.x*vrt.x + normal.y*vrt.y + normal.z*vrt.z + d;
	double denominator = normal.x*(vrt.x-vrt2.x) + normal.y*(vrt.y-vrt2.y) + normal.z*(vrt.z-vrt2.z);
	double u = numerator/denominator;
	result.x = vrt.x + u*(vrt2.x -vrt.x);
	result.y = vrt.y + u*(vrt2.y -vrt.y);
	result.z = vrt.z + u*(vrt2.z -vrt.z);
	
	return result;
}

//
// connectionMade
//
MStatus stretchMeshDeformer::connectionMade(const MPlug& inPlug, const MPlug& inOtherPlug, bool inAsSrc)
{
	MStatus result = MPxDeformerNode::connectionMade(inPlug, inOtherPlug, inAsSrc);
	MObject attractorObj = inOtherPlug.node();
	
	// See if "paintTrans" attribute was (re)connected. If so, update "paintWeights"
	if (!inAsSrc && inPlug==stretchMeshDeformer::attrPaintTrans)
	{
		// Find out which influence objects was activated for weight painting
		uint logicalInfluenceIndex;
		uint influenceType;
		if (MS::kSuccess==FindInfluenceObjectIndex(inOtherPlug.node(), logicalInfluenceIndex, influenceType))
		{
			// Get set of influence weights for connected influence objects
			MDoubleArray influenceWeights;
			GetInfluenceObjectWeights(logicalInfluenceIndex, influenceType, influenceWeights);
			// Update "paintWeights" values. Needs to be done via MFnDoubleArrayData
			MStatus status;
			MFnDoubleArrayData doubleArrayData;
			MObject doubleArrayObj = doubleArrayData.create(influenceWeights, &status);
			CHECK_MSTATUS(status);

			// Get "paintWeights" plug and update it
			MObject thisMObj = thisMObject();
			MPlug paintWeightsPlug(thisMObj, stretchMeshDeformer::attrPaintWeights);
			status = paintWeightsPlug.setValue(doubleArrayObj);
			CHECK_MSTATUS(status);
		}
	}

	return result;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The below is necessary for per-vertex mult painting
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Dispatch to <this>
//
void stretchMeshDeformer::sAttrChangedCallback(MNodeMessage::AttributeMessage inMsg, MPlug& ioPlug, MPlug& ioOtherPlug, void* inUserPtr)
{
	stretchMeshDeformer* self = (stretchMeshDeformer*)inUserPtr;
	self->AttrChangedCallback(inMsg, ioPlug, ioOtherPlug);
}

//
// Called when some attribute changed for our node.
// When weight painting, this assures the weights are copied from "paintWeights"
// to the current influence object's weights. The current influence object
// is the object connected to the "paintTrans" plug.
//
void stretchMeshDeformer::AttrChangedCallback(MNodeMessage::AttributeMessage inMsg, MPlug& ioPlug, MPlug& ioOtherPlug)
{
	if (inMsg & MNodeMessage::kAttributeSet) 
	{
		if (ioPlug==stretchMeshDeformer::attrPaintWeights)
		{
			// Get "paintTrans" plug
			MObject thisMObj = thisMObject();
			MPlug paintTransPlug(thisMObj, stretchMeshDeformer::attrPaintTrans);

			// Painting influence objects
			MPlugArray connections;
			paintTransPlug.connectedTo(connections, true, false);
			
			uint logicalInfluenceIndex;
			uint influenceType;
			if (connections.length()==1 && MS::kSuccess==FindInfluenceObjectIndex(connections[0].node(), logicalInfluenceIndex, influenceType))
			{
				MDoubleArray paintWeights;
				GetPaintWeights(paintWeights);
				SetInfluenceObjectWeights(logicalInfluenceIndex, paintWeights, influenceType);
			}
		}
	}
}

//
// Find logical index of influence object <inInfluenceObjectToFind> to the "matrix" list attribute
//
MStatus stretchMeshDeformer::FindInfluenceObjectIndex(const MObject& inInfluenceObjectToFind, uint& outLogicalIndex, uint& influenceType) const
{
	MObject thisMObj = thisMObject();
	//First, look for the influence object among the curve attractors.
	MPlug influenceList(thisMObj, stretchMeshDeformer::crvAttractorStrength);	// Get "matrix" attribute
	
	MPlugArray connections;
	for (uint i=0, n=influenceList.numElements(); i<n; ++i)
	{
		MPlug influencePlug = influenceList[i];
		influencePlug.connectedTo(connections, true, false);
		if (connections.length()==1 && connections[0].node()==inInfluenceObjectToFind)
		{
			outLogicalIndex = influencePlug.logicalIndex();
			influenceType = 2;
			return MS::kSuccess;
		}
	}

	//If we've gotten here, we didn't find the attractor in the curve/point attractor list, 
	//so we search for it in the curve collider list
	MPlug crvCInfluenceList(thisMObj, stretchMeshDeformer::primCrvColliderMult);	// Get "matrix" attribute
	
	for (uint i=0, n=crvCInfluenceList.numElements(); i<n; ++i)
	{
		MPlug influencePlug = crvCInfluenceList[i];
		influencePlug.connectedTo(connections, true, false);
		if (connections.length()==1 && connections[0].node()==inInfluenceObjectToFind)
		{
			outLogicalIndex = influencePlug.logicalIndex();
			influenceType = 3;
			return MS::kSuccess;
		}
	}
	
	//If we've gotten here, we didn't find the attractor in the curve/point attractor list or the curve collider list, 
	//so we search for it in the mesh collider list
	MPlug mshCInfluenceList(thisMObj, stretchMeshDeformer::mshColliderMult);	// Get "meshColliderMult" attribute
	
	for (uint i=0, n=mshCInfluenceList.numElements(); i<n; ++i)
	{
		MPlug influencePlug = mshCInfluenceList[i];
		influencePlug.connectedTo(connections, true, false);
		if (connections.length()==1 && connections[0].node()==inInfluenceObjectToFind)
		{
			outLogicalIndex = influencePlug.logicalIndex();
			influenceType = 4;
			return MS::kSuccess;
		}
	}
	
	//If we've gotten here, we didn't find the attractor in the curve/point attractor list or the curve collider list, 
	//so we search for it in the point collider list
	MPlug sphereCInfluenceList(thisMObj, stretchMeshDeformer::primSphrColliderMult);	// Get "meshColliderMult" attribute
	for (uint i=0, n=sphereCInfluenceList.numElements(); i<n; ++i)
	{
		MPlug influencePlug = sphereCInfluenceList[i];
		influencePlug.connectedTo(connections, true, false);
		if (connections.length()==1 && connections[0].node()==inInfluenceObjectToFind)
		{
			outLogicalIndex = influencePlug.logicalIndex();
			influenceType = 5;
			return MS::kSuccess;
		}
	}
	
	//If we've gotten here, we didn't find the attractor in the curve attractor list, 
	//so we search for it in the point attractor list
	MPlug ptInfluenceList(thisMObj, stretchMeshDeformer::attrWorldMatrixList);	// Get "matrix" attribute
	
	for (uint i=0, n=ptInfluenceList.numElements(); i<n; ++i)
	{
		MPlug influencePlug = ptInfluenceList[i];
		influencePlug.connectedTo(connections, true, false);
		if (connections.length()==1 && connections[0].node()==inInfluenceObjectToFind)
		{
			outLogicalIndex = influencePlug.logicalIndex();
			influenceType = 1;
			return MS::kSuccess;
		}
	}
	
	
	return MS::kFailure;
}

//
// Get list of weights for a given influence object <inInfluenceObjectIndex>
// List is returned in <outWeights>.
//
void stretchMeshDeformer::GetInfluenceObjectWeights(uint inInfluenceObjectIndex, uint influenceType, MDoubleArray& outWeights) const
{
	MStatus status;

	MObject this_mobj = thisMObject();
	MPlug crvMultList(this_mobj, stretchMeshDeformer::crvAttractorVrtMultList);
	MPlug ptMultList(this_mobj, stretchMeshDeformer::attrctrVrtMultList);
	MPlug crvColliderMultList(this_mobj, stretchMeshDeformer::crvColliderVrtMultList);
	MPlug mshColliderMultList(this_mobj, stretchMeshDeformer::mshColliderVrtMultList);
	MPlug sphereColliderMultList(this_mobj, stretchMeshDeformer::primSphrColliderVrtMultList);
	
	// Clear output. Cannot reserve :(
	outWeights.setLength(0);
	
	double mult;

	// influenceType == 1 for a point attractor, influenceType == 2 for a curve attractor, 
	// influenceType == 3 for a curve collider,  influenceType == 4 for a mesh collider
	// influenceType == 5 for a primitive sphere collider
	if(influenceType == 2){
		uint weightlists_count = crvMultList.numElements();
		for (uint w=0; w<weightlists_count; ++w)
		{
			MPlug vtxMultCmpd = crvMultList.elementByPhysicalIndex(w, &status);
			if (MFAIL(status))
				continue;

			MPlug vtxMultList = vtxMultCmpd.child(stretchMeshDeformer::crvAttractorVrtMult, &status);
			if (MFAIL(status))
				continue;

			// Find weight for logical index <inInfluenceObjectIndex>. 

			// We could be doing 3 things here:
			// 1) Call vtxMultList.getExistingArrayAttributeIndices() and check
			//    whether <inInfluenceObjectIndex> is in the list. BUT THIS DOES NOT SEEM TO WORK!
			// 2) Use the magic MPlug::selectAncestorLogicalIndex() function, which seems
			//    most efficient, but doesn't work as querying unknown logical indices will 
			//    introduce the logical index and return a default value.
			// 3) Loop through all physical elements of <vtxMultList> and check
			//    the logical index of the element plug to be <inInfluenceObjectIndex>.
			//    This works and looks like:
			bool foundInfluenceObj = false;
			mult = 0.0;
			for (uint x=0, nw=vtxMultList.numElements(); x<nw; ++x)
			{			
				MPlug multValuePlug = vtxMultList.elementByPhysicalIndex(x);
				if (multValuePlug.logicalIndex()==inInfluenceObjectIndex)
				{
					foundInfluenceObj = true;
					multValuePlug.getValue(mult);
					break;
				}
			}
				
			// Don't append 0.0 mults
			if (mult==0.0f)
				continue;
			
			// Clear unused logical entries and make room for logical element index
			uint logical_index = vtxMultCmpd.logicalIndex();
			for (uint out_weights_size = outWeights.length(); out_weights_size<=logical_index; ++out_weights_size)
				outWeights.append(0.0);
			
			outWeights[logical_index] = (double)mult;	
		}
	}else if(influenceType == 1){
		uint weightlists_count = ptMultList.numElements();
		for (uint w=0; w<weightlists_count; ++w)
		{			
			// if we get here and foundInfluenceObj is false, we must have a point attractor
			MPlug vtxMultCmpd = ptMultList.elementByPhysicalIndex(w, &status);
			if (MFAIL(status))
				continue;
		
			MPlug vtxMultList = vtxMultCmpd.child(stretchMeshDeformer::attrctrVrtMult, &status);
			if (MFAIL(status))
				continue;
			
			mult = 0.0;
			for (uint x=0, nw=vtxMultList.numElements(); x<nw; ++x)
			{
				MPlug multValuePlug = vtxMultList.elementByPhysicalIndex(x);
				if (multValuePlug.logicalIndex()==inInfluenceObjectIndex)
				{
					multValuePlug.getValue(mult);
					break;
				}
			}

			// Don't append 0.0 mults
			if (mult==0.0f)
				continue;
		
			// Clear unused logical entries and make room for logical element index
			uint logical_index = vtxMultCmpd.logicalIndex();
			for (uint out_weights_size = outWeights.length(); out_weights_size<=logical_index; ++out_weights_size)
				outWeights.append(0.0);

			outWeights[logical_index] = (double)mult;		
		}
	}else if(influenceType == 3){
		uint weightlists_count = crvColliderMultList.numElements();
		for (uint w=0; w<weightlists_count; ++w)
		{
			MPlug vtxMultCmpd = crvColliderMultList.elementByPhysicalIndex(w, &status);
			if (MFAIL(status))
				continue;
			
			MPlug vtxMultList = vtxMultCmpd.child(stretchMeshDeformer::crvColliderVrtMult, &status);
			if (MFAIL(status))
				continue;
			
			bool foundInfluenceObj = false;
			mult = 0.0;
			for (uint x=0, nw=vtxMultList.numElements(); x<nw; ++x)
			{			
				MPlug multValuePlug = vtxMultList.elementByPhysicalIndex(x);
				if (multValuePlug.logicalIndex()==inInfluenceObjectIndex)
				{
					foundInfluenceObj = true;
					multValuePlug.getValue(mult);
					break;
				}
			}
			
			// Don't append 0.0 mults
			if (mult==0.0f)
				continue;
			
			// Clear unused logical entries and make room for logical element index
			uint logical_index = vtxMultCmpd.logicalIndex();
			for (uint out_weights_size = outWeights.length(); out_weights_size<=logical_index; ++out_weights_size)
				outWeights.append(0.0);
			
			outWeights[logical_index] = (double)mult;	
		}
	}else if(influenceType == 4){
		uint weightlists_count = mshColliderMultList.numElements();
		for (uint w=0; w<weightlists_count; ++w)
		{
			MPlug vtxMultCmpd = mshColliderMultList.elementByPhysicalIndex(w, &status);
			if (MFAIL(status))
				continue;
			
			MPlug vtxMultList = vtxMultCmpd.child(stretchMeshDeformer::mshColliderVrtMult, &status);
			if (MFAIL(status))
				continue;
			
			bool foundInfluenceObj = false;
			mult = 0.0;
			for (uint x=0, nw=vtxMultList.numElements(); x<nw; ++x)
			{			
				MPlug multValuePlug = vtxMultList.elementByPhysicalIndex(x);
				if (multValuePlug.logicalIndex()==inInfluenceObjectIndex)
				{
					foundInfluenceObj = true;
					multValuePlug.getValue(mult);
					break;
				}
			}
			
			// Don't append 0.0 mults
			if (mult==0.0f)
				continue;
			
			// Clear unused logical entries and make room for logical element index
			uint logical_index = vtxMultCmpd.logicalIndex();
			for (uint out_weights_size = outWeights.length(); out_weights_size<=logical_index; ++out_weights_size)
				outWeights.append(0.0);
			
			outWeights[logical_index] = (double)mult;	
		}
	}else if(influenceType == 5){
		uint weightlists_count = sphereColliderMultList.numElements();
		for (uint w=0; w<weightlists_count; ++w)
		{
			MPlug vtxMultCmpd = sphereColliderMultList.elementByPhysicalIndex(w, &status);
			if (MFAIL(status))
				continue;
			
			MPlug vtxMultList = vtxMultCmpd.child(stretchMeshDeformer::primSphrColliderVrtMult, &status);
			if (MFAIL(status))
				continue;
			
			bool foundInfluenceObj = false;
			mult = 0.0;
			for (uint x=0, nw=vtxMultList.numElements(); x<nw; ++x)
			{			
				MPlug multValuePlug = vtxMultList.elementByPhysicalIndex(x);
				if (multValuePlug.logicalIndex()==inInfluenceObjectIndex)
				{
					foundInfluenceObj = true;
					multValuePlug.getValue(mult);
					break;
				}
			}
			
			// Don't append 0.0 mults
			if (mult==0.0f)
				continue;
			
			// Clear unused logical entries and make room for logical element index
			uint logical_index = vtxMultCmpd.logicalIndex();
			for (uint out_weights_size = outWeights.length(); out_weights_size<=logical_index; ++out_weights_size)
				outWeights.append(0.0);
			
			outWeights[logical_index] = (double)mult;	
		}
	}
	
}

//
// Set list of weights for a given influence object <inInfluenceObjectIndex>
// Weights are in <inWeights>
//
void stretchMeshDeformer::SetInfluenceObjectWeights(uint inInfluenceObjectIndex, const MDoubleArray& inWeights, uint influenceType) const
{
	MStatus status;

	MObject this_mobj = thisMObject();
	MPlug multList;
	if(influenceType == 2){
		multList = MPlug(this_mobj, stretchMeshDeformer::crvAttractorVrtMultList);
	}else if(influenceType == 1){
		multList = MPlug(this_mobj, stretchMeshDeformer::attrctrVrtMultList);
	}else if(influenceType == 3){
		multList = MPlug(this_mobj, stretchMeshDeformer::crvColliderVrtMultList);
	}else if(influenceType == 4){
		multList = MPlug(this_mobj, stretchMeshDeformer::mshColliderVrtMultList);
	}else if(influenceType == 5){
		multList = MPlug(this_mobj, stretchMeshDeformer::primSphrColliderVrtMultList);
	}
	
	
	for (uint w=0, nw=inWeights.length(); w<nw; ++w)
	{
		MPlug vtxMultCmpd = multList.elementByLogicalIndex(w, &status);
		if (MFAIL(status))
			continue;

		MPlug vtxMultList;
		if(influenceType == 1){
			vtxMultList = vtxMultCmpd.child(stretchMeshDeformer::attrctrVrtMult, &status);
			if (MFAIL(status))
				continue;
		}else if(influenceType == 2){
			vtxMultList = vtxMultCmpd.child(stretchMeshDeformer::crvAttractorVrtMult, &status);
			if (MFAIL(status))
				continue;
		}else if(influenceType == 3){
			vtxMultList = vtxMultCmpd.child(stretchMeshDeformer::crvColliderVrtMult, &status);
			if (MFAIL(status))
				continue;
		}else if(influenceType == 4){
			vtxMultList = vtxMultCmpd.child(stretchMeshDeformer::mshColliderVrtMult, &status);
			if (MFAIL(status))
				continue;
		}else if(influenceType == 5){
			vtxMultList = vtxMultCmpd.child(stretchMeshDeformer::primSphrColliderVrtMult, &status);
			if (MFAIL(status))
				continue;
		}
		
		
		double mult = inWeights[w];

		// We clamp the per vertex mults from 0.0 - 1.0
		if (mult>1.0f)		mult = 1.0f;
		else if (mult<0.0f)	mult = 0.0f;
		
		// Set new mult
		MPlug vtxMultPlug = vtxMultList.elementByLogicalIndex(inInfluenceObjectIndex);
		double oldMult;
		vtxMultPlug.getValue(oldMult);

		// If no change at all, no update needed
		if (oldMult==mult)
			continue;

		// Update painted mult
		vtxMultPlug.setValue(mult);
	}
}

//
// Get current "paintWeights" values as MDoubleArray
//
void stretchMeshDeformer::GetPaintWeights(MDoubleArray& outWeights) const

{
	// Get "paintWeights" plug
	MObject thisMObj = thisMObject();
	MPlug paintWeightsPlug(thisMObj, stretchMeshDeformer::attrPaintWeights);

	// Get array data object
	MStatus status;
	MObject doubleArrayObj;
	status = paintWeightsPlug.getValue(doubleArrayObj);
	CHECK_MSTATUS(status);

	// Update "paintWeights" values. Needs to be done via MFnDoubleArrayData
	MFnDoubleArrayData doubleArrayData(doubleArrayObj, &status);
	CHECK_MSTATUS(status);
	outWeights = doubleArrayData.array(&status);
	CHECK_MSTATUS(status);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The above is necessary for per-vertex mult painting
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
