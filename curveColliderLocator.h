#ifndef __CURVECOLLIDERLOCATOR_H__
#define __CURVECOLLIDERLOCATOR_H__
#pragma once

/*
 *  curveColliderLocator.h
 *  stretchMesh
 *
 *
 */

#define MAYA_CURVECOLLIDERLOCATOR_NAME			"curveColliderLocator"

class MFnPlugin;

#include <maya/MIOStream.h>
#include <maya/MPxLocatorNode.h> 

class curveColliderLocator : public MPxLocatorNode
	{
	public:
		curveColliderLocator();
		virtual					~curveColliderLocator(); 
		
		static bool				Registered;
		static MStatus			Register(MFnPlugin& ioPlugin);
		static MStatus			Deregister(MFnPlugin& ioPlugin);
		
		virtual MStatus   		compute(const MPlug& plug, MDataBlock &data);
		
		virtual void				draw(M3dView &view, const MDagPath &path, 
									 M3dView::DisplayStyle style,
									 M3dView::DisplayStatus status);
		
		virtual bool            isBounded() const;
		virtual MBoundingBox    boundingBox() const; 
		
		static  void *          creator();
		static  MStatus         initialize();
		
		static  MObject         colliderRadiusIn;
		static  MObject         colliderCurveIn;
		static  MObject         colliderXform;
		static  MObject         colliderColorR;
		static  MObject         colliderColorG;
		static  MObject         colliderColorB;
		static  MObject         colliderTransparency;

	public: 
		static	MTypeId			id;
	};

#endif