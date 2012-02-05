#ifndef INCLUDED_MAYAUTILS_H
#define INCLUDED_MAYAUTILS_H
#pragma once

#include <iostream>
#include <maya/MObject.h>
#include <maya/MStatus.h>
class MFloatArray;
class MString;
class MPlug;
class MDGModifier;
class MString;
class MItGeometry;



// Status Checking Macro - MReturnCheckStatus (Debugging tool)
#define MReturnCheckStatus(status,message)	\
	if( MS::kSuccess != status ) {		\
	cerr << message << "\n";		\
	return status;					\
	}

#define MCheckStatus(status,message)	\
	if (MS::kSuccess != status) {		\
	cerr << message << " (" << status.errorString().asChar() << ")\n";		\
	}

inline const MStatus& MReturnStatus(MStatus& ioStatus, const char* inErrorTxt=NULL)
{
	if (inErrorTxt!=NULL)
		ioStatus.perror(inErrorTxt);
	return ioStatus;
}

inline const MStatus& MReturnSetFailure(MStatus& outStatus, const char* inErrorTxt=NULL)
{
	outStatus = MStatus::kFailure;
	if (inErrorTxt!=NULL)
		outStatus.perror(inErrorTxt);
	return outStatus;
}

inline MStatus MReturnFailure(const char* inErrorTxt=NULL)
{
	MStatus status = MStatus::kFailure;
	if (inErrorTxt!=NULL)
		status.perror(inErrorTxt);
	return status;
}

#endif
