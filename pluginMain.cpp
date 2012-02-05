#include <iostream>

#include <maya/MFnPlugin.h>
#include <maya/MIOStream.h>
#include <maya/MGlobal.h>
#include <maya/MSceneMessage.h>

#include "stretchMeshDeformer.h"
#include "stretchMeshCmd.h"
#include "stretchMeshConsts.h"
#include "curveColliderLocator.h"

using namespace std;

//---------------------------------------------------------------------------------------------------------------------
// initializePlugin / uninitializePlugin
//---------------------------------------------------------------------------------------------------------------------

MStatus initializePlugin( MObject obj )
{
 	char smVersionStr[16];
	sprintf(smVersionStr, "%.1f", SM_VERSION);
	MStatus   status;
	MFnPlugin plugin( obj, "Kickstand", smVersionStr, "Any");
	bool licensed = true;

	MString buildMenuCmd;
	if ( licensed )
	{
		buildMenuCmd = "int $smCmdEcho;\n";
		buildMenuCmd += "$smCmdEcho = `commandEcho -q -state`;\n";	
		buildMenuCmd += "commandEcho -state off;\n";
		MGlobal::executeCommand(buildMenuCmd);
	}

	// Register Deformer, passing licensed parameter
	status = stretchMeshDeformer::Register( plugin, licensed );
	if (!status) {
		status.perror("Couldn't register plugin");
		return status;
	}

	// Register Command only if licensed
	if ( licensed )
	{
		status = stretchMeshCmd::Register( plugin );
		if (!status) {
			status.perror("Couldn't register plugin");
			return status;
		}
	}

	// Register Curve Collider Locator
	status = curveColliderLocator::Register( plugin );
	if (!status) {
		status.perror("Couldn't register plugin");
		return status;
	}
	
	// Make the stiffness attribute paintable only if licensed
	if ( licensed )
	{
		MGlobal::executeCommand("makePaintable -attrType \"multiFloat\" -sm \"deformer\" \"weightGeometryFilter\" \"stiffness\"");
		stretchMeshCmd::buildstretchMeshCmdMenu();

		buildMenuCmd = "commandEcho -state $smCmdEcho;\n";
		MGlobal::executeCommand(buildMenuCmd);
	}

	return MStatus::kSuccess;
}

MStatus uninitializePlugin( MObject obj)
{
	MStatus status;
	MFnPlugin plugin( obj );

	if ( stretchMeshCmd::Registered )
	{
		// Remove Menu
		MGlobal::executeCommand("catch(`stretchMeshCmdMenuRemove`)");

		status = stretchMeshCmd::Deregister( plugin );

		if (!status) {
			status.perror("Couldn't unload plugin.");
			return status;
		}
	}

	if ( stretchMeshDeformer::Registered )
	{
		status = stretchMeshDeformer::Deregister( plugin );

		if (!status) {
			status.perror("Couldn't unload plugin.");
			return status;
		}
	}
	
	if ( curveColliderLocator::Registered )
	{
		status = curveColliderLocator::Deregister( plugin );
		
		if (!status) {
			status.perror("Couldn't unload plugin.");
			return status;
		}
	}
	
	return status;
}