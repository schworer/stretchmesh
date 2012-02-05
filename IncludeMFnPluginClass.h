// Evil MFnPlugin.h include. Inludes MFnPlugin.h without DLL entry points
#define MNoVersionString
#define MNoPluginEntry
#ifdef NT_PLUGIN
	#undef NT_PLUGIN
		#include <maya/MFnPlugin.h>
	#define NT_PLUGIN
#else
	#include <maya/MFnPlugin.h>
#endif

