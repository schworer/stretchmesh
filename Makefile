## Makefile cribbed from the Bullet Physics makefile found here:
## http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=7783

MAYA_LOCATION=/Applications/Autodesk/maya2012/devkit

## MAYA_LOCATION is the Maya installation directory. It should be already defined in your
# environment variables. If not, please change it to the appropriate directory
MAYA=$(MAYA_LOCATION)

## Change this if you want to change the name of the final plugin 
LIBRARY=stretchMesh2012.bundle

##################################

CPP = g++
LD = ld

CPPFLAGS = -arch x86_64 -DMAC_PLUGIN -DOSMac_MachO_ -DBits32_ -m32 -DUNIX -D_BOOL -DOSMac_ -DFUNCPROTO -D_GNU_SOURCE  -fPIC \
		-fno-strict-aliasing -DREQUIRE_IOSTREAM -Wno-deprecated -Wall \
		-Wno-multichar -Wno-comment -Wno-sign-compare -funsigned-char \
		-Wno-reorder -fno-gnu-keywords -ftemplate-depth-25 -pthread \
		-Wno-deprecated -fno-gnu-keywords -g

#LDFLAGS =-bundle -ldl -shared
LDFLAGS = -ldl -shared

GL_LIB=-framework OpenGL

MAYA_INCLUDE=-I$(MAYA)/include
MAYA_LIB=-L/Applications/Autodesk/maya2012/Maya.app/Contents/MacOS -lOpenMaya -lFoundation -Wl,-executable_path,/Applications/Autodesk/maya2012/Maya.app/Contents/MacOS -lOpenMayaUI -lOpenMaya -lOpenMayaRender -lOpenMayaAnim -lFoundation

SOURCES = curveColliderLocator.cpp stretchMeshCmd.cpp pluginMain.cpp stretchMeshDeformer.cpp

HEADERS = CRSpline.h ksUtils.h stretchMeshDeformer.h IncludeMFnPluginClass.h stretchMeshCmd.h curveColliderLocator.h stretchMeshConsts.h


INCLUDE_FLAGS= $(GL_INCLUDE) $(MAYA_INCLUDE)
LIB_FLAGS= $(MAYA_LIB) $(GL_LIB)

OBJECTS=$(SOURCES:.cpp=.o)

all: $(SOURCES) $(LIBRARY)

.cpp.o: $(SOURCES) $(HEADERS)
	$(CPP) -c $< $(CPPFLAGS) $(INCLUDE_FLAGS) -o $@ 

$(OBJECTS): $(HEADERS)

$(LIBRARY): $(OBJECTS) 
	$(CPP) $(OBJECTS) $(LDFLAGS) $(LIB_FLAGS) -o $@

clean:
	rm -f *.o *.bundle

