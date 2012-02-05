#ifndef __stretchMeshCONSTS_H__
#define __stretchMeshCONSTS_H__
#pragma once

#include <math.h>

const float SM_VERSION = 1.6;
const float SM_POLAR_FIX = 1.1;
const int MAX_NUM_CONN_VRTS = 128;
const double PI = (4.0*atan(1.0));
const int ITERATIONS_DFLT = 10;
const double STIFFNESS_DFLT = 0.3;
const int COLLISION_STEP_DFLT = 1;
const bool COLLISION_DFLT = true;
const bool SCALE_SAFE_DFLT = false;
const bool EXTEND_CONN_VRTS_DFLT = false;

#endif