/*
 *  CRSpline.h
 *  stretchMesh
 */
#ifndef __CRSpline_h__
#define __CRSpline_h__
#pragma once

#include <maya/MPoint.h>
#include <maya/MPointArray.h>

// Solve the Catmull-Rom parametric equation for a given time(t) and vector quadruple (p1,p2,p3,p4)
inline MPoint computeSplinePoint(float t, const MPoint& p1, const MPoint& p2, const MPoint& p3, const MPoint& p4)
{
	float t2 = t * t;
	float t3 = t2 * t;

	float b1 = .5 * (  -t3 + 2*t2 - t);
	float b2 = .5 * ( 3*t3 - 5*t2 + 2);
	float b3 = .5 * (-3*t3 + 4*t2 + t);
	float b4 = .5 * (   t3 -   t2    );
	
	MPoint result;
	result =  (p1*b1 + p2*b2 + p3*b3 + p4*b4); 
	
	return result; 
}

inline MPoint getInterpolatedSplinePoint(float t, const MPointArray& crvPoints)
{
	
	// Find out in which interval we are on the spline
	float delta_t = (float)1 / ((float)crvPoints.length()-1);
	
	int p = (int)(t / delta_t);
#define BOUNDS(pp) { if (pp < 0) pp = 0; else if (pp >= (int)crvPoints.length()-1) pp = crvPoints.length() - 1; }
	int p0 = p - 1;     BOUNDS(p0);
	int p1 = p;         BOUNDS(p1);
	int p2 = p + 1;     BOUNDS(p2);
	int p3 = p + 2;     BOUNDS(p3);
	// Relative (local) time 
	float lt = (t - delta_t*(float)p) / delta_t;
	// Interpolate
	return computeSplinePoint(lt, crvPoints[p0], crvPoints[p1], crvPoints[p2], crvPoints[p3]);
}



#endif // __CRSpline_h__

