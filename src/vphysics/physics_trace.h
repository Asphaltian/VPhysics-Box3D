#pragma once

#include "gametrace.h"
#include "tier0/valve_minmax_off.h"

class CPhysCollide;
class CPhysCollideCompactSurface;
class IConvexInfo;
struct truncatedcone_t;

void TraceBoxAgainstCollide( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace );
void TraceCollideAgainstCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace );
bool IsBoxIntersectingTruncatedCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone );
void GetCompactSurfaceAABB( const CPhysCollideCompactSurface &collide, const Vector &origin, const QAngle &angles, Vector *pMins, Vector *pMaxs );
Vector GetCompactSurfaceExtent( const CPhysCollideCompactSurface &collide, const Vector &origin, const QAngle &angles, const Vector &direction );
