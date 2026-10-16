#pragma once

#include "vphysics_interface.h"
#include "tier0/valve_minmax_off.h"

#include <vector>

class CPhysicsCollision final : public IPhysicsCollision
{
public:
	~CPhysicsCollision() override;

	CPhysConvex *ConvexFromVerts( Vector **pVerts, int vertCount ) override;
	CPhysConvex *ConvexFromPlanes( float *pPlanes, int planeCount, float mergeDistance ) override;
	float ConvexVolume( CPhysConvex *pConvex ) override;
	float ConvexSurfaceArea( CPhysConvex *pConvex ) override;
	void SetConvexGameData( CPhysConvex *pConvex, unsigned int gameData ) override;
	void ConvexFree( CPhysConvex *pConvex ) override;
	CPhysConvex *BBoxToConvex( const Vector &mins, const Vector &maxs ) override;
	CPhysConvex *ConvexFromConvexPolyhedron( const CPolyhedron &ConvexPolyhedron ) override;
	void ConvexesFromConvexPolygon( const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput ) override;

	CPhysPolysoup *PolysoupCreate() override;
	void PolysoupDestroy( CPhysPolysoup *pSoup ) override;
	void PolysoupAddTriangle( CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits ) override;
	CPhysCollide *ConvertPolysoupToCollide( CPhysPolysoup *pSoup, bool useMOPP ) override;

	CPhysCollide *ConvertConvexToCollide( CPhysConvex **pConvex, int convexCount ) override;
	CPhysCollide *ConvertConvexToCollideParams( CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams ) override;
	void DestroyCollide( CPhysCollide *pCollide ) override;

	int CollideSize( CPhysCollide *pCollide ) override;
	int CollideWrite( char *pDest, CPhysCollide *pCollide, bool bSwap ) override;
	CPhysCollide *UnserializeCollide( char *pBuffer, int size, int index ) override;

	float CollideVolume( CPhysCollide *pCollide ) override;
	float CollideSurfaceArea( CPhysCollide *pCollide ) override;
	Vector CollideGetExtent( const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction ) override;
	void CollideGetAABB( Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles ) override;
	void CollideGetMassCenter( CPhysCollide *pCollide, Vector *pOutMassCenter ) override;
	void CollideSetMassCenter( CPhysCollide *pCollide, const Vector &massCenter ) override;
	Vector CollideGetOrthographicAreas( const CPhysCollide *pCollide ) override;
	void CollideSetOrthographicAreas( CPhysCollide *pCollide, const Vector &areas ) override;
	int CollideIndex( const CPhysCollide *pCollide ) override;

	CPhysCollide *BBoxToCollide( const Vector &mins, const Vector &maxs ) override;
	int GetConvexesUsedInCollideable( const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit ) override;

	void TraceBox( const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr ) override;
	void TraceBox( const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr ) override;
	void TraceBox( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr ) override;
	void TraceCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr ) override;
	bool IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone ) override;

	void VCollideLoad( vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap ) override;
	void VCollideUnload( vcollide_t *pVCollide ) override;

	IVPhysicsKeyParser *VPhysicsKeyParserCreate( const char *pKeyData ) override;
	void VPhysicsKeyParserDestroy( IVPhysicsKeyParser *pParser ) override;

	int CreateDebugMesh( CPhysCollide const *pCollisionModel, Vector **outVerts ) override;
	void DestroyDebugMesh( int vertCount, Vector *outVerts ) override;

	ICollisionQuery *CreateQueryModel( CPhysCollide *pCollide ) override;
	void DestroyQueryModel( ICollisionQuery *pQuery ) override;

	IPhysicsCollision *ThreadContextCreate() override;
	void ThreadContextDestroy( IPhysicsCollision *pThreadContex ) override;

	CPhysCollide *CreateVirtualMesh( const virtualmeshparams_t &params ) override;
	bool SupportsVirtualMesh() override;

	bool GetBBoxCacheSize( int *pCachedSize, int *pCachedCount ) override;

	CPolyhedron *PolyhedronFromConvex( CPhysConvex *const pConvex, bool bUseTempPolyhedron ) override;

	void OutputDebugInfo( const CPhysCollide *pCollide ) override;
	unsigned int ReadStat( int statID ) override;

private:
	struct BBoxCacheEntry_t
	{
		Vector m_Mins;
		Vector m_Maxs;
		CPhysCollide *m_pCollide;
	};

	bool IsCachedBBox( const CPhysCollide *pCollide ) const;

	std::vector<BBoxCacheEntry_t> m_BBoxCache;
};

extern CPhysicsCollision g_PhysicsCollision;
