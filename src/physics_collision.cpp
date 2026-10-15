#include "physics_collision.h"

#include "compact_surface.h"
#include "conversion.h"
#include "physics_collide.h"

#include "bspflags.h"
#include "tier0/commonmacros.h"
#include "vcollide.h"

#include "box3d/constants.h"

#include <algorithm>
#include <array>
#include <cstring>

CPhysicsCollision g_PhysicsCollision;

static constexpr int COMPACT_SURFACE_HEADER_SIZE = 28;
static constexpr int LEGACY_SURFACE_HEADER_SIZE = 48;

enum
{
	MODEL_TYPE_COMPACT_SURFACE = 0,
	MODEL_TYPE_NULL = 1,
};

struct compactsurfaceheader_t
{
	int vphysicsID;
	short version;
	short modelType;
	int surfaceSize;
	Vector dragAxisAreas;
	int axisMapSize;
};

static_assert( sizeof( compactsurfaceheader_t ) == COMPACT_SURFACE_HEADER_SIZE );

static CPhysConvex *CreateConvex( const b3Vec3 *pPoints, int nPointCount )
{
	b3HullData *pHull = CPhysConvex::CreateHull( pPoints, nPointCount );
	return pHull ? new CPhysConvex( pHull ) : nullptr;
}

static CPhysCollide *UnserializeSolid( const char *pBuffer, int nSize, int nIndex, bool bSwap )
{
	int nID;
	std::memcpy( &nID, pBuffer, sizeof( nID ) );

	if ( nID == MAKEID( 'V', 'P', 'H', 'Y' ) )
	{
		compactsurfaceheader_t header;
		std::memcpy( &header, pBuffer, sizeof( header ) );

		if ( header.modelType == MODEL_TYPE_NULL )
			DevMsg( 2, "Null physics model\n" );

		if ( header.modelType != MODEL_TYPE_COMPACT_SURFACE )
			return nullptr;

		CPhysCollide *pCollide = CreateCollideFromCompactSurface( pBuffer + COMPACT_SURFACE_HEADER_SIZE, header.surfaceSize, bSwap );
		if ( pCollide )
		{
			pCollide->SetIndex( nIndex );
			pCollide->SetOrthographicAreas( header.dragAxisAreas );
		}
		return pCollide;
	}

	if ( nSize < LEGACY_SURFACE_HEADER_SIZE )
		Error( "Corrupt physics model\n" );

	int nSurfaceID;
	std::memcpy( &nSurfaceID, pBuffer + 44, sizeof( nSurfaceID ) );

	if ( nSurfaceID == MAKEID( 'M', 'O', 'P', 'P' ) )
		return nullptr;

	if ( nSurfaceID != MAKEID( 'I', 'V', 'P', 'S' ) && nSurfaceID != MAKEID( 'S', 'P', 'V', 'I' ) )
	{
		if ( nSurfaceID != 0 )
			return nullptr;

		DevMsg( 1, "Old format .PHY file loaded!!!\n" );
	}

	CPhysCollide *pCollide = CreateCollideFromCompactSurface( pBuffer, nSize, bSwap );
	if ( pCollide )
		pCollide->SetIndex( nIndex );
	return pCollide;
}

CPhysicsCollision::~CPhysicsCollision()
{
	for ( const BBoxCacheEntry_t &entry : m_BBoxCache )
		delete entry.m_pCollide;
}

CPhysConvex *CPhysicsCollision::ConvexFromVerts( Vector **pVerts, int vertCount )
{
	std::vector<b3Vec3> points( std::max( vertCount, 0 ) );
	for ( int i = 0; i < vertCount; ++i )
		points[i] = ToBox3D( *pVerts[i] );

	return CreateConvex( points.data(), vertCount );
}

CPhysConvex *CPhysicsCollision::ConvexFromPlanes( float *pPlanes, int planeCount, float mergeDistance )
{
	(void)mergeDistance;

	struct Plane_t
	{
		double x, y, z, d;
	};

	std::vector<Plane_t> planes( std::max( planeCount, 0 ) );
	for ( int i = 0; i < planeCount; ++i )
		planes[i] = { pPlanes[i * 4 + 0], pPlanes[i * 4 + 1], pPlanes[i * 4 + 2], pPlanes[i * 4 + 3] };

	constexpr double flInsideEpsilon = 1e-3;
	constexpr double flParallelEpsilon = 1e-9;

	std::vector<b3Vec3> points;
	for ( int i = 0; i < planeCount; ++i )
	{
		for ( int j = i + 1; j < planeCount; ++j )
		{
			for ( int k = j + 1; k < planeCount; ++k )
			{
				const Plane_t &a = planes[i];
				const Plane_t &b = planes[j];
				const Plane_t &c = planes[k];

				const double bcx = b.y * c.z - b.z * c.y;
				const double bcy = b.z * c.x - b.x * c.z;
				const double bcz = b.x * c.y - b.y * c.x;
				const double det = a.x * bcx + a.y * bcy + a.z * bcz;
				if ( std::abs( det ) < flParallelEpsilon )
					continue;

				const double cax = c.y * a.z - c.z * a.y;
				const double cay = c.z * a.x - c.x * a.z;
				const double caz = c.x * a.y - c.y * a.x;
				const double abx = a.y * b.z - a.z * b.y;
				const double aby = a.z * b.x - a.x * b.z;
				const double abz = a.x * b.y - a.y * b.x;

				const double px = ( a.d * bcx + b.d * cax + c.d * abx ) / det;
				const double py = ( a.d * bcy + b.d * cay + c.d * aby ) / det;
				const double pz = ( a.d * bcz + b.d * caz + c.d * abz ) / det;

				const bool bInside = std::ranges::all_of( planes, [&]( const Plane_t &plane ) {
					return plane.x * px + plane.y * py + plane.z * pz <= plane.d + flInsideEpsilon;
				} );

				if ( bInside )
					points.push_back( { static_cast<float>( px ), static_cast<float>( py ), static_cast<float>( pz ) } );
			}
		}
	}

	return CreateConvex( points.data(), static_cast<int>( points.size() ) );
}

float CPhysicsCollision::ConvexVolume( CPhysConvex *pConvex )
{
	return pConvex->GetVolume();
}

float CPhysicsCollision::ConvexSurfaceArea( CPhysConvex *pConvex )
{
	return pConvex->GetSurfaceArea();
}

void CPhysicsCollision::SetConvexGameData( CPhysConvex *pConvex, unsigned int gameData )
{
	pConvex->SetGameData( gameData );
}

void CPhysicsCollision::ConvexFree( CPhysConvex *pConvex )
{
	delete pConvex;
}

CPhysConvex *CPhysicsCollision::BBoxToConvex( const Vector &mins, const Vector &maxs )
{
	const std::array<b3Vec3, 8> corners = { {
		{ mins.x, mins.y, mins.z },
		{ maxs.x, mins.y, mins.z },
		{ mins.x, maxs.y, mins.z },
		{ maxs.x, maxs.y, mins.z },
		{ mins.x, mins.y, maxs.z },
		{ maxs.x, mins.y, maxs.z },
		{ mins.x, maxs.y, maxs.z },
		{ maxs.x, maxs.y, maxs.z },
	} };

	return CreateConvex( corners.data(), static_cast<int>( corners.size() ) );
}

CPhysConvex *CPhysicsCollision::ConvexFromConvexPolyhedron( const CPolyhedron &ConvexPolyhedron )
{
	(void)ConvexPolyhedron;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

void CPhysicsCollision::ConvexesFromConvexPolygon( const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput )
{
	(void)vPolyNormal;
	(void)pPoints;
	(void)iPointCount;
	(void)pOutput;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

CPhysPolysoup *CPhysicsCollision::PolysoupCreate()
{
	Error( "%s is not implemented\n", __FUNCTION__ );
}

void CPhysicsCollision::PolysoupDestroy( CPhysPolysoup *pSoup )
{
	(void)pSoup;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

void CPhysicsCollision::PolysoupAddTriangle( CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits )
{
	(void)pSoup;
	(void)a;
	(void)b;
	(void)c;
	(void)materialIndex7bits;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

CPhysCollide *CPhysicsCollision::ConvertPolysoupToCollide( CPhysPolysoup *pSoup, bool useMOPP )
{
	(void)pSoup;
	(void)useMOPP;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollide( CPhysConvex **pConvex, int convexCount )
{
	convertconvexparams_t params;
	params.Defaults();
	return ConvertConvexToCollideParams( pConvex, convexCount, params );
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollideParams( CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams )
{
	if ( !pConvex || convexCount == 0 )
		return nullptr;

	std::vector<std::unique_ptr<CPhysConvex>> convexes;
	for ( int i = 0; i < convexCount; ++i )
	{
		if ( pConvex[i] )
			convexes.emplace_back( pConvex[i] );
	}

	if ( convexes.empty() )
		return nullptr;

	b3Vec3 weightedCenter = b3Vec3_zero;
	float flTotalVolume = 0.0f;
	for ( const std::unique_ptr<CPhysConvex> &pPiece : convexes )
	{
		if ( const b3HullData *pHull = pPiece->GetHull() )
		{
			weightedCenter = b3MulAdd( weightedCenter, pHull->volume, pHull->center );
			flTotalVolume += pHull->volume;
		}
	}

	auto *pCollide = new CPhysCollide( std::move( convexes ) );
	if ( flTotalVolume > 0.0f )
		pCollide->SetMassCenter( ToSource( b3MulSV( 1.0f / flTotalVolume, weightedCenter ) ) );

	pCollide->SetHasOuterHull( convertParams.buildOuterConvexHull );

	if ( convertParams.buildDragAxisAreas )
		Error( "%s: drag axis areas are not implemented\n", __FUNCTION__ );

	return pCollide;
}

bool CPhysicsCollision::IsCachedBBox( const CPhysCollide *pCollide ) const
{
	return std::ranges::any_of( m_BBoxCache, [pCollide]( const BBoxCacheEntry_t &entry ) { return entry.m_pCollide == pCollide; } );
}

void CPhysicsCollision::DestroyCollide( CPhysCollide *pCollide )
{
	if ( IsCachedBBox( pCollide ) )
		return;

	delete pCollide;
}

int CPhysicsCollision::CollideSize( CPhysCollide *pCollide )
{
	(void)pCollide;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

int CPhysicsCollision::CollideWrite( char *pDest, CPhysCollide *pCollide, bool bSwap )
{
	(void)pDest;
	(void)pCollide;
	(void)bSwap;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

CPhysCollide *CPhysicsCollision::UnserializeCollide( char *pBuffer, int size, int index )
{
	return UnserializeSolid( pBuffer, size, index, false );
}

float CPhysicsCollision::CollideVolume( CPhysCollide *pCollide )
{
	return pCollide->GetVolume();
}

float CPhysicsCollision::CollideSurfaceArea( CPhysCollide *pCollide )
{
	return pCollide->GetSurfaceArea();
}

Vector CPhysicsCollision::CollideGetExtent( const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction )
{
	if ( !pCollide )
		return collideOrigin;

	return pCollide->GetExtent( ToBox3D( collideOrigin, collideAngles ), direction );
}

void CPhysicsCollision::CollideGetAABB( Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles )
{
	pCollide->GetAABB( pMins, pMaxs, ToBox3D( collideOrigin, collideAngles ) );
}

void CPhysicsCollision::CollideGetMassCenter( CPhysCollide *pCollide, Vector *pOutMassCenter )
{
	*pOutMassCenter = pCollide->GetMassCenter();
}

void CPhysicsCollision::CollideSetMassCenter( CPhysCollide *pCollide, const Vector &massCenter )
{
	pCollide->SetMassCenter( massCenter );
}

Vector CPhysicsCollision::CollideGetOrthographicAreas( const CPhysCollide *pCollide )
{
	return pCollide ? pCollide->GetOrthographicAreas() : Vector( 0.0f, 0.0f, 0.0f );
}

void CPhysicsCollision::CollideSetOrthographicAreas( CPhysCollide *pCollide, const Vector &areas )
{
	if ( pCollide )
		pCollide->SetOrthographicAreas( areas );
}

int CPhysicsCollision::CollideIndex( const CPhysCollide *pCollide )
{
	return pCollide ? pCollide->GetIndex() : 0;
}

CPhysCollide *CPhysicsCollision::BBoxToCollide( const Vector &mins, const Vector &maxs )
{
	if ( mins == maxs )
		return nullptr;

	for ( auto it = m_BBoxCache.rbegin(); it != m_BBoxCache.rend(); ++it )
	{
		if ( it->m_Mins == mins && it->m_Maxs == maxs )
			return it->m_pCollide;
	}

	CPhysConvex *pConvex = BBoxToConvex( mins, maxs );
	CPhysCollide *pCollide = ConvertConvexToCollide( &pConvex, 1 );
	if ( pCollide )
		m_BBoxCache.push_back( { mins, maxs, pCollide } );

	return pCollide;
}

int CPhysicsCollision::GetConvexesUsedInCollideable( const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit )
{
	const auto &convexes = pCollideable->GetConvexes();
	const int nCount = std::min( static_cast<int>( convexes.size() ), iOutputArrayLimit );
	for ( int i = 0; i < nCount; ++i )
		pOutputArray[i] = convexes[i].get();
	return nCount;
}

void CPhysicsCollision::TraceBox( const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	(void)start;
	(void)end;
	(void)mins;
	(void)maxs;
	(void)pCollide;
	(void)collideOrigin;
	(void)collideAngles;
	(void)ptr;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

void CPhysicsCollision::TraceBox( const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	TraceBox( ray, MASK_ALL, nullptr, pCollide, collideOrigin, collideAngles, ptr );
}

void CPhysicsCollision::TraceBox( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	(void)ray;
	(void)contentsMask;
	(void)pConvexInfo;
	(void)pCollide;
	(void)collideOrigin;
	(void)collideAngles;
	(void)ptr;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

void CPhysicsCollision::TraceCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	(void)start;
	(void)end;
	(void)pSweepCollide;
	(void)sweepAngles;
	(void)pCollide;
	(void)collideOrigin;
	(void)collideAngles;
	(void)ptr;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

bool CPhysicsCollision::IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone )
{
	(void)boxAbsMins;
	(void)boxAbsMaxs;
	(void)cone;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

void CPhysicsCollision::VCollideLoad( vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap )
{
	std::memset( pOutput, 0, sizeof( *pOutput ) );
	pOutput->solidCount = solidCount & 0x7FFF;
	pOutput->solids = new CPhysCollide *[solidCount];

	int nOffset = 0;
	for ( int i = 0; i < solidCount; ++i )
	{
		int nSolidSize;
		std::memcpy( &nSolidSize, pBuffer + nOffset, sizeof( nSolidSize ) );

		pOutput->solids[i] = UnserializeSolid( pBuffer + nOffset + sizeof( int ), nSolidSize, i, swap );
		nOffset += sizeof( int ) + nSolidSize;
	}

	const int nKeyValuesSize = size - nOffset;
	pOutput->pKeyValues = new char[nKeyValuesSize];
	std::memcpy( pOutput->pKeyValues, pBuffer + nOffset, nKeyValuesSize );
}

void CPhysicsCollision::VCollideUnload( vcollide_t *pVCollide )
{
	for ( int i = 0; i < pVCollide->solidCount; ++i )
		delete pVCollide->solids[i];

	delete[] pVCollide->solids;
	delete[] pVCollide->pKeyValues;
	std::memset( pVCollide, 0, sizeof( *pVCollide ) );
}

IVPhysicsKeyParser *CPhysicsCollision::VPhysicsKeyParserCreate( const char *pKeyData )
{
	(void)pKeyData;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

void CPhysicsCollision::VPhysicsKeyParserDestroy( IVPhysicsKeyParser *pParser )
{
	(void)pParser;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

int CPhysicsCollision::CreateDebugMesh( CPhysCollide const *pCollisionModel, Vector **outVerts )
{
	int nVertexCount = 0;
	for ( const auto &pConvex : pCollisionModel->GetConvexes() )
		nVertexCount += static_cast<int>( pConvex->GetTriangles().size() ) * 3;

	Vector *pVerts = new Vector[nVertexCount];
	Vector *pCursor = pVerts;
	for ( const auto &pConvex : pCollisionModel->GetConvexes() )
	{
		const std::vector<Vector> &vertices = pConvex->GetVertices();
		for ( const CPhysConvex::Triangle_t &triangle : pConvex->GetTriangles() )
		{
			for ( unsigned short nVertex : triangle.m_nVertices )
				*pCursor++ = vertices[nVertex];
		}
	}

	*outVerts = pVerts;
	return nVertexCount;
}

void CPhysicsCollision::DestroyDebugMesh( int vertCount, Vector *outVerts )
{
	(void)vertCount;
	delete[] outVerts;
}

ICollisionQuery *CPhysicsCollision::CreateQueryModel( CPhysCollide *pCollide )
{
	(void)pCollide;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

void CPhysicsCollision::DestroyQueryModel( ICollisionQuery *pQuery )
{
	(void)pQuery;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

IPhysicsCollision *CPhysicsCollision::ThreadContextCreate()
{
	return this;
}

void CPhysicsCollision::ThreadContextDestroy( IPhysicsCollision *pThreadContex )
{
	(void)pThreadContex;
}

CPhysCollide *CPhysicsCollision::CreateVirtualMesh( const virtualmeshparams_t &params )
{
	(void)params;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

bool CPhysicsCollision::SupportsVirtualMesh()
{
	return true;
}

bool CPhysicsCollision::GetBBoxCacheSize( int *pCachedSize, int *pCachedCount )
{
	*pCachedCount = static_cast<int>( m_BBoxCache.size() );
	*pCachedSize = 0;
	for ( const BBoxCacheEntry_t &entry : m_BBoxCache )
		*pCachedSize += CollideSize( entry.m_pCollide );
	return true;
}

CPolyhedron *CPhysicsCollision::PolyhedronFromConvex( CPhysConvex *const pConvex, bool bUseTempPolyhedron )
{
	(void)pConvex;
	(void)bUseTempPolyhedron;
	Error( "%s is not implemented\n", __FUNCTION__ );
}

void CPhysicsCollision::OutputDebugInfo( const CPhysCollide *pCollide )
{
	Msg( "CollisionModel: Compact Surface: %d convex pieces %s outer hull\n", static_cast<int>( pCollide->GetConvexes().size() ), pCollide->HasOuterHull() ? "with" : "no" );
}

unsigned int CPhysicsCollision::ReadStat( int statID )
{
	(void)statID;
	return 0;
}
