#include "physics_collision.h"

#include "compact_builder/compact_surface.h"
#include "compact_builder/surface_builder_ledge_soup.h"
#include "compact_builder/surface_builder_pointsoup.h"
#include "conversion.h"
#include "physics_collide.h"
#include "physics_polyhedron.h"
#include "physics_trace.h"
#include "physics_virtual_mesh.h"
#include "vphysics_key_parser.h"

#include "bspflags.h"
#include "tier0/commonmacros.h"
#include "vcollide.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

CPhysicsCollision g_PhysicsCollision;

static constexpr float DRAG_SAMPLE_MINIMUM_STEP = 0.0001f;
static constexpr float PLANE_NORMAL_MINIMUM_LENGTH = 1e-6f;
static constexpr float REBUILD_MERGE_DISTANCE = 0.01f;

class CPhysPolysoup
{
public:
	std::vector<std::vector<unsigned char>> m_Ledges;
	bool m_bHasTriangles = false;
};

static CPhysConvex *CreateConvexFromPoints( const std::vector<RealVec3_t> &points )
{
	std::vector<unsigned char> ledge = BuildPointSoupLedge( points );
	if ( ledge.empty() )
		return nullptr;

	CPlaneSet planes;
	const unsigned char *pLedge = ledge.data();
	for ( int nTriangle = 0; nTriangle < GetLedgeTriangleCount( pLedge ); ++nTriangle )
	{
		const unsigned char *pTriangle = GetLedgeTriangle( pLedge, nTriangle );
		const RealVec3_t a = ToReal( LoadPoint( GetLedgePoint( pLedge, GetEdgeStartPoint( pTriangle, 0 ) ) ) );
		const RealVec3_t b = ToReal( LoadPoint( GetLedgePoint( pLedge, GetEdgeStartPoint( pTriangle, 1 ) ) ) );
		const RealVec3_t c = ToReal( LoadPoint( GetLedgePoint( pLedge, GetEdgeStartPoint( pTriangle, 2 ) ) ) );

		const RealVec3_t normal = Cross( c - a, b - a );
		const real_t flLength = Length( normal );
		if ( static_cast<float>( flLength ) > PLANE_NORMAL_MINIMUM_LENGTH )
		{
			const real_t flInverse = 1 / flLength;
			planes.AddPlane( flInverse * normal, flInverse * -Dot( normal, a ) );
		}
	}

	std::vector<unsigned char> rebuilt = BuildPointSoupLedge( planes.ComputeVertices( REBUILD_MERGE_DISTANCE ) );
	return CreateConvexFromLedge( rebuilt.empty() ? std::move( ledge ) : std::move( rebuilt ) );
}

static Vector ComputeOrthographicAreas( CPhysCollide *pCollide, float flAreaEpsilon )
{
	Vector mins, maxs;
	pCollide->GetAABB( &mins, &maxs, b3Transform_identity );

	float flStep = std::sqrt( flAreaEpsilon );
	if ( flStep <= DRAG_SAMPLE_MINIMUM_STEP )
		flStep = DRAG_SAMPLE_MINIMUM_STEP;
	const float flHalfStep = flStep * 0.5f;

	auto sampleAxis = [&]( int nAxis, int nOuter, int nInner ) {
		int nTotal = 0;
		int nHits = 0;
		for ( float flOuter = mins[nOuter] + flHalfStep; flOuter < maxs[nOuter]; flOuter += flStep )
		{
			for ( float flInner = mins[nInner] + flHalfStep; flInner < maxs[nInner]; flInner += flStep )
			{
				Vector start, end;
				start[nOuter] = end[nOuter] = flOuter;
				start[nInner] = end[nInner] = flInner;
				start[nAxis] = mins[nAxis] - 1.0f;
				end[nAxis] = maxs[nAxis] + 1.0f;

				Ray_t ray;
				ray.Init( start, end, Vector( 0.0f, 0.0f, 0.0f ), Vector( 0.0f, 0.0f, 0.0f ) );
				trace_t trace;
				TraceBoxAgainstCollide( ray, MASK_ALL, nullptr, pCollide, Vector( 0.0f, 0.0f, 0.0f ), QAngle( 0.0f, 0.0f, 0.0f ), &trace );

				++nTotal;
				if ( trace.fraction < 1.0f || trace.allsolid || trace.startsolid )
					++nHits;
			}
		}

		return static_cast<float>( nHits ) / static_cast<float>( nTotal ? nTotal : 1 );
	};

	return Vector( sampleAxis( 0, 1, 2 ), sampleAxis( 1, 2, 0 ), sampleAxis( 2, 0, 1 ) );
}

static CPhysCollide *UnserializeSolid( const char *pBuffer, int nSize, int nIndex, bool bSwap )
{
	int nID;
	std::memcpy( &nID, pBuffer, sizeof( nID ) );

	if ( nID == MAKEID( 'V', 'P', 'H', 'Y' ) )
	{
		physcollideheader_t collideHeader;
		compactsurfaceheader_t surfaceHeader;
		std::memcpy( &collideHeader, pBuffer, sizeof( collideHeader ) );
		std::memcpy( &surfaceHeader, pBuffer + sizeof( collideHeader ), sizeof( surfaceHeader ) );

		if ( collideHeader.modelType == MODEL_TYPE_NULL )
			DevMsg( 2, "Null physics model\n" );

		if ( collideHeader.modelType != MODEL_TYPE_COMPACT_SURFACE )
			return nullptr;

		CPhysCollideCompactSurface *pCollide = CreateCollideFromCompactSurface( pBuffer + COLLIDE_HEADER_SIZE, surfaceHeader.surfaceSize, bSwap );
		if ( pCollide )
		{
			pCollide->SetIndex( nIndex );
			pCollide->SetOrthographicAreas( Vector( surfaceHeader.dragAxisAreas[0], surfaceHeader.dragAxisAreas[1], surfaceHeader.dragAxisAreas[2] ) );
		}
		return pCollide;
	}

	if ( nSize < static_cast<int>( COMPACT_SURFACE_HEADER_SIZE ) )
		Error( "Corrupt physics model\n" );

	int nSurfaceID;
	std::memcpy( &nSurfaceID, pBuffer + SURFACE_ID_OFFSET, sizeof( nSurfaceID ) );

	if ( nSurfaceID == MAKEID( 'M', 'O', 'P', 'P' ) )
		return nullptr;

	if ( nSurfaceID != MAKEID( 'I', 'V', 'P', 'S' ) && nSurfaceID != MAKEID( 'S', 'P', 'V', 'I' ) )
	{
		if ( nSurfaceID != 0 )
			return nullptr;

		DevMsg( 1, "Old format .PHY file loaded!!!\n" );
	}

	CPhysCollideCompactSurface *pCollide = CreateCollideFromCompactSurface( pBuffer, nSize, bSwap );
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
	std::vector<RealVec3_t> points( std::max( vertCount, 0 ) );
	for ( int i = 0; i < vertCount; ++i )
		points[i] = ToReal( SourceToIVP( *pVerts[i] ) );

	return CreateConvexFromPoints( points );
}

CPhysConvex *CPhysicsCollision::ConvexFromPlanes( float *pPlanes, int planeCount, float mergeDistance )
{
	CPlaneSet planes;
	for ( int i = 0; i < planeCount; ++i )
	{
		const float *pPlane = pPlanes + i * 4;
		planes.AddPlane( { -pPlane[0], pPlane[2], -pPlane[1] }, SOURCE_TO_IVP * pPlane[3] );
	}

	std::vector<unsigned char> ledge = BuildPointSoupLedge( planes.ComputeVertices( mergeDistance * SOURCE_TO_IVP ) );
	return ledge.empty() ? nullptr : CreateConvexFromLedge( std::move( ledge ) );
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
	const std::vector<RealVec3_t> corners = {
		ToReal( SourceToIVP( Vector( mins.x, mins.y, mins.z ) ) ),
		ToReal( SourceToIVP( Vector( maxs.x, mins.y, mins.z ) ) ),
		ToReal( SourceToIVP( Vector( mins.x, maxs.y, mins.z ) ) ),
		ToReal( SourceToIVP( Vector( maxs.x, maxs.y, mins.z ) ) ),
		ToReal( SourceToIVP( Vector( mins.x, mins.y, maxs.z ) ) ),
		ToReal( SourceToIVP( Vector( maxs.x, mins.y, maxs.z ) ) ),
		ToReal( SourceToIVP( Vector( mins.x, maxs.y, maxs.z ) ) ),
		ToReal( SourceToIVP( Vector( maxs.x, maxs.y, maxs.z ) ) ),
	};

	std::vector<unsigned char> ledge = BuildPointSoupLedge( corners );
	return ledge.empty() ? nullptr : CreateConvexFromLedge( std::move( ledge ) );
}

CPhysConvex *CPhysicsCollision::ConvexFromConvexPolyhedron( const CPolyhedron &ConvexPolyhedron )
{
	return CreateConvexFromPolyhedron( ConvexPolyhedron );
}

void CPhysicsCollision::ConvexesFromConvexPolygon( const Vector &, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput )
{
	if ( iPointCount < 2 )
	{
		pOutput[0] = nullptr;
		return;
	}

	if ( iPointCount == 2 )
	{
		pOutput[0] = CreateTriangleConvex( pPoints[0], pPoints[1], pPoints[1] );
		return;
	}

	std::vector<int> strip;
	strip.reserve( iPointCount );
	for ( int nLow = 0, nHigh = iPointCount - 1; nLow <= nHigh; )
	{
		strip.push_back( nLow++ );
		if ( nLow <= nHigh )
			strip.push_back( nHigh-- );
	}

	for ( int i = 0; i + 2 < iPointCount; ++i )
		pOutput[i] = CreateTriangleConvex( pPoints[strip[i]], pPoints[strip[i + 1]], pPoints[strip[i + 2]] );
}

CPhysPolysoup *CPhysicsCollision::PolysoupCreate()
{
	return new CPhysPolysoup;
}

void CPhysicsCollision::PolysoupDestroy( CPhysPolysoup *pSoup )
{
	delete pSoup;
}

void CPhysicsCollision::PolysoupAddTriangle( CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits )
{
	pSoup->m_bHasTriangles = true;

	std::vector<unsigned char> ledge = WriteTriangleLedge( ToReal( SourceToIVP( a ) ), ToReal( SourceToIVP( b ) ), ToReal( SourceToIVP( c ) ) );
	if ( ledge.empty() )
	{
		Warning( "Degenerate Triangle\n" );
		Warning( "(%.2f, %.2f, %.2f), ", a.x, a.y, a.z );
		Warning( "(%.2f, %.2f, %.2f), ", b.x, b.y, b.z );
		Warning( "(%.2f, %.2f, %.2f)\n", c.x, c.y, c.z );
		return;
	}

	SetTriangleMaterial( GetLedgeTriangle( ledge.data(), 0 ), static_cast<uint32_t>( materialIndex7bits ) );
	pSoup->m_Ledges.push_back( std::move( ledge ) );
}

CPhysCollide *CPhysicsCollision::ConvertPolysoupToCollide( CPhysPolysoup *pSoup, bool )
{
	if ( !pSoup->m_bHasTriangles || pSoup->m_Ledges.empty() )
		return nullptr;

	std::vector<const std::vector<unsigned char> *> ledges;
	for ( const std::vector<unsigned char> &ledge : pSoup->m_Ledges )
		ledges.push_back( &ledge );

	std::vector<unsigned char> surface = BuildCompactSurface( ledges, {} );
	pSoup->m_Ledges.clear();
	if ( surface.empty() )
		return nullptr;

	CPhysCollideCompactSurface *pCollide = CreateCollideFromCompactSurface( surface );
	if ( !pCollide )
		return nullptr;

	for ( int nConvex = 0; nConvex < pCollide->GetLedgeCount(); ++nConvex )
	{
		const unsigned char *pLedge = pCollide->GetLedge( nConvex );
		const int nTriangleCount = GetLedgeTriangleCount( pLedge );
		int nMaterial = static_cast<int>( GetTriangleMaterial( GetLedgeTriangle( pLedge, 0 ) ) );
		if ( nMaterial == 0 )
		{
			for ( int nTriangle = 0; nTriangle < nTriangleCount; ++nTriangle )
			{
				if ( const uint32_t nTriangleMaterial = GetTriangleMaterial( GetLedgeTriangle( pLedge, nTriangle ) ) )
					nMaterial = static_cast<int>( nTriangleMaterial );
			}
		}

		for ( int nTriangle = 0; nTriangle < nTriangleCount; ++nTriangle )
			pCollide->SetTriangleMaterial( nConvex, nTriangle, nMaterial );
	}

	return pCollide;
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
	std::vector<const std::vector<unsigned char> *> ledges;
	for ( int i = 0; i < convexCount; ++i )
	{
		if ( !pConvex[i] )
			continue;

		convexes.emplace_back( pConvex[i] );
		ledges.push_back( &pConvex[i]->GetLedge() );
	}

	if ( convexes.empty() )
		return nullptr;

	std::unique_ptr<CPhysConvex> pForcedOuterHull( convertParams.pForcedOuterHull );
	CompactSurfaceParams_t params;
	params.m_bBuildOuterHull = convertParams.buildOuterConvexHull;
	params.m_pForcedOuterHull = pForcedOuterHull ? &pForcedOuterHull->GetLedge() : nullptr;

	std::vector<unsigned char> surface = BuildCompactSurface( ledges, params );
	if ( surface.empty() )
		return nullptr;

	CPhysCollide *pCollide = CreateCollideFromCompactSurface( surface );
	if ( pCollide && convertParams.buildDragAxisAreas )
		pCollide->SetOrthographicAreas( ComputeOrthographicAreas( pCollide, convertParams.dragAreaEpsilon ) );

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
	return pCollide->GetSerializationSize();
}

int CPhysicsCollision::CollideWrite( char *pDest, CPhysCollide *pCollide, bool bSwap )
{
	const int nSize = pCollide->GetSerializationSize();
	if ( nSize > 0 )
		pCollide->Serialize( pDest, bSwap );
	return nSize;
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

	if ( const CPhysCollideCompactSurface *pSurface = pCollide->GetCompactSurface() )
		return GetCompactSurfaceExtent( *pSurface, collideOrigin, collideAngles, direction );

	return pCollide->GetExtent( ToBox3D( collideOrigin, collideAngles ), direction );
}

void CPhysicsCollision::CollideGetAABB( Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles )
{
	if ( const CPhysCollideCompactSurface *pSurface = pCollide->GetCompactSurface() )
	{
		GetCompactSurfaceAABB( *pSurface, collideOrigin, collideAngles, pMins, pMaxs );
		return;
	}

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
	Ray_t ray;
	ray.Init( start, end, mins, maxs );
	TraceBoxAgainstCollide( ray, MASK_ALL, nullptr, pCollide, collideOrigin, collideAngles, ptr );
}

void CPhysicsCollision::TraceBox( const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	TraceBox( ray, MASK_ALL, nullptr, pCollide, collideOrigin, collideAngles, ptr );
}

void CPhysicsCollision::TraceBox( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	TraceBoxAgainstCollide( ray, contentsMask, pConvexInfo, pCollide, collideOrigin, collideAngles, ptr );
}

void CPhysicsCollision::TraceCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	TraceCollideAgainstCollide( start, end, pSweepCollide, sweepAngles, pCollide, collideOrigin, collideAngles, ptr );
}

bool CPhysicsCollision::IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone )
{
	return IsBoxIntersectingTruncatedCone( boxAbsMins, boxAbsMaxs, cone );
}

void CPhysicsCollision::VCollideLoad( vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap )
{
	std::memset( pOutput, 0, sizeof( *pOutput ) );
	pOutput->solidCount = solidCount & 0x7FFF;

	int nOffset = 0;
	for ( int i = 0; i < solidCount; ++i )
	{
		int nSolidSize;
		std::memcpy( &nSolidSize, pBuffer + nOffset, sizeof( nSolidSize ) );
		nOffset += sizeof( int ) + nSolidSize;
	}

	const int nKeyValuesSize = size - nOffset;
	char *pBlock = new char[sizeof( CPhysCollide * ) * solidCount + nKeyValuesSize];
	pOutput->solids = reinterpret_cast<CPhysCollide **>( pBlock );
	pOutput->pKeyValues = pBlock + sizeof( CPhysCollide * ) * solidCount;
	std::memcpy( pOutput->pKeyValues, pBuffer + nOffset, nKeyValuesSize );

	nOffset = 0;
	for ( int i = 0; i < solidCount; ++i )
	{
		int nSolidSize;
		std::memcpy( &nSolidSize, pBuffer + nOffset, sizeof( nSolidSize ) );
		pOutput->solids[i] = UnserializeSolid( pBuffer + nOffset + sizeof( int ), nSolidSize, i, swap );
		nOffset += sizeof( int ) + nSolidSize;
	}
}

void CPhysicsCollision::VCollideUnload( vcollide_t *pVCollide )
{
	for ( int i = 0; i < pVCollide->solidCount; ++i )
		delete pVCollide->solids[i];

	delete[] reinterpret_cast<char *>( pVCollide->solids );
	std::memset( pVCollide, 0, sizeof( *pVCollide ) );
}

IVPhysicsKeyParser *CPhysicsCollision::VPhysicsKeyParserCreate( const char *pKeyData )
{
	return new CVPhysicsKeyParser( pKeyData );
}

void CPhysicsCollision::VPhysicsKeyParserDestroy( IVPhysicsKeyParser *pParser )
{
	delete static_cast<CVPhysicsKeyParser *>( pParser );
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
		const std::vector<b3Vec3> &points = pConvex->GetPoints();
		for ( const CPhysConvex::Triangle_t &triangle : pConvex->GetTriangles() )
		{
			for ( unsigned short nVertex : triangle.m_nVertices )
				*pCursor++ = ToSource( points[nVertex] );
		}
	}

	*outVerts = pVerts;
	return nVertexCount;
}

void CPhysicsCollision::DestroyDebugMesh( int, Vector *outVerts )
{
	delete[] outVerts;
}

class CCompactSurfaceQuery final : public ICollisionQuery
{
public:
	CCompactSurfaceQuery( CPhysCollide *pCollide, const CPhysCollideCompactSurface *pSurface )
	: m_pCollide( pCollide )
	, m_pSurface( pSurface )
	{
	}

	int ConvexCount() override { return m_pSurface->GetLedgeCount(); }

	int TriangleCount( int convexIndex ) override { return GetLedgeTriangleCount( m_pSurface->GetLedge( convexIndex ) ); }

	unsigned int GetGameData( int convexIndex ) override { return GetLedgeClientData( m_pSurface->GetLedge( convexIndex ) ); }

	void GetTriangleVerts( int convexIndex, int triangleIndex, Vector *verts ) override
	{
		const unsigned char *pLedge = m_pSurface->GetLedge( convexIndex );
		const unsigned char *pTriangle = GetLedgeTriangle( pLedge, triangleIndex );
		for ( int i = 0; i < 3; ++i )
			verts[i] = ToSource( IVPToSource( LoadPoint( GetLedgePoint( pLedge, GetEdgeStartPoint( pTriangle, 2 - i ) ) ) ) );
	}

	void SetTriangleVerts( int, int, const Vector * ) override
	{
	}

	int GetTriangleMaterialIndex( int convexIndex, int triangleIndex ) override
	{
		return static_cast<int>( GetTriangleMaterial( GetLedgeTriangle( m_pSurface->GetLedge( convexIndex ), triangleIndex ) ) );
	}

	void SetTriangleMaterialIndex( int convexIndex, int triangleIndex, int index7bits ) override
	{
		m_pCollide->SetTriangleMaterial( convexIndex, triangleIndex, index7bits );
	}

private:
	CPhysCollide *m_pCollide;
	const CPhysCollideCompactSurface *m_pSurface;
};

class CConvexQuery final : public ICollisionQuery
{
public:
	explicit CConvexQuery( CPhysCollide *pCollide )
	: m_pCollide( pCollide )
	{
	}

	int ConvexCount() override { return static_cast<int>( m_pCollide->GetConvexes().size() ); }

	int TriangleCount( int convexIndex ) override { return static_cast<int>( GetConvex( convexIndex ).GetTriangles().size() ); }

	unsigned int GetGameData( int convexIndex ) override { return GetConvex( convexIndex ).GetGameData(); }

	void GetTriangleVerts( int convexIndex, int triangleIndex, Vector *verts ) override
	{
		const CPhysConvex &convex = GetConvex( convexIndex );
		const CPhysConvex::Triangle_t &triangle = convex.GetTriangles()[triangleIndex];
		for ( int i = 0; i < 3; ++i )
			verts[i] = ToSource( convex.GetPoints()[triangle.m_nVertices[i]] );
	}

	void SetTriangleVerts( int, int, const Vector * ) override
	{
	}

	int GetTriangleMaterialIndex( int convexIndex, int triangleIndex ) override { return GetConvex( convexIndex ).GetTriangles()[triangleIndex].m_nMaterialIndex; }

	void SetTriangleMaterialIndex( int convexIndex, int triangleIndex, int index7bits ) override
	{
		m_pCollide->SetTriangleMaterial( convexIndex, triangleIndex, index7bits );
	}

private:
	const CPhysConvex &GetConvex( int convexIndex ) const { return *m_pCollide->GetConvexes()[convexIndex]; }

	CPhysCollide *m_pCollide;
};

ICollisionQuery *CPhysicsCollision::CreateQueryModel( CPhysCollide *pCollide )
{
	if ( const CPhysCollideCompactSurface *pSurface = pCollide->GetCompactSurface() )
		return new CCompactSurfaceQuery( pCollide, pSurface );
	return new CConvexQuery( pCollide );
}

void CPhysicsCollision::DestroyQueryModel( ICollisionQuery *pQuery )
{
	delete pQuery;
}

IPhysicsCollision *CPhysicsCollision::ThreadContextCreate()
{
	return this;
}

void CPhysicsCollision::ThreadContextDestroy( IPhysicsCollision * )
{
}

CPhysCollide *CPhysicsCollision::CreateVirtualMesh( const virtualmeshparams_t &params )
{
	return new CPhysCollideVirtualMesh( params );
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
	return CreatePolyhedronFromConvex( *pConvex, bUseTempPolyhedron );
}

void CPhysicsCollision::OutputDebugInfo( const CPhysCollide *pCollide )
{
	pCollide->OutputDebugInfo();
}

unsigned int CPhysicsCollision::ReadStat( int )
{
	return 0;
}
