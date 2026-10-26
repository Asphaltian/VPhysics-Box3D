#include "physics_collide.h"

#include "compact_builder/compact_surface.h"
#include "compact_builder/vector_hash.h"
#include "conversion.h"

#include "box3d/constants.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <future>
#include <mutex>
#include <new>

static constexpr float LEAF_SPHERE_MARGIN = 1.001f;
static constexpr float QUANTIZED_RADIUS_SCALE = 0.004f;
static constexpr float MIN_MESH_TRIANGLE_AREA_SCALE = 0.01f;

struct LedgeScan_t
{
	uint32_t m_nMinimum;
	uint32_t m_nMaximum;
	uint32_t m_nWindowBase;
	bool m_bWindowValid;
	uint8_t m_UsedWindow[2 * MAX_COLLIDE_MAP_SPAN];
};

static bool ScanLedge( const unsigned char *pBytes, size_t nByteCount, size_t nLedge, LedgeScan_t &scan )
{
	if ( !ContainsRange( nByteCount, nLedge, COMPACT_LEDGE_HEADER_SIZE ) )
		return false;

	const unsigned char *pLedge = pBytes + nLedge;
	const int nTriangleCount = GetLedgeTriangleCount( pLedge );
	if ( nTriangleCount <= 0 || !ContainsRange( nByteCount, nLedge + COMPACT_LEDGE_HEADER_SIZE, nTriangleCount * COMPACT_TRIANGLE_SIZE ) )
		return false;

	const ptrdiff_t nPoints = static_cast<ptrdiff_t>( nLedge ) + GetLedgePointOffset( pLedge );
	if ( nPoints < 0 )
		return false;

	const uint32_t nFirst = GetEdgeStartPoint( GetLedgeTriangle( pLedge, 0 ), 0 );
	const uint32_t nWindowBase = nFirst >= MAX_COLLIDE_MAP_SPAN ? nFirst - MAX_COLLIDE_MAP_SPAN : 0;
	uint32_t nMinimum = UINT32_MAX;
	uint32_t nMaximum = 0;
	bool bWindowValid = true;
	std::memset( scan.m_UsedWindow, 0, sizeof( scan.m_UsedWindow ) );
	for ( int i = 0; i < nTriangleCount; ++i )
	{
		const unsigned char *pTriangle = GetLedgeTriangle( pLedge, i );
		for ( int nEdge = 0; nEdge < 3; ++nEdge )
		{
			const uint32_t nPoint = GetEdgeStartPoint( pTriangle, nEdge );
			const size_t nPointOffset = static_cast<size_t>( nPoints ) + static_cast<size_t>( nPoint ) * COMPACT_POINT_SIZE;
			if ( !ContainsRange( nByteCount, nPointOffset, COMPACT_POINT_SIZE ) )
				return false;

			nMinimum = std::min( nMinimum, nPoint );
			nMaximum = std::max( nMaximum, nPoint );

			const uint32_t nWindowIndex = nPoint - nWindowBase;
			if ( nPoint >= nWindowBase && nWindowIndex < 2 * MAX_COLLIDE_MAP_SPAN )
				scan.m_UsedWindow[nWindowIndex] = 1;
			else
				bWindowValid = false;
		}
	}

	scan.m_nMinimum = nMinimum;
	scan.m_nMaximum = nMaximum;
	scan.m_nWindowBase = nWindowBase;
	scan.m_bWindowValid = bWindowValid;
	return true;
}

static bool IsLedgeReadable( const std::vector<unsigned char> &bytes, size_t nLedge )
{
	LedgeScan_t scan;
	return ScanLedge( bytes.data(), bytes.size(), nLedge, scan );
}

static uint16_t FindLedgeSupportEdge( const unsigned char *pLedge, b3Vec3 direction )
{
	const unsigned char *pPoints = pLedge + GetLedgePointOffset( pLedge );
	const auto dot = [&]( uint32_t nPoint ) { return b3Dot( direction, LoadPoint( pPoints + nPoint * COMPACT_POINT_SIZE ) ); };

	size_t nEdge = FIRST_EDGE_OFFSET;
	size_t nTriangle = COMPACT_LEDGE_HEADER_SIZE;
	uint32_t nBest = GetEdgeStartPointAt( pLedge, nEdge );
	float flBest = dot( nBest );

	for ( int n = 0, nCount = GetLedgeTriangleCount( pLedge ); n < nCount; ++n )
	{
		nEdge = GetPreviousEdge( nEdge );
		const uint32_t nFirst = GetEdgeStartPointAt( pLedge, nEdge );
		uint32_t nPoint = nFirst;
		bool bImproved = false;
		do
		{
			const float flDot = dot( nPoint );
			if ( flBest < flDot )
			{
				nTriangle = nEdge & ~static_cast<size_t>( 0xF );
				nBest = nPoint;
				flBest = flDot;
				bImproved = true;
				break;
			}

			nEdge = GetPreviousEdge( GetOppositeEdge( pLedge, nEdge ) );
			nPoint = GetEdgeStartPointAt( pLedge, nEdge );
		} while ( nPoint != nFirst );

		if ( !bImproved && nPoint != nBest )
			break;
	}

	const size_t nTriangleIndex = ( nTriangle - COMPACT_LEDGE_HEADER_SIZE ) / COMPACT_TRIANGLE_SIZE;
	for ( int nIndex = 0; nIndex < 3; ++nIndex )
	{
		if ( GetEdgeStartPointAt( pLedge, nTriangle + 4 + 4 * nIndex ) == nBest )
			return static_cast<uint16_t>( nTriangleIndex * 4 + nIndex );
	}
	return static_cast<uint16_t>( nTriangleIndex * 4 );
}

static CollideMapEntry_t BuildCollideMapEntry( const unsigned char *pLedge, const LedgeScan_t &scan )
{
	CollideMapEntry_t info = {};
	const int nTriangleCount = GetLedgeTriangleCount( pLedge );
	if ( ( GetLedgeFlags( pLedge ) & 3 ) == 0 && nTriangleCount <= MAX_COLLIDE_MAP_TRIANGLES && nTriangleCount > 0 && scan.m_bWindowValid )
	{
		const uint32_t nMinimum = scan.m_nMinimum;
		const int nSpan = static_cast<int>( scan.m_nMaximum - nMinimum ) + 1;
		if ( nSpan > 0 && nSpan < MAX_COLLIDE_MAP_SPAN )
		{
			bool used[MAX_COLLIDE_MAP_SPAN];
			for ( int i = 0; i < nSpan; ++i )
			{
				const uint32_t nWindowIndex = nMinimum - scan.m_nWindowBase + static_cast<uint32_t>( i );
				used[i] = scan.m_UsedWindow[nWindowIndex] != 0;
			}

			int runs[MAX_COLLIDE_MAP_SPAN];
			int nRunCount = 0;
			int nLength = 1;
			for ( int i = 1; i < nSpan; ++i )
			{
				if ( used[i] != used[i - 1] )
				{
					runs[nRunCount++] = nLength;
					nLength = 0;
				}
				++nLength;
			}
			runs[nRunCount++] = nLength;

			if ( nRunCount == 1 )
			{
				info.m_nFlags = COLLIDE_MAP_CONTIGUOUS;
				info.m_nStart = static_cast<uint16_t>( nMinimum );
				info.m_nCount = static_cast<uint16_t>( runs[0] );
			}
			else if ( nRunCount <= MAX_COLLIDE_MAP_RUNS )
			{
				info.m_nFlags = COLLIDE_MAP_RUNS;
				info.m_nStart = static_cast<uint16_t>( nMinimum );
				info.m_nRunCount = static_cast<uint8_t>( nRunCount );
				for ( int i = 0; i < nRunCount && i < MAX_COLLIDE_MAP_RUNS; ++i )
				{
					info.m_Runs[i] = static_cast<uint8_t>( runs[i] );
					if ( i % 2 == 0 )
						info.m_nCount = static_cast<uint16_t>( info.m_nCount + runs[i] );
				}
			}
		}
	}

	if ( !( info.m_nFlags & ( COLLIDE_MAP_CONTIGUOUS | COLLIDE_MAP_RUNS ) ) )
	{
		info.m_nFlags = COLLIDE_MAP_OCTANTS;
		for ( int nOctant = 0; nOctant < 8; ++nOctant )
		{
			const b3Vec3 direction = { nOctant & 1 ? -1.0f : 1.0f, nOctant & 2 ? -1.0f : 1.0f, nOctant & 4 ? -1.0f : 1.0f };
			info.m_OctantEdges[nOctant] = FindLedgeSupportEdge( pLedge, direction );
		}
	}

	return info;
}

std::unique_ptr<CPhysConvex> ReadLedgeConvex( const unsigned char *pLedge )
{
	const int nTriangleCount = GetLedgeTriangleCount( pLedge );
	const unsigned char *pPoints = pLedge + GetLedgePointOffset( pLedge );

	std::vector<b3Vec3> points;
	std::vector<unsigned short> remap;
	points.reserve( static_cast<size_t>( nTriangleCount ) / 2 + 2 );
	ForEachLedgePoint( pLedge, [&]( uint32_t nPoint ) {
		if ( remap.size() <= nPoint )
			remap.resize( nPoint + 1 );
		remap[nPoint] = static_cast<unsigned short>( points.size() );
		points.push_back( IVPToSource( LoadPoint( pPoints + nPoint * COMPACT_POINT_SIZE ) ) );
	} );

	std::vector<CPhysConvex::Triangle_t> triangles( nTriangleCount );
	for ( int i = 0; i < nTriangleCount; ++i )
	{
		const unsigned char *pTriangle = GetLedgeTriangle( pLedge, i );
		CPhysConvex::Triangle_t &triangle = triangles[i];
		triangle.m_nMaterialIndex = static_cast<unsigned char>( GetTriangleMaterial( pTriangle ) );
		for ( int nEdge = 0; nEdge < 3; ++nEdge )
			triangle.m_nVertices[2 - nEdge] = remap[GetEdgeStartPoint( pTriangle, nEdge )];
	}

	auto pConvex = std::make_unique<CPhysConvex>( std::move( points ), std::move( triangles ) );
	pConvex->SetGameData( GetLedgeClientData( pLedge ) );
	return pConvex;
}

struct LedgeTreeReader_t
{
	const unsigned char *m_pSurface;
	size_t m_nSize;
	CPhysCollideCompactSurface::LedgeTree_t &m_Tree;
	std::array<LedgeScan_t, MAX_COLLIDE_MAP_LEDGES> m_Scans;

	bool Read( size_t nNode, int nDepth, int nParent )
	{
		if ( nDepth > MAX_LEDGETREE_DEPTH || !ContainsRange( m_nSize, nNode, COMPACT_LEDGETREE_NODE_SIZE ) )
			return false;

		const unsigned char *pNode = m_pSurface + nNode;
		const size_t nIndex = m_Tree.m_Nodes.size();
		m_Tree.m_Nodes.push_back( { LoadPoint( pNode + 8 ), Load<float>( pNode + 0x14 ), -1, -1, nParent, { pNode[0x18], pNode[0x19], pNode[0x1a] } } );

		const int32_t nRightOffset = Load<int32_t>( pNode );
		const int32_t nLedgeOffset = Load<int32_t>( pNode + 4 );
		if ( nRightOffset == 0 )
		{
			const ptrdiff_t nLedge = static_cast<ptrdiff_t>( nNode ) + nLedgeOffset;
			if ( nLedge < 0 )
				return false;

			LedgeScan_t scan;
			if ( !ScanLedge( m_pSurface, m_nSize, static_cast<size_t>( nLedge ), scan ) )
				return false;

			const size_t nLedgeIndex = m_Tree.m_Ledges.size();
			if ( nLedgeIndex < m_Scans.size() )
				m_Scans[nLedgeIndex] = scan;

			m_Tree.m_Nodes[nIndex].m_nConvex = static_cast<int>( nLedgeIndex );
			const CPhysCollideCompactSurface::LedgeTreeNode_t &node = m_Tree.m_Nodes[nIndex];
			m_Tree.m_Ledges.push_back( { static_cast<size_t>( nLedge ), { node.m_Center, node.m_flRadius * LEAF_SPHERE_MARGIN }, static_cast<int>( nIndex ) } );
			return true;
		}

		if ( nLedgeOffset != 0 )
			m_Tree.m_bHasOuterHull = true;

		if ( !Read( nNode + COMPACT_LEDGETREE_NODE_SIZE, nDepth + 1, static_cast<int>( nIndex ) ) )
			return false;

		m_Tree.m_Nodes[nIndex].m_nRightChild = static_cast<int>( m_Tree.m_Nodes.size() );
		const ptrdiff_t nRight = static_cast<ptrdiff_t>( nNode ) + nRightOffset;
		return nRight >= 0 && Read( static_cast<size_t>( nRight ), nDepth + 1, static_cast<int>( nIndex ) );
	}
};

bool CPhysCollideCompactSurface::ReadLedgeTree( const unsigned char *pSurface, size_t nSize, LedgeTree_t &tree )
{
	if ( nSize < COMPACT_SURFACE_HEADER_SIZE )
		return false;

	const int32_t nRoot = Load<int32_t>( pSurface + SURFACE_LEDGETREE_ROOT_OFFSET );
	if ( nRoot < 0 || static_cast<size_t>( nRoot ) >= nSize )
		return false;

	const size_t nNodeCount = ( nSize - static_cast<size_t>( nRoot ) ) / COMPACT_LEDGETREE_NODE_SIZE;
	tree.m_Nodes.reserve( nNodeCount );
	tree.m_Ledges.reserve( nNodeCount / 2 + 1 );

	LedgeTreeReader_t reader = { pSurface, nSize, tree, {} };
	if ( !reader.Read( static_cast<size_t>( nRoot ), 0, -1 ) )
		return false;

	if ( tree.m_Ledges.size() > MAX_COLLIDE_MAP_LEDGES )
		return true;

	tree.m_CollideMap.reserve( tree.m_Ledges.size() );
	for ( size_t i = 0; i < tree.m_Ledges.size(); ++i )
	{
		CollideMapEntry_t info = BuildCollideMapEntry( pSurface + tree.m_Ledges[i].m_nOffset, reader.m_Scans[i] );
		info.m_nPointBlockOffset = static_cast<uint16_t>( tree.m_nPointBlockCount );
		if ( info.m_nFlags & ( COLLIDE_MAP_CONTIGUOUS | COLLIDE_MAP_RUNS ) )
			tree.m_nPointBlockCount += ( info.m_nCount + 3 ) / 4;
		tree.m_CollideMap.push_back( info );
	}
	return true;
}

CPhysCollideCompactSurface *CreateCollideFromCompactSurface( const std::vector<unsigned char> &surface )
{
	return CPhysCollideCompactSurface::Create( surface.data(), surface.size() );
}

CPhysCollideCompactSurface *CreateCollideFromCompactSurface( const void *pSurface, size_t nSize, bool bSwap )
{
	const auto *pBytes = static_cast<const unsigned char *>( pSurface );
	if ( !bSwap )
		return CPhysCollideCompactSurface::Create( pBytes, nSize );

	std::vector<unsigned char> surface( pBytes, pBytes + nSize );
	if ( !SwapCompactSurfaceFromBigEndian( surface ) )
		return nullptr;

	return CreateCollideFromCompactSurface( surface );
}

CPhysConvex *CreateConvexFromLedge( std::vector<unsigned char> ledge )
{
	if ( !IsLedgeReadable( ledge, 0 ) )
		return nullptr;

	std::unique_ptr<CPhysConvex> pConvex = ReadLedgeConvex( ledge.data() );
	pConvex->SetLedge( std::move( ledge ) );
	return pConvex.release();
}

static b3HullData *CreateHullWithVertexLimit( const b3Vec3 *pPoints, int nPointCount )
{
	int nMaxVertexCount = std::min( nPointCount, B3_MAX_HULL_VERTICES );
	b3HullData *pHull = b3CreateHull( pPoints, nPointCount, nMaxVertexCount );

	while ( !pHull && nMaxVertexCount > 4 )
	{
		nMaxVertexCount = std::max( nMaxVertexCount * 3 / 4, 4 );
		pHull = b3CreateHull( pPoints, nPointCount, nMaxVertexCount );
	}

	return pHull;
}

b3HullData *CPhysConvex::CreateTriangleOffsetHull() const
{
	std::vector<b3Vec3> points;
	points.reserve( m_Triangles.size() * 3 );
	for ( const Triangle_t &triangle : m_Triangles )
	{
		const b3Vec3 a = m_Points[triangle.m_nVertices[0]];
		const b3Vec3 b = m_Points[triangle.m_nVertices[1]];
		const b3Vec3 c = m_Points[triangle.m_nVertices[2]];
		const b3Vec3 normal = b3Cross( b3Sub( c, a ), b3Sub( b, a ) );
		const float flLength = b3Length( normal );
		if ( flLength <= 0.0f )
			continue;

		const b3Vec3 offset = b3MulSV( SHAPE_INFLATION / flLength, normal );
		points.push_back( b3Add( a, offset ) );
		points.push_back( b3Add( b, offset ) );
		points.push_back( b3Add( c, offset ) );
	}

	return points.empty() ? nullptr : CreateHullWithVertexLimit( points.data(), static_cast<int>( points.size() ) );
}

static std::vector<b3Vec3> IntersectPlanes( const std::vector<b3Vec3> &normals, const std::vector<float> &offsets, float flTolerance )
{
	static constexpr float MIN_DETERMINANT = 1e-6f;

	std::vector<b3Vec3> points;
	const size_t nCount = normals.size();
	for ( size_t i = 0; i < nCount; ++i )
	{
		for ( size_t j = i + 1; j < nCount; ++j )
		{
			const b3Vec3 cross = b3Cross( normals[i], normals[j] );
			for ( size_t k = j + 1; k < nCount; ++k )
			{
				const float flDeterminant = b3Dot( cross, normals[k] );
				if ( std::fabs( flDeterminant ) < MIN_DETERMINANT )
					continue;

				const b3Vec3 sum = b3Add( b3Add( b3MulSV( offsets[i], b3Cross( normals[j], normals[k] ) ), b3MulSV( offsets[j], b3Cross( normals[k], normals[i] ) ) ), b3MulSV( offsets[k], cross ) );
				const b3Vec3 point = b3MulSV( 1.0f / flDeterminant, sum );
				bool bInside = true;
				for ( size_t m = 0; m < nCount && bInside; ++m )
					bInside = b3Dot( normals[m], point ) <= offsets[m] + flTolerance;
				if ( bInside )
					points.push_back( point );
			}
		}
	}
	return points;
}

b3HullData *CPhysConvex::CreateHull( b3HullData *pBase ) const
{
	static constexpr float INTERSECTION_TOLERANCE = 1e-3f;
	static constexpr float MAX_HULL_GROWTH = 2.0f * SHAPE_INFLATION;

	const b3Vec3 center = pBase->center;
	const b3Plane *pPlanes = b3GetHullPlanes( pBase );
	float flScale = 0.0f;
	for ( const b3Vec3 &point : m_Points )
		flScale = std::max( flScale, b3DistanceSquared( point, center ) );

	std::vector<b3Vec3> normals, dual;
	std::vector<float> offsets;
	const auto AddPlane = [&]( b3Vec3 normal ) {
		float flExtent = 0.0f;
		for ( const b3Vec3 &point : m_Points )
			flExtent = std::max( flExtent, b3Dot( normal, b3Sub( point, center ) ) );
		normals.push_back( normal );
		offsets.push_back( flExtent + SHAPE_INFLATION );
		dual.push_back( b3MulSV( flScale / offsets.back(), normal ) );
	};
	for ( int i = 0; i < pBase->faceCount; ++i )
		AddPlane( pPlanes[i].normal );

	const b3Vec3 *pBasePoints = b3GetHullPoints( pBase );
	std::vector<b3Vec3> basePoints( pBase->vertexCount );
	for ( int i = 0; i < pBase->vertexCount; ++i )
		basePoints[i] = b3Sub( pBasePoints[i], center );
	b3DestroyHull( pBase );

	b3DistanceInput input = {};
	input.proxyA = { basePoints.data(), static_cast<int>( basePoints.size() ), 0.0f };
	input.transform = b3Transform_identity;

	const float flTolerance = INTERSECTION_TOLERANCE * std::sqrt( flScale );
	std::vector<b3Vec3> points, previousPoints;
	for ( ;; )
	{
		b3HullData *pDual = b3CreateHull( dual.data(), static_cast<int>( dual.size() ), B3_MAX_HULL_VERTICES );
		if ( !pDual )
		{
			points = previousPoints.empty() ? IntersectPlanes( normals, offsets, flTolerance ) : previousPoints;
			break;
		}

		const b3Plane *pDualPlanes = b3GetHullPlanes( pDual );
		points.resize( pDual->faceCount );
		for ( int i = 0; i < pDual->faceCount; ++i )
			points[i] = b3MulSV( flScale / pDualPlanes[i].offset, pDualPlanes[i].normal );
		b3DestroyHull( pDual );

		if ( !previousPoints.empty() )
		{
			bool bApplied = true;
			for ( const b3Vec3 &point : points )
				bApplied = bApplied && b3Dot( normals.back(), point ) <= offsets.back() + flTolerance;
			if ( !bApplied )
			{
				points = previousPoints;
				break;
			}
		}

		float flWorst = 0.0f;
		b3Vec3 normal = b3Vec3_zero;
		b3Vec3 worst = b3Vec3_zero;
		for ( const b3Vec3 &point : points )
		{
			input.proxyB = { &point, 1, 0.0f };
			b3SimplexCache cache = {};
			const b3DistanceOutput output = b3ShapeDistance( &input, &cache, nullptr, 0 );
			if ( output.distance <= flWorst )
				continue;

			flWorst = output.distance;
			normal = b3Normalize( b3Sub( point, output.pointA ) );
			worst = point;
		}
		if ( flWorst <= MAX_HULL_GROWTH || points.size() > B3_MAX_HULL_VERTICES )
			break;

		AddPlane( normal );
		if ( b3Dot( normal, worst ) <= offsets.back() + flTolerance )
		{
			normals.pop_back();
			offsets.pop_back();
			dual.pop_back();
			break;
		}
		previousPoints = points;
	}

	for ( b3Vec3 &point : points )
		point = b3Add( point, center );
	return points.empty() ? CreateTriangleOffsetHull() : CreateHullWithVertexLimit( points.data(), static_cast<int>( points.size() ) );
}

CPhysConvex::CPhysConvex( std::vector<b3Vec3> points, std::vector<Triangle_t> triangles )
: m_Points( std::move( points ) )
, m_Triangles( std::move( triangles ) )
{
}

void CPhysConvex::SetLedgeNode( const CPhysCollideCompactSurface *pSurface, int nNode, bool bDetached )
{
	m_pLedgeSurface = pSurface;
	m_nLedgeNode = nNode;
	m_bDetached = bDetached;
}

bool CPhysConvex::IsReachable( b3Vec3 center, float flRadius ) const
{
	return m_pLedgeSurface->IsLedgeReachable( m_nLedgeNode, center, flRadius );
}

bool CPhysCollideCompactSurface::IsLedgeReachable( int nNode, b3Vec3 center, float flRadius ) const
{
	for ( ; nNode >= 0; nNode = m_Nodes[nNode].m_nParent )
	{
		const LedgeTreeNode_t &node = m_Nodes[nNode];
		const b3Vec3 delta = b3Sub( node.m_Center, center );
		const float flReach = node.m_flRadius + flRadius;
		if ( delta.y * delta.y + delta.x * delta.x + delta.z * delta.z > flReach * flReach )
			return false;

		const float flScale = node.m_flRadius * QUANTIZED_RADIUS_SCALE;
		if ( static_cast<float>( node.m_BoxSizes[0] ) * flScale + flRadius <= std::fabs( delta.x ) || static_cast<float>( node.m_BoxSizes[1] ) * flScale + flRadius <= std::fabs( delta.y ) || static_cast<float>( node.m_BoxSizes[2] ) * flScale + flRadius <= std::fabs( delta.z ) )
			return false;
	}
	return true;
}

static std::mutex s_LazyBuildMutex;

static bool IsCoplanar( const std::vector<b3Vec3> &points, const std::vector<CPhysConvex::Triangle_t> &triangles )
{
	b3Vec3 extent = b3Vec3_zero;
	for ( const b3Vec3 &point : points )
		extent = b3Max( extent, b3Abs( point ) );

	const float flMaxCoordinate = std::max( { extent.x, extent.y, extent.z } );
	const float flMaxDistance = std::min( std::sqrt( 3.0f ) * flMaxCoordinate, extent.x + extent.y + extent.z );
	const float flTolerance = ( 3.0f * flMaxDistance * 1.01f + flMaxCoordinate ) * FLT_EPSILON;

	b3Vec3 normal = b3Vec3_zero;
	b3Vec3 origin = b3Vec3_zero;
	float flBestArea = 0.0f;
	for ( const CPhysConvex::Triangle_t &triangle : triangles )
	{
		const b3Vec3 a = points[triangle.m_nVertices[0]];
		const b3Vec3 cross = b3Cross( b3Sub( points[triangle.m_nVertices[1]], a ), b3Sub( points[triangle.m_nVertices[2]], a ) );
		const float flArea = b3LengthSquared( cross );
		if ( flArea <= flBestArea )
			continue;

		flBestArea = flArea;
		normal = cross;
		origin = a;
	}
	if ( flBestArea <= 0.0f )
		return true;

	normal = b3Normalize( normal );
	for ( const b3Vec3 &point : points )
	{
		if ( std::fabs( b3Dot( normal, b3Sub( point, origin ) ) ) > 2.0f * flTolerance )
			return false;
	}
	return true;
}

void CPhysConvex::ClassifyHull() const
{
	std::lock_guard lock( s_LazyBuildMutex );
	if ( m_nHullState.load( std::memory_order_relaxed ) != HULL_UNCLASSIFIED )
		return;

	b3HullData *pBase = IsCoplanar( m_Points, m_Triangles ) ? nullptr : CreateHullWithVertexLimit( m_Points.data(), static_cast<int>( m_Points.size() ) );
	m_bFlat = !pBase;
	if ( pBase )
		m_pHull = CreateHull( pBase );
	m_nHullState.store( pBase ? HULL_BUILT : HULL_FLAT, std::memory_order_release );
}

void CPhysConvex::BuildHull() const
{
	if ( !IsFlat() )
		return;

	std::lock_guard lock( s_LazyBuildMutex );
	if ( m_nHullState.load( std::memory_order_relaxed ) == HULL_BUILT )
		return;

	m_pHull = CreateTriangleOffsetHull();
	m_nHullState.store( HULL_BUILT, std::memory_order_release );
}

void CPhysConvex::SetGameData( unsigned int nGameData )
{
	m_nGameData = nGameData;
	if ( m_Ledge.size() >= COMPACT_LEDGE_HEADER_SIZE )
		Store<uint32_t>( m_Ledge.data() + 4, nGameData );
}

CPhysConvex::~CPhysConvex()
{
	if ( m_pHull )
		b3DestroyHull( m_pHull );
}

CollideMesh_t::~CollideMesh_t()
{
	if ( m_pData )
		b3DestroyMesh( m_pData );
}

static bool GetMeshTriangleNormal( const CPhysConvex &convex, const CPhysConvex::Triangle_t &triangle, b3Vec3 &normal )
{
	const std::vector<b3Vec3> &points = convex.GetPoints();
	const b3Vec3 a = points[triangle.m_nVertices[0]];
	float flLength = 0.0f;
	normal = b3GetLengthAndNormalize( &flLength, b3Cross( b3Sub( points[triangle.m_nVertices[2]], a ), b3Sub( points[triangle.m_nVertices[1]], a ) ) );
	return 0.5f * flLength >= MIN_MESH_TRIANGLE_AREA_SCALE * B3_LINEAR_SLOP * B3_LINEAR_SLOP;
}

struct StaticMeshTriangle_t
{
	int m_nConvex;
	int m_nTriangle;
	b3Vec3 m_Normal;
};

static std::unique_ptr<CollideMesh_t> CreateStaticMesh( const std::vector<std::unique_ptr<CPhysConvex>> &convexes, int nMaterial, const std::vector<StaticMeshTriangle_t> &triangles )
{
	static constexpr float FLAT_EDGE_COSINE = 0.9962f;

	using PointTable_t = CVectorHash<3>;

	struct Cluster_t
	{
		b3Vec3 m_Normal;
		b3Vec3 m_Sum;
		int m_nPoint;
		int m_nNext;
	};

	std::vector<b3Vec3> points;
	std::vector<int> heads;
	std::vector<Cluster_t> clusters;
	PointTable_t table( 3 * triangles.size() );
	auto findCluster = [&]( b3Vec3 point, b3Vec3 normal ) {
		point = { point.x + 0.0f, point.y + 0.0f, point.z + 0.0f };
		const auto [nPoint, bInserted] = table.FindOrInsert( BitCast<PointTable_t::Key_t>( point ), static_cast<uint32_t>( points.size() ), [&]( uint32_t nIndex ) { return BitCast<PointTable_t::Key_t>( points[nIndex] ); } );
		if ( bInserted )
		{
			points.push_back( point );
			heads.push_back( -1 );
		}

		for ( int nCluster = heads[nPoint]; nCluster >= 0; nCluster = clusters[nCluster].m_nNext )
		{
			if ( b3Dot( clusters[nCluster].m_Normal, normal ) > FLAT_EDGE_COSINE )
			{
				clusters[nCluster].m_Sum = b3Add( clusters[nCluster].m_Sum, normal );
				return nCluster;
			}
		}

		clusters.push_back( Cluster_t{ normal, normal, static_cast<int>( nPoint ), heads[nPoint] } );
		heads[nPoint] = static_cast<int>( clusters.size() ) - 1;
		return heads[nPoint];
	};

	std::unique_ptr<CollideMesh_t> pMesh = std::make_unique<CollideMesh_t>();
	pMesh->m_nMaterial = nMaterial;
	pMesh->m_Sources.reserve( triangles.size() );
	std::vector<int32_t> indices;
	indices.reserve( 3 * triangles.size() );
	for ( const StaticMeshTriangle_t &entry : triangles )
	{
		const CPhysConvex &convex = *convexes[entry.m_nConvex];
		const std::vector<b3Vec3> &convexPoints = convex.GetPoints();
		const CPhysConvex::Triangle_t &triangle = convex.GetTriangles()[entry.m_nTriangle];
		const b3Vec3 a = convexPoints[triangle.m_nVertices[0]];
		const b3Vec3 b = convexPoints[triangle.m_nVertices[1]];
		const b3Vec3 c = convexPoints[triangle.m_nVertices[2]];
		indices.insert( indices.end(), { findCluster( a, entry.m_Normal ), findCluster( c, entry.m_Normal ), findCluster( b, entry.m_Normal ) } );
		pMesh->m_Sources.push_back( { entry.m_nConvex, entry.m_nTriangle } );
	}

	const float flOffset = SHAPE_INFLATION - B3_MESH_REST_OFFSET;
	std::vector<b3Vec3> vertices( clusters.size() );
	for ( size_t i = 0; i < clusters.size(); ++i )
		vertices[i] = b3MulAdd( points[clusters[i].m_nPoint], flOffset, b3Normalize( clusters[i].m_Sum ) );

	b3MeshDef def = {};
	def.vertices = vertices.data();
	def.indices = indices.data();
	def.vertexCount = static_cast<int>( vertices.size() );
	def.triangleCount = static_cast<int>( pMesh->m_Sources.size() );
	def.identifyEdges = true;
	pMesh->m_pData = b3CreateMesh( &def, nullptr, 0 );
	if ( !pMesh->m_pData )
		return nullptr;

	using Key_t = std::array<int32_t, 3>;
	std::vector<std::pair<Key_t, int>> order( pMesh->m_Sources.size() );
	for ( size_t i = 0; i < order.size(); ++i )
		order[i] = { { indices[3 * i], indices[3 * i + 1], indices[3 * i + 2] }, static_cast<int>( i ) };
	std::sort( order.begin(), order.end() );

	const b3MeshTriangle *pTriangles = b3GetMeshTriangles( pMesh->m_pData );
	std::vector<CollideMesh_t::Source_t> sources( pMesh->m_pData->triangleCount );
	std::vector<bool> used( order.size(), false );
	for ( size_t i = 0; i < sources.size(); ++i )
	{
		const Key_t key = { pTriangles[i].index1, pTriangles[i].index2, pTriangles[i].index3 };
		auto it = std::lower_bound( order.begin(), order.end(), key, []( const std::pair<Key_t, int> &entry, const Key_t &value ) { return entry.first < value; } );
		while ( used[it - order.begin()] )
			++it;
		used[it - order.begin()] = true;
		sources[i] = pMesh->m_Sources[it->second];
	}
	pMesh->m_Sources = std::move( sources );
	return pMesh;
}

static std::vector<std::unique_ptr<CollideMesh_t>> CreateStaticMeshes( const std::vector<std::unique_ptr<CPhysConvex>> &convexes )
{
	std::array<std::vector<StaticMeshTriangle_t>, 256> buckets;
	for ( size_t nConvex = 0; nConvex < convexes.size(); ++nConvex )
	{
		const CPhysConvex &convex = *convexes[nConvex];
		if ( !convex.IsFlat() || convex.IsDetached() )
			continue;

		const std::vector<CPhysConvex::Triangle_t> &triangles = convex.GetTriangles();
		for ( size_t nTriangle = 0; nTriangle < triangles.size(); ++nTriangle )
		{
			b3Vec3 normal;
			if ( GetMeshTriangleNormal( convex, triangles[nTriangle], normal ) )
				buckets[triangles[nTriangle].m_nMaterialIndex].push_back( { static_cast<int>( nConvex ), static_cast<int>( nTriangle ), normal } );
		}
	}

	std::vector<int> materials;
	for ( int nMaterial = 0; nMaterial < static_cast<int>( buckets.size() ); ++nMaterial )
	{
		if ( !buckets[nMaterial].empty() )
			materials.push_back( nMaterial );
	}

	std::vector<std::future<std::unique_ptr<CollideMesh_t>>> pending;
	for ( size_t i = 1; i < materials.size(); ++i )
		pending.push_back( std::async( std::launch::async, CreateStaticMesh, std::cref( convexes ), materials[i], std::cref( buckets[materials[i]] ) ) );

	std::vector<std::unique_ptr<CollideMesh_t>> meshes;
	if ( !materials.empty() )
		meshes.push_back( CreateStaticMesh( convexes, materials[0], buckets[materials[0]] ) );
	for ( std::future<std::unique_ptr<CollideMesh_t>> &future : pending )
		meshes.push_back( future.get() );

	if ( std::any_of( meshes.begin(), meshes.end(), []( const std::unique_ptr<CollideMesh_t> &pMesh ) { return !pMesh; } ) )
		return {};
	return meshes;
}

static std::mutex s_StaticMeshMutex;

void CPhysCollide::BuildStaticMeshes() const
{
	std::lock_guard lock( s_StaticMeshMutex );
	if ( m_bStaticMeshesBuilt.load( std::memory_order_relaxed ) )
		return;

	m_StaticMeshes = CreateStaticMeshes( GetConvexes() );
	m_bStaticMeshesBuilt.store( true, std::memory_order_release );
}

int CPhysConvex::GetUniformMaterial() const
{
	if ( m_Triangles.empty() )
		return 0;

	const int nMaterial = m_Triangles.front().m_nMaterialIndex;
	for ( const Triangle_t &triangle : m_Triangles )
	{
		if ( triangle.m_nMaterialIndex != nMaterial )
			return 0;
	}
	return nMaterial;
}

int CPhysConvex::FindTriangleMaterial( b3Vec3 localNormal, b3Vec3 localPoint ) const
{
	static constexpr float COPLANAR_TOLERANCE = 1e-4f;

	if ( m_TriangleNormals.size() != m_Triangles.size() )
	{
		m_TriangleNormals.clear();
		for ( const Triangle_t &triangle : m_Triangles )
		{
			const b3Vec3 a = m_Points[triangle.m_nVertices[0]];
			const b3Vec3 normal = b3Cross( b3Sub( m_Points[triangle.m_nVertices[2]], a ), b3Sub( m_Points[triangle.m_nVertices[1]], a ) );
			const float flLength = b3Length( normal );
			m_TriangleNormals.push_back( flLength > 0.0f ? b3MulSV( 1.0f / flLength, normal ) : b3Vec3_zero );
		}
	}

	float flBest = -FLT_MAX;
	for ( const b3Vec3 &normal : m_TriangleNormals )
		flBest = std::max( flBest, b3Dot( normal, localNormal ) );

	int nFallback = -1;
	for ( size_t i = 0; i < m_Triangles.size(); ++i )
	{
		const b3Vec3 &normal = m_TriangleNormals[i];
		if ( b3Dot( normal, localNormal ) < flBest - COPLANAR_TOLERANCE )
			continue;

		if ( nFallback < 0 )
			nFallback = static_cast<int>( i );

		const Triangle_t &triangle = m_Triangles[i];
		bool bInside = true;
		for ( int nEdge = 0; nEdge < 3 && bInside; ++nEdge )
		{
			const b3Vec3 start = m_Points[triangle.m_nVertices[( nEdge + 1 ) % 3]];
			const b3Vec3 end = m_Points[triangle.m_nVertices[nEdge]];
			bInside = b3Dot( b3Cross( b3Sub( end, start ), b3Sub( localPoint, start ) ), normal ) >= 0.0f;
		}

		if ( bInside )
			return triangle.m_nMaterialIndex;
	}

	return nFallback >= 0 ? m_Triangles[nFallback].m_nMaterialIndex : 0;
}

float CPhysConvex::GetVolume() const
{
	if ( m_Triangles.empty() )
		return 0.0f;

	const Vector origin = ToSource( m_Points[m_Triangles.front().m_nVertices[0]] );

	float flVolume = 0.0f;
	for ( const Triangle_t &triangle : m_Triangles )
	{
		const Vector a = ToSource( m_Points[triangle.m_nVertices[0]] ) - origin;
		const Vector b = ToSource( m_Points[triangle.m_nVertices[1]] ) - origin;
		const Vector c = ToSource( m_Points[triangle.m_nVertices[2]] ) - origin;
		flVolume += std::fabs( a.Dot( b.Cross( c ) ) ) * ( 1.0f / 6.0f );
	}
	return flVolume;
}

float CPhysConvex::GetSurfaceArea() const
{
	float flArea = 0.0f;
	for ( const Triangle_t &triangle : m_Triangles )
	{
		const Vector a = ToSource( m_Points[triangle.m_nVertices[0]] );
		const Vector ab = ToSource( m_Points[triangle.m_nVertices[1]] ) - a;
		const Vector ac = ToSource( m_Points[triangle.m_nVertices[2]] ) - a;
		flArea += ab.Cross( ac ).Length() * 0.5f;
	}
	return flArea;
}

static size_t AlignBlockSize( size_t nBytes )
{
	return ( nBytes + 15 ) & ~static_cast<size_t>( 15 );
}

template<typename T>
static size_t GetArrayBlockSize( const std::vector<T> &source )
{
	return AlignBlockSize( source.size() * sizeof( T ) );
}

template<typename T>
static T *PlaceArray( unsigned char *&pCursor, const std::vector<T> &source )
{
	T *pArray = reinterpret_cast<T *>( pCursor );
	if ( !source.empty() )
		std::memcpy( pArray, source.data(), source.size() * sizeof( T ) );
	pCursor += GetArrayBlockSize( source );
	return pArray;
}

CPhysCollideCompactSurface *CPhysCollideCompactSurface::Create( const unsigned char *pSurface, size_t nSize )
{
	thread_local LedgeTree_t s_Tree;
	LedgeTree_t &tree = s_Tree;
	tree.Clear();
	if ( !ReadLedgeTree( pSurface, nSize, tree ) )
		return nullptr;

	const size_t nObjectSize = AlignBlockSize( sizeof( CPhysCollideCompactSurface ) );
	const size_t nBlockSize = nObjectSize + GetArrayBlockSize( tree.m_Nodes ) + GetArrayBlockSize( tree.m_Ledges ) + GetArrayBlockSize( tree.m_CollideMap ) + nSize;
	unsigned char *pBlock = static_cast<unsigned char *>( ::operator new( nBlockSize ) );
	CPhysCollideCompactSurface *pCollide = new ( pBlock ) CPhysCollideCompactSurface;

	unsigned char *pCursor = pBlock + nObjectSize;
	pCollide->m_Nodes = { PlaceArray( pCursor, tree.m_Nodes ), tree.m_Nodes.size() };
	pCollide->m_Ledges = { PlaceArray( pCursor, tree.m_Ledges ), tree.m_Ledges.size() };
	pCollide->m_pCollideMap = tree.m_CollideMap.empty() ? nullptr : PlaceArray( pCursor, tree.m_CollideMap );
	pCollide->m_nPointBlockCount = tree.m_nPointBlockCount;
	pCollide->m_bHasOuterHull = tree.m_bHasOuterHull;
	pCollide->m_pSurface = pCursor;
	std::memcpy( pCursor, pSurface, nSize );
	return pCollide;
}

void CPhysCollideCompactSurface::BuildConvexes() const
{
	std::lock_guard lock( s_LazyBuildMutex );
	if ( m_bConvexesBuilt.load( std::memory_order_relaxed ) )
		return;

	m_Convexes.reserve( m_Ledges.size() );
	for ( const Ledge_t &ledge : m_Ledges )
	{
		std::unique_ptr<CPhysConvex> pConvex = ReadLedgeConvex( m_pSurface + ledge.m_nOffset );
		const LedgeTreeNode_t &node = m_Nodes[ledge.m_nNode];
		const b3Vec3 center = IVPToSource( node.m_Center );
		const float flLimit = node.m_flRadius * IVP_TO_SOURCE * LEAF_SPHERE_MARGIN;
		const bool bDetached = std::any_of( pConvex->GetPoints().begin(), pConvex->GetPoints().end(), [&]( b3Vec3 point ) { return b3Length( b3Sub( point, center ) ) > flLimit; } );
		pConvex->SetLedgeNode( this, ledge.m_nNode, bDetached );
		m_Convexes.push_back( std::move( pConvex ) );
	}
	m_bConvexesBuilt.store( true, std::memory_order_release );
}

const std::vector<std::unique_ptr<CPhysConvex>> &CPhysCollideCompactSurface::GetConvexes() const
{
	if ( !m_bConvexesBuilt.load( std::memory_order_acquire ) )
		BuildConvexes();
	return m_Convexes;
}

int CPhysCollideCompactSurface::GetSurfaceSize() const
{
	return static_cast<int>( Load<uint32_t>( m_pSurface + SURFACE_BYTE_SIZE_OFFSET ) >> 8 );
}

Vector CPhysCollideCompactSurface::GetMassCenter() const
{
	return ToSource( IVPToSource( LoadPoint( m_pSurface + SURFACE_MASS_CENTER_OFFSET ) ) );
}

void CPhysCollideCompactSurface::SetMassCenter( const Vector &massCenter )
{
	Store( m_pSurface + SURFACE_MASS_CENTER_OFFSET, SourceToIVP( massCenter ) );
}

b3Vec3 CPhysCollideCompactSurface::GetIVPMassCenter() const
{
	return LoadPoint( m_pSurface + SURFACE_MASS_CENTER_OFFSET );
}

Vector CPhysCollideCompactSurface::GetRotationInertia() const
{
	const unsigned char *pInertia = m_pSurface + SURFACE_ROTATION_INERTIA_OFFSET;
	return Vector( Load<float>( pInertia ), Load<float>( pInertia + 8 ), Load<float>( pInertia + 4 ) );
}

float CPhysCollideCompactSurface::GetUpperLimitRadius() const
{
	return Load<float>( m_pSurface + SURFACE_UPPER_LIMIT_RADIUS_OFFSET );
}

float CPhysCollideCompactSurface::GetMaxDeviation() const
{
	return static_cast<float>( m_pSurface[SURFACE_BYTE_SIZE_OFFSET] ) * QUANTIZED_RADIUS_SCALE * GetUpperLimitRadius();
}

float CPhysCollideCompactSurface::GetBoundingRadius() const
{
	const b3Vec3 center = LoadPoint( m_pSurface + SURFACE_MASS_CENTER_OFFSET );
	return ( std::sqrt( b3LengthSquared( center ) + FLT_EPSILON ) + GetUpperLimitRadius() ) * IVP_TO_SOURCE;
}

int CPhysCollideCompactSurface::GetIndex() const
{
	return Load<int32_t>( m_pSurface + SURFACE_INDEX_OFFSET );
}

void CPhysCollideCompactSurface::SetIndex( int nIndex )
{
	Store<int32_t>( m_pSurface + SURFACE_INDEX_OFFSET, nIndex );
}

void CPhysCollideCompactSurface::SetTriangleMaterial( int nConvex, int nTriangle, int nMaterial )
{
	::SetTriangleMaterial( GetLedgeTriangle( m_pSurface + m_Ledges[nConvex].m_nOffset, nTriangle ), static_cast<uint32_t>( nMaterial ) );
	if ( m_bConvexesBuilt.load( std::memory_order_acquire ) )
		m_Convexes[nConvex]->GetTriangles()[nTriangle].m_nMaterialIndex = static_cast<unsigned char>( nMaterial & 0x7F );
}

static Vector LoadLedgeVertex( const unsigned char *pPoints, uint32_t nPoint )
{
	return ToSource( IVPToSource( LoadPoint( pPoints + nPoint * COMPACT_POINT_SIZE ) ) );
}

float CPhysCollideCompactSurface::GetVolume() const
{
	float flVolume = 0.0f;
	for ( const Ledge_t &ledge : m_Ledges )
	{
		const unsigned char *pLedge = m_pSurface + ledge.m_nOffset;
		const unsigned char *pPoints = pLedge + GetLedgePointOffset( pLedge );
		const Vector origin = LoadLedgeVertex( pPoints, GetEdgeStartPoint( GetLedgeTriangle( pLedge, 0 ), 2 ) );

		float flLedgeVolume = 0.0f;
		for ( int i = 0, nCount = GetLedgeTriangleCount( pLedge ); i < nCount; ++i )
		{
			const unsigned char *pTriangle = GetLedgeTriangle( pLedge, i );
			const Vector a = LoadLedgeVertex( pPoints, GetEdgeStartPoint( pTriangle, 2 ) ) - origin;
			const Vector b = LoadLedgeVertex( pPoints, GetEdgeStartPoint( pTriangle, 1 ) ) - origin;
			const Vector c = LoadLedgeVertex( pPoints, GetEdgeStartPoint( pTriangle, 0 ) ) - origin;
			flLedgeVolume += std::fabs( a.Dot( b.Cross( c ) ) ) * ( 1.0f / 6.0f );
		}
		flVolume += flLedgeVolume;
	}
	return flVolume;
}

float CPhysCollideCompactSurface::GetSurfaceArea() const
{
	float flArea = 0.0f;
	for ( const Ledge_t &ledge : m_Ledges )
	{
		const unsigned char *pLedge = m_pSurface + ledge.m_nOffset;
		const unsigned char *pPoints = pLedge + GetLedgePointOffset( pLedge );

		float flLedgeArea = 0.0f;
		for ( int i = 0, nCount = GetLedgeTriangleCount( pLedge ); i < nCount; ++i )
		{
			const unsigned char *pTriangle = GetLedgeTriangle( pLedge, i );
			const Vector a = LoadLedgeVertex( pPoints, GetEdgeStartPoint( pTriangle, 2 ) );
			const Vector ab = LoadLedgeVertex( pPoints, GetEdgeStartPoint( pTriangle, 1 ) ) - a;
			const Vector ac = LoadLedgeVertex( pPoints, GetEdgeStartPoint( pTriangle, 0 ) ) - a;
			flLedgeArea += ab.Cross( ac ).Length() * 0.5f;
		}
		flArea += flLedgeArea;
	}
	return flArea;
}

void CPhysCollideCompactSurface::GetAABB( Vector *pMins, Vector *pMaxs, const b3Transform &transform ) const
{
	b3Vec3 mins = { FLT_MAX, FLT_MAX, FLT_MAX };
	b3Vec3 maxs = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

	for ( const Ledge_t &ledge : m_Ledges )
	{
		const unsigned char *pLedge = m_pSurface + ledge.m_nOffset;
		const unsigned char *pPoints = pLedge + GetLedgePointOffset( pLedge );
		ForEachLedgePoint( pLedge, [&]( uint32_t nPoint ) {
			const b3Vec3 point = b3TransformPoint( transform, IVPToSource( LoadPoint( pPoints + nPoint * COMPACT_POINT_SIZE ) ) );
			mins = b3Min( mins, point );
			maxs = b3Max( maxs, point );
		} );
	}

	*pMins = ToSource( mins );
	*pMaxs = ToSource( maxs );
}

Vector CPhysCollideCompactSurface::GetExtent( const b3Transform &transform, const Vector &direction ) const
{
	Vector extent = Vector( 0.0f, 0.0f, 0.0f );
	float flBestDistance = -1e6f;

	for ( const Ledge_t &ledge : m_Ledges )
	{
		const unsigned char *pLedge = m_pSurface + ledge.m_nOffset;
		const unsigned char *pPoints = pLedge + GetLedgePointOffset( pLedge );
		ForEachLedgePoint( pLedge, [&]( uint32_t nPoint ) {
			const Vector point = ToSource( b3TransformPoint( transform, IVPToSource( LoadPoint( pPoints + nPoint * COMPACT_POINT_SIZE ) ) ) );
			const float flDistance = point.Dot( direction );
			if ( flDistance > flBestDistance )
			{
				flBestDistance = flDistance;
				extent = point;
			}
		} );
	}

	return extent;
}

float CPhysCollide::GetVolume() const
{
	float flVolume = 0.0f;
	for ( const std::unique_ptr<CPhysConvex> &pConvex : GetConvexes() )
		flVolume += pConvex->GetVolume();
	return flVolume;
}

float CPhysCollide::GetSurfaceArea() const
{
	float flArea = 0.0f;
	for ( const std::unique_ptr<CPhysConvex> &pConvex : GetConvexes() )
		flArea += pConvex->GetSurfaceArea();
	return flArea;
}

void CPhysCollide::GetAABB( Vector *pMins, Vector *pMaxs, const b3Transform &transform ) const
{
	b3Vec3 mins = { FLT_MAX, FLT_MAX, FLT_MAX };
	b3Vec3 maxs = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

	for ( const std::unique_ptr<CPhysConvex> &pConvex : GetConvexes() )
	{
		for ( b3Vec3 localPoint : pConvex->GetPoints() )
		{
			const b3Vec3 point = b3TransformPoint( transform, localPoint );
			mins = b3Min( mins, point );
			maxs = b3Max( maxs, point );
		}
	}

	*pMins = ToSource( mins );
	*pMaxs = ToSource( maxs );
}

Vector CPhysCollide::GetExtent( const b3Transform &transform, const Vector &direction ) const
{
	Vector extent = Vector( 0.0f, 0.0f, 0.0f );
	float flBestDistance = -1e6f;

	for ( const std::unique_ptr<CPhysConvex> &pConvex : GetConvexes() )
	{
		for ( b3Vec3 localPoint : pConvex->GetPoints() )
		{
			const Vector point = ToSource( b3TransformPoint( transform, localPoint ) );
			const float flDistance = point.Dot( direction );
			if ( flDistance > flBestDistance )
			{
				flBestDistance = flDistance;
				extent = point;
			}
		}
	}

	return extent;
}

int CPhysCollideCompactSurface::GetSerializationSize() const
{
	return GetSurfaceSize() + COLLIDE_HEADER_SIZE;
}

void CPhysCollideCompactSurface::Serialize( char *pDest, bool bSwap ) const
{
	const int nSurfaceSize = GetSurfaceSize();

	physcollideheader_t collideHeader;
	collideHeader.vphysicsID = MAKEID( 'V', 'P', 'H', 'Y' );
	collideHeader.version = 0x100;
	collideHeader.modelType = MODEL_TYPE_COMPACT_SURFACE;

	compactsurfaceheader_t surfaceHeader;
	surfaceHeader.surfaceSize = nSurfaceSize;
	for ( int i = 0; i < 3; ++i )
		surfaceHeader.dragAxisAreas[i] = m_OrthographicAreas[i];
	surfaceHeader.axisMapSize = 0;

	if ( bSwap )
	{
		collideHeader.vphysicsID = static_cast<int>( ByteSwap( static_cast<uint32_t>( collideHeader.vphysicsID ) ) );
		collideHeader.version = static_cast<short>( ByteSwap( static_cast<uint16_t>( collideHeader.version ) ) );
		collideHeader.modelType = static_cast<short>( ByteSwap( static_cast<uint16_t>( collideHeader.modelType ) ) );
		surfaceHeader.surfaceSize = static_cast<int>( ByteSwap( static_cast<uint32_t>( surfaceHeader.surfaceSize ) ) );
		for ( int i = 0; i < 3; ++i )
			surfaceHeader.dragAxisAreas[i] = BitCast<float>( ByteSwap( BitCast<uint32_t>( surfaceHeader.dragAxisAreas[i] ) ) );
	}

	std::memcpy( pDest, &collideHeader, sizeof( collideHeader ) );
	std::memcpy( pDest + sizeof( collideHeader ), &surfaceHeader, sizeof( surfaceHeader ) );
	if ( !bSwap )
	{
		std::memcpy( pDest + COLLIDE_HEADER_SIZE, m_pSurface, static_cast<size_t>( nSurfaceSize ) );
		return;
	}

	std::vector<unsigned char> surface( m_pSurface, m_pSurface + nSurfaceSize );
	SwapCompactSurfaceToBigEndian( surface );
	std::memcpy( pDest + COLLIDE_HEADER_SIZE, surface.data(), surface.size() );
}

void CPhysCollideCompactSurface::OutputDebugInfo() const
{
	Msg( "CollisionModel: Compact Surface: %d convex pieces %s outer hull\n", GetLedgeCount(), m_bHasOuterHull ? "with" : "no" );
}
