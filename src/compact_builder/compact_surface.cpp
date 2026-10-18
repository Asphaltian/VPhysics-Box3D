#include "compact_surface.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <set>
#include <span>

static constexpr float PIERCE_DOT_LIMIT = -1e-6f;
static constexpr float TRIANGLE_AREA_EPSILON = 1e-12f;

static uint32_t SwapTriangleHeaderToBigEndian( uint32_t nValue )
{
	const uint32_t nTriangle = nValue & 0xFFF;
	const uint32_t nPierce = ( nValue >> 12 ) & 0xFFF;
	const uint32_t nMaterial = ( nValue >> 24 ) & 0x7F;
	const uint32_t nVirtual = nValue >> 31;
	return ByteSwap( ( nTriangle << 20 ) | ( nPierce << 8 ) | ( nMaterial << 1 ) | nVirtual );
}

static uint32_t SwapTriangleHeaderFromBigEndian( uint32_t nValue )
{
	const uint32_t nPacked = ByteSwap( nValue );
	return ( nPacked >> 20 ) | ( ( ( nPacked >> 8 ) & 0xFFF ) << 12 ) | ( ( ( nPacked >> 1 ) & 0x7F ) << 24 ) | ( ( nPacked & 1 ) << 31 );
}

static uint32_t SwapEdgeToBigEndian( uint32_t nValue )
{
	const uint32_t nStart = nValue & 0xFFFF;
	const uint32_t nOpposite = ( nValue >> 16 ) & 0x7FFF;
	const uint32_t nVirtual = nValue >> 31;
	return ByteSwap( ( nStart << 16 ) | ( nOpposite << 1 ) | nVirtual );
}

static uint32_t SwapEdgeFromBigEndian( uint32_t nValue )
{
	const uint32_t nPacked = ByteSwap( nValue );
	return ( nPacked >> 16 ) | ( ( ( nPacked >> 1 ) & 0x7FFF ) << 16 ) | ( ( nPacked & 1 ) << 31 );
}

static uint32_t SwapLedgeFlagsToBigEndian( uint32_t nValue )
{
	const uint32_t nHasChildren = nValue & 3;
	const uint32_t nIsCompact = ( nValue >> 2 ) & 3;
	const uint32_t nDummy = ( nValue >> 4 ) & 0xF;
	const uint32_t nSize = nValue >> 8;
	return ByteSwap( ( nHasChildren << 30 ) | ( nIsCompact << 28 ) | ( nDummy << 24 ) | nSize );
}

static uint32_t SwapLedgeFlagsFromBigEndian( uint32_t nValue )
{
	const uint32_t nPacked = ByteSwap( nValue );
	return ( nPacked >> 30 ) | ( ( ( nPacked >> 28 ) & 3 ) << 2 ) | ( ( ( nPacked >> 24 ) & 0xF ) << 4 ) | ( ( nPacked & 0xFFFFFF ) << 8 );
}

static void SwapWordInPlace( unsigned char *pBytes )
{
	Store( pBytes, ByteSwap( Load<uint32_t>( pBytes ) ) );
}

static void SwapShortInPlace( unsigned char *pBytes )
{
	Store( pBytes, ByteSwap( Load<uint16_t>( pBytes ) ) );
}

class CCompactSurfaceSwapper
{
public:
	CCompactSurfaceSwapper( std::vector<unsigned char> &surface, bool bToBigEndian )
	: m_Surface( surface )
	, m_bToBigEndian( bToBigEndian )
	{
	}

	bool Swap()
	{
		if ( m_Surface.size() < COMPACT_SURFACE_HEADER_SIZE )
			return false;

		const uint32_t nRoot = ReadNative( SURFACE_LEDGETREE_ROOT_OFFSET );
		if ( !SwapNode( nRoot, 0 ) )
			return false;

		for ( size_t nWord = 0; nWord < 12; ++nWord )
		{
			unsigned char *pWord = m_Surface.data() + nWord * 4;
			if ( nWord != 7 )
			{
				SwapWordInPlace( pWord );
				continue;
			}

			const uint32_t nValue = Load<uint32_t>( pWord );
			if ( m_bToBigEndian )
				Store( pWord, ByteSwap( ( nValue >> 8 ) | ( ( nValue & 0xFF ) << 24 ) ) );
			else
			{
				const uint32_t nPacked = ByteSwap( nValue );
				Store( pWord, ( ( nPacked & 0xFFFFFF ) << 8 ) | ( nPacked >> 24 ) );
			}
		}

		return true;
	}

private:
	uint32_t ReadNative( size_t nOffset ) const
	{
		const uint32_t nValue = Load<uint32_t>( m_Surface.data() + nOffset );
		return m_bToBigEndian ? nValue : ByteSwap( nValue );
	}

	bool SwapNode( size_t nNode, int nDepth )
	{
		if ( nDepth > MAX_LEDGETREE_DEPTH || !ContainsRange( m_Surface, nNode, COMPACT_LEDGETREE_NODE_SIZE ) )
			return false;

		const int32_t nRightOffset = static_cast<int32_t>( ReadNative( nNode ) );
		const int32_t nLedgeOffset = static_cast<int32_t>( ReadNative( nNode + 4 ) );

		if ( nRightOffset != 0 )
		{
			if ( !SwapNode( nNode + COMPACT_LEDGETREE_NODE_SIZE, nDepth + 1 ) )
				return false;

			const ptrdiff_t nRight = static_cast<ptrdiff_t>( nNode ) + nRightOffset;
			if ( nRight < 0 || !SwapNode( static_cast<size_t>( nRight ), nDepth + 1 ) )
				return false;
		}

		if ( nLedgeOffset != 0 )
		{
			const ptrdiff_t nLedge = static_cast<ptrdiff_t>( nNode ) + nLedgeOffset;
			if ( nLedge < 0 || !SwapLedge( static_cast<size_t>( nLedge ) ) )
				return false;
		}

		for ( size_t nWord = 0; nWord < 6; ++nWord )
			SwapWordInPlace( m_Surface.data() + nNode + nWord * 4 );

		return true;
	}

	bool SwapLedge( size_t nLedge )
	{
		if ( !m_SwappedLedges.insert( nLedge ).second )
			return true;

		if ( !ContainsRange( m_Surface, nLedge, COMPACT_LEDGE_HEADER_SIZE ) )
			return false;

		unsigned char *pLedge = m_Surface.data() + nLedge;
		const int32_t nPointOffset = static_cast<int32_t>( ReadNative( nLedge ) );
		const uint16_t nRawCount = Load<uint16_t>( pLedge + 12 );
		const int nTriangleCount = static_cast<int16_t>( m_bToBigEndian ? nRawCount : ByteSwap( nRawCount ) );
		if ( nTriangleCount < 0 || !ContainsRange( m_Surface, nLedge + COMPACT_LEDGE_HEADER_SIZE, nTriangleCount * COMPACT_TRIANGLE_SIZE ) )
			return false;

		for ( int i = 0; i < nTriangleCount; ++i )
		{
			unsigned char *pTriangle = pLedge + COMPACT_LEDGE_HEADER_SIZE + i * COMPACT_TRIANGLE_SIZE;
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
			{
				const uint32_t nEdgeValue = Load<uint32_t>( pTriangle + 4 + nEdge * 4 );
				const uint32_t nStart = m_bToBigEndian ? ( nEdgeValue & 0xFFFF ) : ( ByteSwap( nEdgeValue ) >> 16 );
				const ptrdiff_t nPoint = static_cast<ptrdiff_t>( nLedge ) + nPointOffset + static_cast<ptrdiff_t>( nStart * COMPACT_POINT_SIZE );
				if ( nPoint < 0 || !ContainsRange( m_Surface, static_cast<size_t>( nPoint ), COMPACT_POINT_SIZE ) )
					return false;

				if ( m_SwappedPoints.insert( static_cast<size_t>( nPoint ) ).second )
				{
					for ( size_t nComponent = 0; nComponent < 3; ++nComponent )
						SwapWordInPlace( m_Surface.data() + nPoint + nComponent * 4 );
				}
			}

			const uint32_t nHeader = Load<uint32_t>( pTriangle );
			Store( pTriangle, m_bToBigEndian ? SwapTriangleHeaderToBigEndian( nHeader ) : SwapTriangleHeaderFromBigEndian( nHeader ) );
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
			{
				unsigned char *pEdge = pTriangle + 4 + nEdge * 4;
				const uint32_t nEdgeValue = Load<uint32_t>( pEdge );
				Store( pEdge, m_bToBigEndian ? SwapEdgeToBigEndian( nEdgeValue ) : SwapEdgeFromBigEndian( nEdgeValue ) );
			}
		}

		SwapWordInPlace( pLedge );
		SwapWordInPlace( pLedge + 4 );
		const uint32_t nFlags = Load<uint32_t>( pLedge + 8 );
		Store( pLedge + 8, m_bToBigEndian ? SwapLedgeFlagsToBigEndian( nFlags ) : SwapLedgeFlagsFromBigEndian( nFlags ) );
		SwapShortInPlace( pLedge + 12 );
		SwapShortInPlace( pLedge + 14 );
		return true;
	}

	std::vector<unsigned char> &m_Surface;
	bool m_bToBigEndian;
	std::set<size_t> m_SwappedLedges;
	std::set<size_t> m_SwappedPoints;
};

void SwapCompactSurfaceToBigEndian( std::vector<unsigned char> &surface )
{
	CCompactSurfaceSwapper( surface, true ).Swap();
}

bool SwapCompactSurfaceFromBigEndian( std::vector<unsigned char> &surface )
{
	return CCompactSurfaceSwapper( surface, false ).Swap();
}

static DoubleVec3_t ComputeLedgeTriangleNormal( const DoubleVec3_t *pPoints, const std::array<int, 3> &triangle )
{
	const DoubleVec3_t a = pPoints[triangle[0]];
	const DoubleVec3_t normal = Cross( pPoints[triangle[2]] - a, pPoints[triangle[1]] - a );
	return ( 1 / Length( normal ) ) * normal;
}

static void FindPierceTriangles( std::span<const DoubleVec3_t> normals, std::span<int> pierces )
{
	std::ranges::fill( pierces, -1 );
	for ( int i = 0; i < static_cast<int>( normals.size() ); ++i )
	{
		if ( pierces[i] >= 0 )
			continue;

		int nBest = 0;
		double flBest = Dot( normals[i], normals[0] );
		for ( int j = 1; j < static_cast<int>( normals.size() ); ++j )
		{
			const double flDot = Dot( normals[i], normals[j] );
			if ( flDot < flBest )
			{
				flBest = flDot;
				nBest = j;
			}
		}

		if ( flBest < PIERCE_DOT_LIMIT )
		{
			pierces[i] = nBest;
			pierces[nBest] = i;
		}
	}
}

static void WriteLedgeHeader( unsigned char *pLedge, int nTriangleCount, size_t nPointCount )
{
	const size_t nSize = COMPACT_LEDGE_HEADER_SIZE + COMPACT_TRIANGLE_SIZE * nTriangleCount + COMPACT_POINT_SIZE * nPointCount;
	Store<int32_t>( pLedge, static_cast<int32_t>( COMPACT_LEDGE_HEADER_SIZE + COMPACT_TRIANGLE_SIZE * nTriangleCount ) );
	Store<uint32_t>( pLedge + 8, static_cast<uint32_t>( ( nSize / 16 ) << 8 ) | LEDGE_FLAG_COMPACT );
	Store<int16_t>( pLedge + 12, static_cast<int16_t>( nTriangleCount ) );
}

static void WriteLedgeTriangle( unsigned char *pLedge, int nTriangle, int nPierce, const std::array<int, 3> &startPoints, const std::array<int, 3> &opposites )
{
	unsigned char *pTriangle = pLedge + COMPACT_LEDGE_HEADER_SIZE + nTriangle * COMPACT_TRIANGLE_SIZE;
	Store<uint32_t>( pTriangle, static_cast<uint32_t>( nTriangle & 0xFFF ) | ( static_cast<uint32_t>( ( nPierce >= 0 ? nPierce : 0 ) & 0xFFF ) << 12 ) );
	for ( int nEdge = 0; nEdge < 3; ++nEdge )
		Store<uint32_t>( pTriangle + 4 + nEdge * 4, static_cast<uint32_t>( startPoints[nEdge] ) | ( ( static_cast<uint32_t>( opposites[nEdge] ) & 0x7FFF ) << 16 ) );
}

static constexpr int GetOppositeEdgeOffset( int nTriangle, int nEdge, int nOtherTriangle, int nOtherEdge )
{
	return ( 4 * nOtherTriangle + nOtherEdge + 1 ) - ( 4 * nTriangle + nEdge + 1 );
}

static constexpr int FindOppositeEdge( std::span<const std::array<int, 3>> triangles, int nTriangle, int nEdge )
{
	const int nStart = triangles[nTriangle][nEdge];
	const int nEnd = triangles[nTriangle][( nEdge + 1 ) % 3];
	for ( int j = 0; j < static_cast<int>( triangles.size() ); ++j )
	{
		if ( j == nTriangle )
			continue;

		for ( int nOther = 0; nOther < 3; ++nOther )
		{
			if ( triangles[j][nOther] == nEnd && triangles[j][( nOther + 1 ) % 3] == nStart )
				return GetOppositeEdgeOffset( nTriangle, nEdge, j, nOther );
		}
	}
	return 0;
}

std::vector<unsigned char> WriteCompactLedge( const std::vector<DoubleVec3_t> &points, const std::vector<std::array<int, 3>> &triangles )
{
	const int nTriangleCount = static_cast<int>( triangles.size() );

	std::vector<int> numbering( points.size(), -1 );
	std::vector<int> pointOrder;
	for ( const std::array<int, 3> &triangle : triangles )
	{
		for ( int nPoint : triangle )
		{
			if ( numbering[nPoint] < 0 )
			{
				numbering[nPoint] = static_cast<int>( pointOrder.size() );
				pointOrder.push_back( nPoint );
			}
		}
	}

	std::vector<DoubleVec3_t> normals( nTriangleCount );
	for ( int i = 0; i < nTriangleCount; ++i )
		normals[i] = ComputeLedgeTriangleNormal( points.data(), triangles[i] );

	std::vector<int> pierces( nTriangleCount );
	FindPierceTriangles( normals, pierces );

	const int nHalfEdgeCount = nTriangleCount * 3;
	std::vector<int> edgeOffsets( points.size() + 1, 0 );
	for ( const std::array<int, 3> &triangle : triangles )
	{
		for ( int nPoint : triangle )
			++edgeOffsets[nPoint];
	}
	for ( size_t i = 1; i < points.size(); ++i )
		edgeOffsets[i] += edgeOffsets[i - 1];
	edgeOffsets[points.size()] = nHalfEdgeCount;

	std::vector<int> halfEdges( nHalfEdgeCount );
	for ( int nHalfEdge = nHalfEdgeCount; nHalfEdge-- > 0; )
		halfEdges[--edgeOffsets[triangles[nHalfEdge / 3][nHalfEdge % 3]]] = nHalfEdge;

	std::vector<unsigned char> ledge( COMPACT_LEDGE_HEADER_SIZE + COMPACT_TRIANGLE_SIZE * nTriangleCount + COMPACT_POINT_SIZE * pointOrder.size(), 0 );
	unsigned char *pLedge = ledge.data();
	WriteLedgeHeader( pLedge, nTriangleCount, pointOrder.size() );

	for ( int i = 0; i < nTriangleCount; ++i )
	{
		std::array<int, 3> startPoints;
		std::array<int, 3> opposites;
		for ( int nEdge = 0; nEdge < 3; ++nEdge )
		{
			const int nStart = triangles[i][nEdge];
			const int nEnd = triangles[i][( nEdge + 1 ) % 3];
			startPoints[nEdge] = numbering[nStart];
			opposites[nEdge] = 0;

			for ( int k = edgeOffsets[nEnd]; k < edgeOffsets[nEnd + 1]; ++k )
			{
				const int j = halfEdges[k] / 3;
				const int nOther = halfEdges[k] % 3;
				if ( j != i && triangles[j][( nOther + 1 ) % 3] == nStart )
				{
					opposites[nEdge] = GetOppositeEdgeOffset( i, nEdge, j, nOther );
					break;
				}
			}
		}
		WriteLedgeTriangle( pLedge, i, pierces[i], startPoints, opposites );
	}

	unsigned char *pPoints = pLedge + COMPACT_LEDGE_HEADER_SIZE + COMPACT_TRIANGLE_SIZE * nTriangleCount;
	for ( size_t i = 0; i < pointOrder.size(); ++i )
		StorePoint( pPoints + i * COMPACT_POINT_SIZE, ToFloat( points[pointOrder[i]] ) );

	return ledge;
}

static constexpr std::array<std::array<int, 3>, 2> TRIANGLE_LEDGE_FACES = { { { 0, 1, 2 }, { 1, 0, 2 } } };
static constexpr std::array<int, 2> TRIANGLE_LEDGE_PIERCES = { 1, 0 };

static constexpr std::array<std::array<int, 3>, 2> TRIANGLE_LEDGE_OPPOSITES = [] {
	std::array<std::array<int, 3>, 2> opposites = {};
	for ( int i = 0; i < 2; ++i )
	{
		for ( int nEdge = 0; nEdge < 3; ++nEdge )
			opposites[i][nEdge] = FindOppositeEdge( TRIANGLE_LEDGE_FACES, i, nEdge );
	}
	return opposites;
}();

std::vector<unsigned char> WriteTriangleLedge( DoubleVec3_t a, DoubleVec3_t b, DoubleVec3_t c )
{
	if ( !( LengthSquared( Cross( c - a, b - a ) ) >= TRIANGLE_AREA_EPSILON ) )
		return {};

	std::vector<unsigned char> ledge( COMPACT_LEDGE_HEADER_SIZE + COMPACT_TRIANGLE_SIZE * 2 + COMPACT_POINT_SIZE * 3, 0 );
	unsigned char *pLedge = ledge.data();
	WriteLedgeHeader( pLedge, 2, 3 );
	for ( int i = 0; i < 2; ++i )
		WriteLedgeTriangle( pLedge, i, TRIANGLE_LEDGE_PIERCES[i], TRIANGLE_LEDGE_FACES[i], TRIANGLE_LEDGE_OPPOSITES[i] );

	unsigned char *pPoints = pLedge + COMPACT_LEDGE_HEADER_SIZE + COMPACT_TRIANGLE_SIZE * 2;
	const DoubleVec3_t points[3] = { a, b, c };
	for ( int i = 0; i < 3; ++i )
		StorePoint( pPoints + i * COMPACT_POINT_SIZE, ToFloat( points[i] ) );

	return ledge;
}
