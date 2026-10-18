#pragma once

#include "math_util.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

inline constexpr size_t COMPACT_SURFACE_HEADER_SIZE = 48;
inline constexpr size_t COMPACT_LEDGETREE_NODE_SIZE = 28;
inline constexpr size_t COMPACT_LEDGE_HEADER_SIZE = 16;
inline constexpr size_t COMPACT_TRIANGLE_SIZE = 16;
inline constexpr size_t COMPACT_POINT_SIZE = 16;
inline constexpr int MAX_LEDGETREE_DEPTH = 64;

inline constexpr size_t SURFACE_MASS_CENTER_OFFSET = 0x00;
inline constexpr size_t SURFACE_ROTATION_INERTIA_OFFSET = 0x0C;
inline constexpr size_t SURFACE_UPPER_LIMIT_RADIUS_OFFSET = 0x18;
inline constexpr size_t SURFACE_BYTE_SIZE_OFFSET = 0x1C;
inline constexpr size_t SURFACE_LEDGETREE_ROOT_OFFSET = 0x20;
inline constexpr size_t SURFACE_INDEX_OFFSET = 0x24;
inline constexpr size_t SURFACE_ID_OFFSET = 0x2C;

inline constexpr uint32_t LEDGE_FLAG_HAS_CHILDREN = 1;
inline constexpr uint32_t LEDGE_FLAG_COMPACT = 4;

inline constexpr size_t FIRST_EDGE_OFFSET = COMPACT_LEDGE_HEADER_SIZE + 4;

template<typename T>
T Load( const unsigned char *pBytes )
{
	T value;
	std::memcpy( &value, pBytes, sizeof( value ) );
	return value;
}

template<typename T>
void Store( unsigned char *pBytes, T value )
{
	std::memcpy( pBytes, &value, sizeof( value ) );
}

inline uint32_t ByteSwap( uint32_t nValue )
{
	return ( nValue >> 24 ) | ( ( nValue >> 8 ) & 0xFF00 ) | ( ( nValue << 8 ) & 0xFF0000 ) | ( nValue << 24 );
}

inline uint16_t ByteSwap( uint16_t nValue )
{
	return static_cast<uint16_t>( ( nValue >> 8 ) | ( nValue << 8 ) );
}

inline b3Vec3 LoadPoint( const unsigned char *pPoint )
{
	return { Load<float>( pPoint ), Load<float>( pPoint + 4 ), Load<float>( pPoint + 8 ) };
}

inline void StorePoint( unsigned char *pPoint, b3Vec3 point )
{
	Store( pPoint, point.x );
	Store( pPoint + 4, point.y );
	Store( pPoint + 8, point.z );
	Store( pPoint + 12, 0.0f );
}

inline int32_t GetLedgePointOffset( const unsigned char *pLedge )
{
	return Load<int32_t>( pLedge );
}

inline uint32_t GetLedgeClientData( const unsigned char *pLedge )
{
	return Load<uint32_t>( pLedge + 4 );
}

inline uint32_t GetLedgeFlags( const unsigned char *pLedge )
{
	return Load<uint32_t>( pLedge + 8 );
}

inline int GetLedgeTriangleCount( const unsigned char *pLedge )
{
	return Load<int16_t>( pLedge + 12 );
}

inline size_t GetLedgeSize( const unsigned char *pLedge )
{
	return static_cast<size_t>( GetLedgeFlags( pLedge ) >> 8 ) * 16;
}

inline const unsigned char *GetLedgePoint( const unsigned char *pLedge, uint32_t nPoint )
{
	return pLedge + GetLedgePointOffset( pLedge ) + static_cast<ptrdiff_t>( nPoint * COMPACT_POINT_SIZE );
}

inline const unsigned char *GetLedgeTriangle( const unsigned char *pLedge, int nTriangle )
{
	return pLedge + COMPACT_LEDGE_HEADER_SIZE + nTriangle * COMPACT_TRIANGLE_SIZE;
}

inline unsigned char *GetLedgeTriangle( unsigned char *pLedge, int nTriangle )
{
	return pLedge + COMPACT_LEDGE_HEADER_SIZE + nTriangle * COMPACT_TRIANGLE_SIZE;
}

inline uint32_t GetTriangleMaterial( const unsigned char *pTriangle )
{
	return ( Load<uint32_t>( pTriangle ) >> 24 ) & 0x7F;
}

inline void SetTriangleMaterial( unsigned char *pTriangle, uint32_t nMaterial )
{
	const uint32_t nHeader = Load<uint32_t>( pTriangle );
	Store( pTriangle, nHeader ^ ( ( ( nMaterial << 24 ) ^ nHeader ) & 0x7F000000 ) );
}

inline uint32_t GetEdgeStartPoint( const unsigned char *pTriangle, int nEdge )
{
	return Load<uint32_t>( pTriangle + 4 + nEdge * 4 ) & 0xFFFF;
}

inline void SetEdgeStartPoint( unsigned char *pTriangle, int nEdge, uint32_t nPoint )
{
	unsigned char *pEdge = pTriangle + 4 + nEdge * 4;
	Store( pEdge, ( Load<uint32_t>( pEdge ) & 0xFFFF0000 ) | ( nPoint & 0xFFFF ) );
}

inline uint32_t GetEdgeStartPointAt( const unsigned char *pLedge, size_t nEdge )
{
	return Load<uint32_t>( pLedge + nEdge ) & 0xFFFF;
}

inline size_t GetPreviousEdge( size_t nEdge )
{
	static constexpr ptrdiff_t s_PreviousEdgeDelta[4] = { 0, 8, -4, -4 };
	return static_cast<size_t>( static_cast<ptrdiff_t>( nEdge ) + s_PreviousEdgeDelta[( nEdge >> 2 ) & 3] );
}

inline size_t GetOppositeEdge( const unsigned char *pLedge, size_t nEdge )
{
	const int32_t nOpposite = static_cast<int32_t>( Load<uint32_t>( pLedge + nEdge ) << 1 ) >> 17;
	return static_cast<size_t>( static_cast<ptrdiff_t>( nEdge ) + 4 * static_cast<ptrdiff_t>( nOpposite ) );
}

template<typename Fn>
void ForEachLedgePoint( const unsigned char *pLedge, Fn &&fn )
{
	const int nTriangleCount = GetLedgeTriangleCount( pLedge );
	uint32_t nMaximumPoint = 0;
	for ( int i = 0; i < nTriangleCount; ++i )
	{
		const unsigned char *pTriangle = GetLedgeTriangle( pLedge, i );
		for ( int nEdge = 0; nEdge < 3; ++nEdge )
			nMaximumPoint = std::max( nMaximumPoint, GetEdgeStartPoint( pTriangle, nEdge ) );
	}

	std::array<uint64_t, 1024> used;
	const uint32_t nWordCount = nMaximumPoint / 64 + 1;
	std::fill_n( used.data(), nWordCount, 0 );
	for ( int i = 0; i < nTriangleCount; ++i )
	{
		const unsigned char *pTriangle = GetLedgeTriangle( pLedge, i );
		for ( int nEdge = 0; nEdge < 3; ++nEdge )
		{
			const uint32_t nPoint = GetEdgeStartPoint( pTriangle, nEdge );
			used[nPoint / 64] |= uint64_t( 1 ) << ( nPoint % 64 );
		}
	}

	for ( uint32_t nWord = 0; nWord < nWordCount; ++nWord )
	{
		for ( uint64_t nBits = used[nWord]; nBits; nBits &= nBits - 1 )
			fn( nWord * 64 + static_cast<uint32_t>( std::countr_zero( nBits ) ) );
	}
}

inline bool ContainsRange( size_t nTotal, size_t nOffset, size_t nSize )
{
	return nOffset <= nTotal && nSize <= nTotal - nOffset;
}

inline bool ContainsRange( const std::vector<unsigned char> &bytes, size_t nOffset, size_t nSize )
{
	return ContainsRange( bytes.size(), nOffset, nSize );
}

std::vector<unsigned char> WriteCompactLedge( const std::vector<DoubleVec3_t> &points, const std::vector<std::array<int, 3>> &triangles );
std::vector<unsigned char> WriteTriangleLedge( DoubleVec3_t a, DoubleVec3_t b, DoubleVec3_t c );

void SwapCompactSurfaceToBigEndian( std::vector<unsigned char> &surface );
bool SwapCompactSurfaceFromBigEndian( std::vector<unsigned char> &surface );
