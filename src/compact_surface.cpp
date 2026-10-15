#include "compact_surface.h"

#include "physics_collide.h"

#include <bit>
#include <cstdint>
#include <cstring>
#include <map>

static constexpr float IVP_TO_SOURCE = 39.37008f;
static constexpr size_t COMPACT_SURFACE_HEADER_SIZE = 48;
static constexpr size_t COMPACT_LEDGETREE_NODE_SIZE = 28;
static constexpr size_t COMPACT_LEDGE_HEADER_SIZE = 16;
static constexpr size_t COMPACT_TRIANGLE_SIZE = 16;
static constexpr size_t COMPACT_POINT_SIZE = 16;
static constexpr int MAX_LEDGETREE_DEPTH = 64;

static uint32_t ByteSwap( uint32_t nValue )
{
	return ( nValue >> 24 ) | ( ( nValue >> 8 ) & 0xFF00 ) | ( ( nValue << 8 ) & 0xFF0000 ) | ( nValue << 24 );
}

static uint16_t ByteSwap( uint16_t nValue )
{
	return static_cast<uint16_t>( ( nValue >> 8 ) | ( nValue << 8 ) );
}

class CCompactSurfaceReader
{
public:
	CCompactSurfaceReader( const void *pSurface, size_t nSize, bool bSwap )
		: m_pBytes( static_cast<const unsigned char *>( pSurface ) )
		, m_nSize( nSize )
		, m_bSwap( bSwap )
	{
	}

	bool Contains( size_t nOffset, size_t nLength ) const
	{
		return nOffset <= m_nSize && nLength <= m_nSize - nOffset;
	}

	uint32_t ReadUInt( size_t nOffset ) const
	{
		uint32_t nValue;
		std::memcpy( &nValue, m_pBytes + nOffset, sizeof( nValue ) );
		return m_bSwap ? ByteSwap( nValue ) : nValue;
	}

	int32_t ReadInt( size_t nOffset ) const
	{
		return static_cast<int32_t>( ReadUInt( nOffset ) );
	}

	int16_t ReadShort( size_t nOffset ) const
	{
		uint16_t nValue;
		std::memcpy( &nValue, m_pBytes + nOffset, sizeof( nValue ) );
		return static_cast<int16_t>( m_bSwap ? ByteSwap( nValue ) : nValue );
	}

	float ReadFloat( size_t nOffset ) const
	{
		return std::bit_cast<float>( ReadUInt( nOffset ) );
	}

	Vector ReadPosition( size_t nOffset ) const
	{
		const float x = ReadFloat( nOffset );
		const float y = ReadFloat( nOffset + 4 );
		const float z = ReadFloat( nOffset + 8 );
		return Vector( IVP_TO_SOURCE * x, IVP_TO_SOURCE * z, -( IVP_TO_SOURCE * y ) );
	}

private:
	const unsigned char *m_pBytes;
	size_t m_nSize;
	bool m_bSwap;
};

static std::unique_ptr<CPhysConvex> ReadCompactLedge( const CCompactSurfaceReader &reader, size_t nLedge )
{
	if ( !reader.Contains( nLedge, COMPACT_LEDGE_HEADER_SIZE ) )
		return nullptr;

	const int32_t nPointOffset = reader.ReadInt( nLedge );
	const uint32_t nClientData = reader.ReadUInt( nLedge + 4 );
	const int nTriangleCount = reader.ReadShort( nLedge + 12 );
	if ( nTriangleCount <= 0 || !reader.Contains( nLedge + COMPACT_LEDGE_HEADER_SIZE, nTriangleCount * COMPACT_TRIANGLE_SIZE ) )
		return nullptr;

	const size_t nPoints = nLedge + nPointOffset;
	std::map<unsigned int, unsigned short> pointRemap;
	std::vector<CPhysConvex::Triangle_t> triangles( nTriangleCount );

	for ( int i = 0; i < nTriangleCount; ++i )
	{
		const size_t nTriangle = nLedge + COMPACT_LEDGE_HEADER_SIZE + i * COMPACT_TRIANGLE_SIZE;
		CPhysConvex::Triangle_t &triangle = triangles[i];
		triangle.m_nMaterialIndex = static_cast<unsigned char>( ( reader.ReadUInt( nTriangle ) >> 24 ) & 0x7F );

		for ( int nEdge = 0; nEdge < 3; ++nEdge )
		{
			const unsigned int nPointIndex = reader.ReadUInt( nTriangle + 4 + nEdge * 4 ) & 0xFFFF;
			auto [it, bInserted] = pointRemap.try_emplace( nPointIndex, static_cast<unsigned short>( pointRemap.size() ) );
			triangle.m_nVertices[2 - nEdge] = it->second;
		}
	}

	std::vector<Vector> vertices( pointRemap.size() );
	for ( const auto &[nPointIndex, nVertex] : pointRemap )
	{
		const size_t nPoint = nPoints + nPointIndex * COMPACT_POINT_SIZE;
		if ( !reader.Contains( nPoint, COMPACT_POINT_SIZE ) )
			return nullptr;

		vertices[nVertex] = reader.ReadPosition( nPoint );
	}

	auto pConvex = std::make_unique<CPhysConvex>( std::move( vertices ), std::move( triangles ) );
	pConvex->SetGameData( nClientData );
	return pConvex;
}

static bool ReadLedgeTree( const CCompactSurfaceReader &reader, size_t nNode, int nDepth, std::vector<std::unique_ptr<CPhysConvex>> &convexes, bool &bHasOuterHull )
{
	if ( nDepth > MAX_LEDGETREE_DEPTH || !reader.Contains( nNode, COMPACT_LEDGETREE_NODE_SIZE ) )
		return false;

	const int32_t nRightOffset = reader.ReadInt( nNode );
	if ( nRightOffset == 0 )
	{
		std::unique_ptr<CPhysConvex> pConvex = ReadCompactLedge( reader, nNode + reader.ReadInt( nNode + 4 ) );
		if ( !pConvex )
			return false;

		convexes.push_back( std::move( pConvex ) );
		return true;
	}

	if ( reader.ReadInt( nNode + 4 ) != 0 )
		bHasOuterHull = true;

	return ReadLedgeTree( reader, nNode + COMPACT_LEDGETREE_NODE_SIZE, nDepth + 1, convexes, bHasOuterHull ) &&
		ReadLedgeTree( reader, nNode + nRightOffset, nDepth + 1, convexes, bHasOuterHull );
}

CPhysCollide *CreateCollideFromCompactSurface( const void *pSurface, size_t nSize, bool bSwap )
{
	const CCompactSurfaceReader reader( pSurface, nSize, bSwap );
	if ( !reader.Contains( 0, COMPACT_SURFACE_HEADER_SIZE ) )
		return nullptr;

	std::vector<std::unique_ptr<CPhysConvex>> convexes;
	bool bHasOuterHull = false;
	if ( !ReadLedgeTree( reader, reader.ReadInt( 32 ), 0, convexes, bHasOuterHull ) )
		return nullptr;

	auto *pCollide = new CPhysCollide( std::move( convexes ) );
	pCollide->SetHasOuterHull( bHasOuterHull );
	pCollide->SetMassCenter( reader.ReadPosition( 0 ) );
	pCollide->SetRotationInertia( Vector( reader.ReadFloat( 12 ), reader.ReadFloat( 20 ), reader.ReadFloat( 16 ) ) );
	return pCollide;
}
