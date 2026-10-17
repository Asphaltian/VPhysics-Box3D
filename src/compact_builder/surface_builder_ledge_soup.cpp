#include "surface_builder_ledge_soup.h"

#include "compact_surface.h"
#include "precision.h"
#include "surface_builder_pointsoup.h"
#include "vector_hash.h"

#include "tier0/commonmacros.h"
#include "tier0/valve_minmax_off.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <span>
#include <utility>

static constexpr real_t BOX_UNIT = static_cast<real_t>( 0.004 );
static constexpr real_t LEAF_BOX_SCALE = 1 / BOX_UNIT;
static constexpr float SPLIT_BAND = 1e-6f;
static constexpr float EMPTY_BOX_EXTENT = 1e6f;
static constexpr float VOLUME_DEGENERACY_FACTOR = 1e-9f;

#if defined( VPHYSICS_GMOD )
static constexpr float SPLIT_COST_LIMIT = std::numeric_limits<float>::infinity();
#else
static constexpr float SPLIT_COST_LIMIT = 1e20f;
#endif

static b3Vec3 GetTriangleVertex( const unsigned char *pLedge, const unsigned char *pTriangle, int nEdge )
{
	return LoadPoint( GetLedgePoint( pLedge, GetEdgeStartPoint( pTriangle, nEdge ) ) );
}

static float Component( b3Vec3 v, int nAxis )
{
	return nAxis == 0 ? v.x : ( nAxis == 1 ? v.y : v.z );
}

static RealVec3_t Midpoint( RealVec3_t a, RealVec3_t b )
{
	return real_t( 0.5 ) * a + real_t( 0.5 ) * b;
}

static RealVec3_t RealTriangleNormal( b3Vec3 v0, b3Vec3 v1, b3Vec3 v2 )
{
	const RealVec3_t origin = ToReal( v0 );
	return Cross( ToReal( v1 ) - origin, ToReal( v2 ) - origin );
}

static RealVec3_t NormalizeTriangleNormal( RealVec3_t normal )
{
	const real_t flLengthSquared = LengthSquared( normal );
#if defined( VPHYSICS_GMOD )
	if ( !( REAL_RESOLUTION <= flLengthSquared ) )
		return normal;

	const double flInverse = ApproximateInverseSquareRoot( flLengthSquared, 5 );
#else
	if ( flLengthSquared < REAL_RESOLUTION )
		return normal;

	const float flInverse = 1.0f / std::sqrt( flLengthSquared );
#endif
	return flInverse * normal;
}

static void GetExactBounds( const unsigned char *pLedge, b3Vec3 &mins, b3Vec3 &maxs, bool &bFirst )
{
	for ( int i = 0; i < GetLedgeTriangleCount( pLedge ); ++i )
	{
		const unsigned char *pTriangle = GetLedgeTriangle( pLedge, i );
		for ( int nEdge = 0; nEdge < 3; ++nEdge )
		{
			const b3Vec3 point = GetTriangleVertex( pLedge, pTriangle, nEdge );
			if ( bFirst )
			{
				mins = maxs = point;
				bFirst = false;
				continue;
			}

			mins = b3Min( point, mins );
			maxs = b3Max( point, maxs );
		}
	}
}

struct LedgeInfo_t
{
	RealVec3_t m_Center;
	real_t m_flRadius;
	RealVec3_t m_QuantizedMins;
	RealVec3_t m_QuantizedMaxs;
	unsigned char m_Box[3];
	int m_nLedge;
	int m_nLeft;
	int m_nRight;
};

static void MergeQuantizedBounds( const LedgeInfo_t &info, b3Vec3 &mins, b3Vec3 &maxs )
{
	const RealVec3_t &low = info.m_QuantizedMins;
	const RealVec3_t &high = info.m_QuantizedMaxs;
	mins.x = low.x < mins.x ? static_cast<float>( low.x ) : mins.x;
	mins.y = low.y < mins.y ? static_cast<float>( low.y ) : mins.y;
	mins.z = low.z < mins.z ? static_cast<float>( low.z ) : mins.z;
	maxs.x = maxs.x < high.x ? static_cast<float>( high.x ) : maxs.x;
	maxs.y = maxs.y < high.y ? static_cast<float>( high.y ) : maxs.y;
	maxs.z = maxs.z < high.z ? static_cast<float>( high.z ) : maxs.z;
}

static void InitializeNode( LedgeInfo_t &info, b3Vec3 mins, b3Vec3 maxs )
{
	const RealVec3_t realMaxs = ToReal( maxs );
	info.m_Center = Midpoint( realMaxs, ToReal( mins ) );
	info.m_flRadius = Length( realMaxs - info.m_Center );
}

static LedgeInfo_t ComputeLedgeInfo( const unsigned char *pLedge, int nLedge )
{
	b3Vec3 mins = b3Vec3_zero;
	b3Vec3 maxs = b3Vec3_zero;
	bool bFirst = true;
	GetExactBounds( pLedge, mins, maxs, bFirst );

	LedgeInfo_t info = {};
	InitializeNode( info, mins, maxs );
	info.m_nLedge = nLedge;
	info.m_nLeft = -1;
	info.m_nRight = -1;

	const real_t flScale = LEAF_BOX_SCALE / info.m_flRadius;
	for ( int nAxis = 0; nAxis < 3; ++nAxis )
		info.m_Box[nAxis] = static_cast<unsigned char>( static_cast<int>( ( Component( maxs, nAxis ) - Component( info.m_Center, nAxis ) ) * flScale ) + 1 );

	const real_t flUnit = BOX_UNIT * info.m_flRadius;
	const RealVec3_t extent = { info.m_Box[0] * flUnit, info.m_Box[1] * flUnit, info.m_Box[2] * flUnit };
	info.m_QuantizedMins = info.m_Center - extent;
	info.m_QuantizedMaxs = info.m_Center + extent;
	return info;
}

class CLedgeTreeBuilder
{
public:
	CLedgeTreeBuilder( std::vector<LedgeInfo_t> &infos, size_t nLeafCount )
	: m_Infos( infos )
	, m_Scratch( nLeafCount * 6 )
	{
	}

	int Build( std::span<int> list )
	{
		if ( list.size() == 1 )
			return list[0];

		b3Vec3 mins = { EMPTY_BOX_EXTENT, EMPTY_BOX_EXTENT, EMPTY_BOX_EXTENT };
		b3Vec3 maxs = { -EMPTY_BOX_EXTENT, -EMPTY_BOX_EXTENT, -EMPTY_BOX_EXTENT };
		for ( int nInfo : list )
			MergeQuantizedBounds( m_Infos[nInfo], mins, maxs );

		LedgeInfo_t node = {};
		InitializeNode( node, mins, maxs );
		node.m_nLedge = -1;
		const real_t flScale = real_t( -1 ) / ( BOX_UNIT * node.m_flRadius );
		for ( int nAxis = 0; nAxis < 3; ++nAxis )
			node.m_Box[nAxis] = static_cast<unsigned char>( 1 - static_cast<int>( ( Component( maxs, nAxis ) - Component( node.m_Center, nAxis ) ) * flScale ) );

		const int nNode = static_cast<int>( m_Infos.size() );
		m_Infos.push_back( node );

		if ( list.size() == 2 )
		{
			m_Infos[nNode].m_nLeft = list[0];
			m_Infos[nNode].m_nRight = list[1];
			return nNode;
		}

		const size_t nCount = list.size();
		Split_t splits[3];
		for ( int nAxis = 0; nAxis < 3; ++nAxis )
		{
			int *pLow = m_Scratch.data() + nAxis * 2 * nCount;
			splits[nAxis] = Partition( list, nAxis, pLow, pLow + nCount );
		}

		int nAxis = 0;
		float flBest = SPLIT_COST_LIMIT;
		if ( splits[0].m_flCost < flBest )
			flBest = splits[0].m_flCost;
		if ( splits[1].m_flCost < flBest )
		{
			flBest = splits[1].m_flCost;
			if ( splits[1].IsValid() )
				nAxis = 1;
		}
		if ( splits[2].m_flCost < flBest && splits[2].IsValid() )
			nAxis = 2;

		size_t nLowCount = nCount / 2;
		if ( splits[nAxis].IsValid() )
		{
			nLowCount = splits[nAxis].m_Low.size();
			std::ranges::copy( splits[nAxis].m_Low, list.begin() );
			std::ranges::copy( splits[nAxis].m_High, list.begin() + nLowCount );
		}

		const int nLeft = Build( list.first( nLowCount ) );
		const int nRight = Build( list.subspan( nLowCount ) );
		m_Infos[nNode].m_nLeft = nLeft;
		m_Infos[nNode].m_nRight = nRight;
		return nNode;
	}

private:
	struct Split_t
	{
		std::span<int> m_Low;
		std::span<int> m_High;
		float m_flCost;

		bool IsValid() const { return !m_Low.empty() && !m_High.empty(); }
	};

	Split_t Partition( std::span<const int> list, int nAxis, int *pLow, int *pHigh ) const
	{
		real_t flSum = 0;
		for ( int nInfo : list )
			flSum += Component( m_Infos[nInfo].m_Center, nAxis );

		const real_t flMean = flSum / static_cast<real_t>( list.size() );
		const real_t flLow = flMean - SPLIT_BAND;
		const real_t flHigh = flMean + SPLIT_BAND;

		size_t nLowCount = 0;
		size_t nHighCount = 0;
		bool bToggle = true;
		for ( size_t i = 0; i < list.size(); ++i )
		{
			const real_t flCenter = Component( m_Infos[list[i]].m_Center, nAxis );
			if ( flCenter < flLow )
			{
				pLow[nLowCount++] = list[i];
				continue;
			}
			if ( flCenter > flHigh )
			{
				pHigh[nHighCount++] = list[i];
				continue;
			}

			const int nNeighbor = i + 1 < list.size() ? list[i + 1] : list[i - 1];
			const real_t flNeighbor = Component( m_Infos[nNeighbor].m_Center, nAxis );
			if ( flNeighbor < flLow )
				pHigh[nHighCount++] = list[i];
			else if ( flNeighbor > flHigh )
				pLow[nLowCount++] = list[i];
			else if ( nLowCount == 0 )
				pLow[nLowCount++] = list[i];
			else if ( nHighCount == 0 )
				pHigh[nHighCount++] = list[i];
			else if ( bToggle )
			{
				pLow[nLowCount++] = list[i];
				bToggle = false;
			}
			else
			{
				pHigh[nHighCount++] = list[i];
				bToggle = true;
			}
		}

		const std::span<int> low = { pLow, nLowCount };
		const std::span<int> high = { pHigh, nHighCount };
		return { low, high, std::fabs( GetVolume( high ) ) + std::fabs( GetVolume( low ) ) };
	}

	float GetVolume( std::span<const int> list ) const
	{
		b3Vec3 mins = { EMPTY_BOX_EXTENT, EMPTY_BOX_EXTENT, EMPTY_BOX_EXTENT };
		b3Vec3 maxs = { -EMPTY_BOX_EXTENT, -EMPTY_BOX_EXTENT, -EMPTY_BOX_EXTENT };
		for ( int nInfo : list )
			MergeQuantizedBounds( m_Infos[nInfo], mins, maxs );

		return ( ( maxs.y - mins.y ) * ( maxs.x - mins.x ) ) * ( maxs.z - mins.z );
	}

	std::vector<LedgeInfo_t> &m_Infos;
	std::vector<int> m_Scratch;
};

struct MassProperties_t
{
	RealVec3_t m_Center;
	RealVec3_t m_RotationInertia;
};

static MassProperties_t ComputeFastMassProperties( const std::vector<const unsigned char *> &ledges )
{
	b3Vec3 mins = b3Vec3_zero;
	b3Vec3 maxs = b3Vec3_zero;
	bool bFirst = true;
	GetExactBounds( ledges[0], mins, maxs, bFirst );
	for ( size_t i = ledges.size() - 1; i >= 1; --i )
		GetExactBounds( ledges[i], mins, maxs, bFirst );

	const RealVec3_t realMins = ToReal( mins );
	const RealVec3_t realMaxs = ToReal( maxs );
	const real_t flInertia = LengthSquared( realMins - realMaxs ) * real_t( 0.125 );
	return { Midpoint( realMins, realMaxs ), { flInertia, flInertia, flInertia } };
}

static real_t ComputeInertiaRatio( const std::vector<const unsigned char *> &ledges, RealVec3_t center, int a, int b, int c )
{
	double integral = 0.0;
	double secondMoment = 0.0;

	for ( size_t nLedge = ledges.size(); nLedge-- > 0; )
	{
		const unsigned char *pLedge = ledges[nLedge];
		for ( int nTriangle = 0; nTriangle < GetLedgeTriangleCount( pLedge ); ++nTriangle )
		{
			const unsigned char *pTriangle = GetLedgeTriangle( pLedge, nTriangle );
			b3Vec3 rawVertices[3];
			b3Vec3 vertices[3];
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
			{
				rawVertices[nEdge] = GetTriangleVertex( pLedge, pTriangle, nEdge );
				vertices[nEdge] = ToFloat( ToReal( rawVertices[nEdge] ) - center );
			}

			const RealVec3_t normal = NormalizeTriangleNormal( RealTriangleNormal( rawVertices[0], rawVertices[1], rawVertices[2] ) );
			const real_t nb = Component( normal, b );
			const real_t nc = Component( normal, c );
			const bool bProjectToAB = std::fabs( nb ) <= std::fabs( nc );
			const double bQuadratic = bProjectToAB ? ( std::fabs( nc ) <= REAL_EPSILON ? 0.0 : static_cast<double>( ( nb * real_t( -0.5 ) ) / nc ) ) : 0.0;
			const double cQuadratic = bProjectToAB ? 0.0 : static_cast<double>( ( nc * real_t( 0.5 ) ) / nb );

			double triangleIntegral = 0.0;
			double triangleSecondMoment = 0.0;
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
			{
				const b3Vec3 p = vertices[nEdge];
				const b3Vec3 q = vertices[( nEdge + 1 ) % 3];
				const RealVec3_t d = ToReal( b3Sub( q, p ) );
				if ( !( Length( d ) * REAL_EPSILON <= std::fabs( Component( d, a ) ) ) )
					continue;

				const real_t flInverse = real_t( 1 ) / Component( d, a );
				const double x0 = Component( p, a );
				const double slopeB = static_cast<double>( flInverse * Component( d, b ) );
				const double interceptB = static_cast<double>( Component( p, b ) ) - x0 * slopeB;

				double constant, linear, quadratic;
				if ( bProjectToAB )
				{
					constant = interceptB * bQuadratic * interceptB;
					quadratic = slopeB * bQuadratic * slopeB;
					linear = ( bQuadratic + bQuadratic ) * interceptB * slopeB;
				}
				else
				{
					const double slopeC = static_cast<double>( flInverse * Component( d, c ) );
					const double interceptC = static_cast<double>( Component( p, c ) ) - x0 * slopeC;
					quadratic = ( slopeC * cQuadratic + slopeB ) * slopeC;
					constant = ( interceptC * cQuadratic + interceptB ) * interceptC;
					linear = ( ( cQuadratic + cQuadratic ) * interceptC + interceptB ) * slopeC + interceptC * slopeB;
				}

				const double x1 = Component( q, a );
				const double x1Cubed = x1 * x1 * x1;
				const double x0Cubed = x0 * x0 * x0;
				const double integralX = ( x1 * x1 - x0 * x0 ) * 0.5;
				const double x1Fourth = x1Cubed * x1;
				const double x0Fourth = x0Cubed * x0;
				const double integralX2 = ( x1Cubed - x0Cubed ) * ( 1.0 / 3.0 );
				const double integralX3 = ( x1Fourth - x0Fourth ) * 0.25;
				const double integralX4 = ( x1Fourth * x1 - x0Fourth * x0 ) * static_cast<double>( 0.2f );
				triangleIntegral += ( ( x1 - x0 ) * constant + integralX * linear ) + integralX2 * quadratic;
				triangleSecondMoment += integralX4 * quadratic + ( integralX3 * linear + integralX2 * constant );
			}

			secondMoment += triangleSecondMoment;
			integral += triangleIntegral;
		}
	}

	return integral >= REAL_RESOLUTION ? static_cast<real_t>( ( 1.0 / integral ) * secondMoment ) : real_t( 1 );
}

static MassProperties_t ComputeMassProperties( const std::vector<const unsigned char *> &ledges )
{
	b3Vec3 weightedCenter = b3Vec3_zero;
	real_t flVolume = 0;
	real_t flArea = 0;

	const auto accumulate = [&]( real_t flWeight, b3Vec3 point ) {
		weightedCenter.x = static_cast<float>( point.x * flWeight + weightedCenter.x );
		weightedCenter.y = static_cast<float>( point.y * flWeight + weightedCenter.y );
		weightedCenter.z = static_cast<float>( point.z * flWeight + weightedCenter.z );
	};

	for ( size_t nLedge = ledges.size(); nLedge-- > 0; )
	{
		const unsigned char *pLedge = ledges[nLedge];
		for ( int nTriangle = 0; nTriangle < GetLedgeTriangleCount( pLedge ); ++nTriangle )
		{
			const unsigned char *pTriangle = GetLedgeTriangle( pLedge, nTriangle );
			const b3Vec3 v0 = GetTriangleVertex( pLedge, pTriangle, 0 );
			const b3Vec3 v1 = GetTriangleVertex( pLedge, pTriangle, 1 );
			const b3Vec3 v2 = GetTriangleVertex( pLedge, pTriangle, 2 );
			const b3Vec3 n = ToFloat( RealTriangleNormal( v0, v1, v2 ) );
			const real_t s = b3Dot( n, v0 );
			const real_t q = s * real_t( 0.25 );
			flVolume += s;
			flArea += b3LengthSquared( n );
			accumulate( q, v0 );
			accumulate( q, v1 );
			accumulate( q, v2 );
		}
	}

	const real_t flScale = std::sqrt( flArea );
	if ( flVolume <= flScale * flScale * flScale * VOLUME_DEGENERACY_FACTOR )
		return ComputeFastMassProperties( ledges );

	const RealVec3_t center = ( real_t( 1 ) / flVolume ) * ToReal( weightedCenter );
	const real_t o0 = ComputeInertiaRatio( ledges, center, 0, 1, 2 );
	const real_t o1 = ComputeInertiaRatio( ledges, center, 1, 2, 0 );
	const real_t o2 = ComputeInertiaRatio( ledges, center, 2, 0, 1 );
	return { center, { std::sqrt( o2 * o2 + o1 * o1 ), std::sqrt( o2 * o2 + o0 * o0 ), std::sqrt( o1 * o1 + o0 * o0 ) } };
}

static void ComputeRadius( const std::vector<const unsigned char *> &ledges, RealVec3_t center, real_t &flRadius, real_t &flDeviation )
{
	real_t flRadiusSquared = 0;
	real_t flDeviationSquared = 0;

	for ( const unsigned char *pLedge : ledges )
	{
		for ( int nTriangle = 0; nTriangle < GetLedgeTriangleCount( pLedge ); ++nTriangle )
		{
			const unsigned char *pTriangle = GetLedgeTriangle( pLedge, nTriangle );
			b3Vec3 vertices[3];
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
				vertices[nEdge] = GetTriangleVertex( pLedge, pTriangle, nEdge );

			const RealVec3_t n = RealTriangleNormal( vertices[0], vertices[1], vertices[2] );
			const real_t flInverse = real_t( 1 ) / LengthSquared( n );
			for ( const b3Vec3 &vertex : vertices )
			{
				const RealVec3_t d = ToReal( vertex ) - center;
				flRadiusSquared = std::max( flRadiusSquared, LengthSquared( d ) );
				flDeviationSquared = std::max( flDeviationSquared, LengthSquared( Cross( d, n ) ) * flInverse );
			}
		}
	}

	flRadius = flRadiusSquared > 0 ? std::sqrt( flRadiusSquared ) : 0;
	flDeviation = flDeviationSquared > 0 ? std::sqrt( flDeviationSquared ) : 0;
}

using PointKey_t = std::array<uint32_t, 4>;

static std::vector<unsigned char> BuildOuterHullLedge( const std::vector<const unsigned char *> &sourceLedges )
{
	std::map<PointKey_t, int> pointIndices;
	std::vector<RealVec3_t> points;
	for ( size_t nLedge = sourceLedges.size(); nLedge-- > 0; )
	{
		const unsigned char *pLedge = sourceLedges[nLedge];
		for ( int nTriangle = 0; nTriangle < GetLedgeTriangleCount( pLedge ); ++nTriangle )
		{
			const unsigned char *pTriangle = GetLedgeTriangle( pLedge, nTriangle );
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
			{
				const unsigned char *pPoint = GetLedgePoint( pLedge, GetEdgeStartPoint( pTriangle, nEdge ) );
				if ( pointIndices.try_emplace( Load<PointKey_t>( pPoint ), static_cast<int>( points.size() ) ).second )
					points.push_back( ToReal( LoadPoint( pPoint ) ) );
			}
		}
	}

	return BuildPointSoupLedge( points );
}

static void MarkVirtualHullFeatures( unsigned char *pHull, const std::vector<const unsigned char *> &sourceLedges )
{
	std::map<PointKey_t, int> pointIndices;
	std::set<std::array<int, 3>> triangles;
	std::set<std::pair<int, int>> edges;

	auto getIndex = [&]( const unsigned char *pPoint, bool bInsert ) {
		const PointKey_t key = Load<PointKey_t>( pPoint );
		if ( bInsert )
			return pointIndices.try_emplace( key, static_cast<int>( pointIndices.size() ) ).first->second;

		const auto it = pointIndices.find( key );
		return it != pointIndices.end() ? it->second : -1;
	};

	for ( size_t nLedge = sourceLedges.size(); nLedge-- > 0; )
	{
		const unsigned char *pLedge = sourceLedges[nLedge];
		for ( int nTriangle = 0; nTriangle < GetLedgeTriangleCount( pLedge ); ++nTriangle )
		{
			const unsigned char *pTriangle = GetLedgeTriangle( pLedge, nTriangle );
			int indices[3];
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
				indices[nEdge] = getIndex( GetLedgePoint( pLedge, GetEdgeStartPoint( pTriangle, nEdge ) ), true );

			triangles.insert( { indices[0], indices[1], indices[2] } );
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
				edges.insert( { indices[nEdge], indices[( nEdge + 1 ) % 3] } );
		}
	}

	for ( int nTriangle = 0; nTriangle < GetLedgeTriangleCount( pHull ); ++nTriangle )
	{
		unsigned char *pTriangle = GetLedgeTriangle( pHull, nTriangle );
		int indices[3];
		for ( int nEdge = 0; nEdge < 3; ++nEdge )
			indices[nEdge] = getIndex( GetLedgePoint( pHull, GetEdgeStartPoint( pTriangle, nEdge ) ), false );

		if ( !triangles.contains( { indices[0], indices[1], indices[2] } ) )
			Store( pTriangle, Load<uint32_t>( pTriangle ) | 0x80000000u );

		for ( int nEdge = 0; nEdge < 3; ++nEdge )
		{
			if ( !edges.contains( { indices[nEdge], indices[( nEdge + 1 ) % 3] } ) )
			{
				unsigned char *pEdge = pTriangle + 4 + nEdge * 4;
				Store( pEdge, Load<uint32_t>( pEdge ) | 0x80000000u );
			}
		}
	}
}

static std::vector<const unsigned char *> GetLeafOrder( const std::vector<LedgeInfo_t> &infos, int nRoot, const std::vector<const std::vector<unsigned char> *> &inputLedges )
{
	std::vector<const unsigned char *> leaves;
	std::vector<int> stack = { nRoot };
	while ( !stack.empty() )
	{
		const int nInfo = stack.back();
		stack.pop_back();
		if ( infos[nInfo].m_nLedge >= 0 )
		{
			leaves.push_back( inputLedges[infos[nInfo].m_nLedge]->data() );
			continue;
		}
		stack.push_back( infos[nInfo].m_nRight );
		stack.push_back( infos[nInfo].m_nLeft );
	}
	return leaves;
}

struct PackedLedges_t
{
	std::vector<unsigned char> m_Bytes;
	std::vector<size_t> m_Offsets;
};

static PackedLedges_t PackLedges( const std::vector<const unsigned char *> &ledges )
{
	PackedLedges_t packed;
	if ( ledges.size() < 2 )
	{
		for ( const unsigned char *pLedge : ledges )
		{
			packed.m_Offsets.push_back( COMPACT_SURFACE_HEADER_SIZE + packed.m_Bytes.size() );
			packed.m_Bytes.insert( packed.m_Bytes.end(), pLedge, pLedge + GetLedgeSize( pLedge ) );
		}
		return packed;
	}

	size_t nTriangleTotal = 0;
	for ( const unsigned char *pLedge : ledges )
		nTriangleTotal += GetLedgeTriangleCount( pLedge );

	const size_t nPoolBase = COMPACT_SURFACE_HEADER_SIZE + COMPACT_TRIANGLE_SIZE * ( ledges.size() + nTriangleTotal );
	std::vector<unsigned char> pool;
	pool.reserve( nTriangleTotal * 3 * COMPACT_POINT_SIZE );
	using PoolTable_t = CVectorHash<3>;
	PoolTable_t poolIndices( nTriangleTotal * 3 );
	const auto findPoolIndex = [&]( const unsigned char *pPoint ) {
		const auto [nIndex, bInserted] = poolIndices.FindOrInsert( Load<PoolTable_t::Key_t>( pPoint ), static_cast<uint32_t>( pool.size() / COMPACT_POINT_SIZE ), [&]( uint32_t nPoolIndex ) {
			return Load<PoolTable_t::Key_t>( pool.data() + nPoolIndex * COMPACT_POINT_SIZE );
		} );
		if ( bInserted )
			pool.insert( pool.end(), pPoint, pPoint + COMPACT_POINT_SIZE );
		return nIndex;
	};

	for ( const unsigned char *pSource : ledges )
	{
		const size_t nOffset = COMPACT_SURFACE_HEADER_SIZE + packed.m_Bytes.size();
		packed.m_Offsets.push_back( nOffset );

		const int nTriangles = GetLedgeTriangleCount( pSource );
		const size_t nStart = packed.m_Bytes.size();
		packed.m_Bytes.insert( packed.m_Bytes.end(), pSource, pSource + COMPACT_LEDGE_HEADER_SIZE + COMPACT_TRIANGLE_SIZE * nTriangles );

		for ( int nTriangle = 0; nTriangle < nTriangles; ++nTriangle )
		{
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
			{
				const unsigned char *pPoint = GetLedgePoint( pSource, GetEdgeStartPoint( GetLedgeTriangle( pSource, nTriangle ), nEdge ) );
				SetEdgeStartPoint( GetLedgeTriangle( packed.m_Bytes.data() + nStart, nTriangle ), nEdge, findPoolIndex( pPoint ) );
			}
		}

		Store<int32_t>( packed.m_Bytes.data() + nStart, static_cast<int32_t>( nPoolBase - nOffset ) );
	}

	packed.m_Bytes.insert( packed.m_Bytes.end(), pool.begin(), pool.end() );
	return packed;
}

static void WriteLedgeTree( std::vector<unsigned char> &surface, size_t nNodeBase, const std::vector<LedgeInfo_t> &infos, int nRoot, const std::vector<size_t> &ledgeOffsets, bool bHasOuterHull )
{
	size_t nNextNode = nNodeBase;
	const auto writeNode = [&]( auto &&self, int nInfo ) -> void {
		const LedgeInfo_t &info = infos[nInfo];
		const size_t nNode = nNextNode;
		nNextNode += COMPACT_LEDGETREE_NODE_SIZE;

		unsigned char *pNode = surface.data() + nNode;
		Store( pNode + 8, ToFloat( info.m_Center ) );
		Store( pNode + 0x14, static_cast<float>( info.m_flRadius ) );
		pNode[0x18] = info.m_Box[0];
		pNode[0x19] = info.m_Box[1];
		pNode[0x1A] = info.m_Box[2];

		if ( info.m_nLedge >= 0 )
		{
			const size_t nLedge = ledgeOffsets[info.m_nLedge];
			Store<int32_t>( pNode + 4, static_cast<int32_t>( nLedge - nNode ) );
			unsigned char *pLedge = surface.data() + nLedge;
			Store<uint32_t>( pLedge + 8, GetLedgeFlags( pLedge ) & ~3u );
			return;
		}

		if ( nInfo == nRoot && bHasOuterHull )
		{
			const size_t nLedge = ledgeOffsets.back();
			unsigned char *pLedge = surface.data() + nLedge;
			Store<int32_t>( pNode + 4, static_cast<int32_t>( nLedge - nNode ) );
			Store<int32_t>( pLedge + 4, static_cast<int32_t>( nNode - nLedge ) );
			Store<uint32_t>( pLedge + 8, ( GetLedgeFlags( pLedge ) & ~2u ) | LEDGE_FLAG_HAS_CHILDREN );
		}

		self( self, info.m_nLeft );
		Store<int32_t>( surface.data() + nNode, static_cast<int32_t>( nNextNode - nNode ) );
		self( self, info.m_nRight );
	};
	writeNode( writeNode, nRoot );
}

static void WriteSurfaceHeader( std::vector<unsigned char> &surface, size_t nNodeBase, const std::vector<const unsigned char *> &ledges )
{
	const MassProperties_t mass = ComputeMassProperties( ledges );
	real_t flRadius = 0;
	real_t flDeviation = 0;
	ComputeRadius( ledges, mass.m_Center, flRadius, flDeviation );

	unsigned char *pHeader = surface.data();
	Store( pHeader + SURFACE_MASS_CENTER_OFFSET, ToFloat( mass.m_Center ) );
	Store( pHeader + SURFACE_ROTATION_INERTIA_OFFSET, ToFloat( mass.m_RotationInertia ) );
	Store( pHeader + SURFACE_UPPER_LIMIT_RADIUS_OFFSET, static_cast<float>( flRadius ) );

	const auto nDeviation = static_cast<uint8_t>( static_cast<int>( flDeviation / ( flRadius * BOX_UNIT ) + 1 ) );
	Store<uint32_t>( pHeader + SURFACE_BYTE_SIZE_OFFSET, static_cast<uint32_t>( surface.size() << 8 ) | nDeviation );
	Store<int32_t>( pHeader + SURFACE_LEDGETREE_ROOT_OFFSET, static_cast<int32_t>( nNodeBase ) );
	Store<uint32_t>( pHeader + SURFACE_ID_OFFSET, MAKEID( 'I', 'V', 'P', 'S' ) );
}

std::vector<unsigned char> BuildCompactSurface( const std::vector<const std::vector<unsigned char> *> &inputLedges, const CompactSurfaceParams_t &params )
{
	if ( inputLedges.empty() )
		return {};

	const int nInputCount = static_cast<int>( inputLedges.size() );
	std::vector<LedgeInfo_t> infos;
	infos.reserve( 2 * nInputCount );
	std::vector<int> list( nInputCount );
	for ( int i = 0; i < nInputCount; ++i )
	{
		infos.push_back( ComputeLedgeInfo( inputLedges[i]->data(), i ) );
		list[i] = i;
	}

	const int nRoot = CLedgeTreeBuilder( infos, list.size() ).Build( list );
	const std::vector<const unsigned char *> treeLedges = GetLeafOrder( infos, nRoot, inputLedges );

	std::vector<unsigned char> hull;
	if ( params.m_bBuildOuterHull && nInputCount > 1 )
	{
		hull = params.m_pForcedOuterHull ? *params.m_pForcedOuterHull : BuildOuterHullLedge( treeLedges );
		if ( hull.empty() )
			return {};

		if ( !params.m_pForcedOuterHull )
			MarkVirtualHullFeatures( hull.data(), treeLedges );
	}

	std::vector<const unsigned char *> writtenLedges;
	for ( const std::vector<unsigned char> *pLedge : inputLedges )
		writtenLedges.push_back( pLedge->data() );
	if ( !hull.empty() )
		writtenLedges.push_back( hull.data() );

	const PackedLedges_t packed = PackLedges( writtenLedges );
	const size_t nNodeBase = COMPACT_SURFACE_HEADER_SIZE + packed.m_Bytes.size();
	std::vector<unsigned char> surface( nNodeBase + COMPACT_LEDGETREE_NODE_SIZE * ( 2 * nInputCount - 1 ), 0 );
	std::memcpy( surface.data() + COMPACT_SURFACE_HEADER_SIZE, packed.m_Bytes.data(), packed.m_Bytes.size() );

	WriteLedgeTree( surface, nNodeBase, infos, nRoot, packed.m_Offsets, !hull.empty() );
	WriteSurfaceHeader( surface, nNodeBase, treeLedges );
	return surface;
}
