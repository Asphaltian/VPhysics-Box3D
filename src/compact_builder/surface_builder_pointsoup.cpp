#include "surface_builder_pointsoup.h"

#include "compact_surface.h"
#include "template_polygon.h"
#include "vector_hash.h"

#include <bit>
#include <cmath>
#include <cstdio>
#include <mutex>

extern "C" {
#include "qhull_a.h"

char qh_version[] = "";
}

static constexpr char PLAIN_COMMAND[] = "qhull Qs Pp C-0 W1e-14 E1.0e-6";
static constexpr char JOGGLE_COMMAND[] = "qhull Qs QJ%G C-0 Pp W1e-14 E1.0e-18";
static constexpr double INITIAL_JOGGLE = static_cast<double>( 1e-12f );
static constexpr double JOGGLE_GROWTH = static_cast<double>( 1.2f );
static constexpr double JOGGLE_INCREMENT = 1.2000000428887523e-12;
static constexpr double MAXIMUM_JOGGLE = static_cast<double>( 0.02f );
static constexpr float DEGENERATE_FACET_FACTOR = 0.005f;
static constexpr float PARALLEL_PLANE_COSINE = 0.9999f;

#if defined( VPHYSICS_GMOD )
static constexpr real_t INSIDE_TOLERANCE = -9.999999552965169e-05;
#else
static constexpr real_t INSIDE_TOLERANCE = -1e-4f;
#endif

#ifdef _WIN32
static constexpr char NULL_DEVICE[] = "NUL";
#else
static constexpr char NULL_DEVICE[] = "/dev/null";
#endif

static std::mutex s_QhullMutex;

static FILE *GetNullFile()
{
	static FILE *s_pFile = std::fopen( NULL_DEVICE, "w" );
	return s_pFile;
}

static bool RunQhull( const char *pCommand, std::vector<coordT> &coordinates )
{
	char command[64];
	std::snprintf( command, sizeof( command ), "%s", pCommand );
	return qh_new_qhull( 3, static_cast<int>( coordinates.size() / 3 ), coordinates.data(), False, command, nullptr, GetNullFile() ) == 0;
}

static void FreeQhull()
{
	qh_freeqhull( !qh_ALL );

	int nCurrentLong = 0;
	int nTotalLong = 0;
	qh_memfreeshort( &nCurrentLong, &nTotalLong );
}

struct FacetPolygon_t
{
	RealVec3_t m_Normal;
	std::vector<int> m_Vertices;
};

static int FindFarthestVertex( const std::vector<RealVec3_t> &points, const std::vector<int> &vertices, int nFrom )
{
	int nFarthest = -1;
	real_t flFarthest = -1;
	for ( size_t i = 0; i < vertices.size(); ++i )
	{
		const real_t flDistance = Length( points[vertices[i]] - points[vertices[nFrom]] );
		if ( flDistance > flFarthest )
		{
			flFarthest = flDistance;
			nFarthest = static_cast<int>( i );
		}
	}
	return nFarthest;
}

static std::vector<unsigned char> ConvertFacets( const std::vector<RealVec3_t> &points, std::vector<int> &usage, std::vector<unsigned char> &dropped )
{
	std::vector<FacetPolygon_t> polygons;
	bool bFailed = false;

	facetT *facet;
	FORALLfacets
	{
		FacetPolygon_t polygon;
		polygon.m_Normal = { static_cast<real_t>( facet->normal[0] ), static_cast<real_t>( facet->normal[1] ), static_cast<real_t>( facet->normal[2] ) };

		setT *vertices = qh_facet3vertex( facet );
		vertexT *vertex, **vertexp;
		FOREACHvertex_( vertices )
		{
			const int nPoint = qh_pointid( vertex->point );
			polygon.m_Vertices.push_back( nPoint );
			++usage[nPoint];
		}
		qh_settempfree( &vertices );

		const size_t nCount = polygon.m_Vertices.size();
		const RealVec3_t origin = points[polygon.m_Vertices[0]];
		real_t flWinding = 0;
		real_t flPerimeter = 0;
		for ( size_t i = 0; i < nCount; ++i )
		{
			const RealVec3_t current = points[polygon.m_Vertices[i]];
			const RealVec3_t next = points[polygon.m_Vertices[( i + 1 ) % nCount]];
			flWinding += Dot( Cross( next - current, current - origin ), polygon.m_Normal );
			flPerimeter += Length( current - next );
		}

		if ( flWinding < 0 )
		{
			flWinding = -flWinding;
			polygon.m_Normal = -polygon.m_Normal;
		}

		if ( flWinding < flPerimeter * DEGENERATE_FACET_FACTOR )
		{
			bool bAlreadyDropped = false;
			for ( int nPoint : polygon.m_Vertices )
				bAlreadyDropped |= dropped[nPoint] != 0;
			if ( bAlreadyDropped )
				continue;

			const int nFirst = FindFarthestVertex( points, polygon.m_Vertices, 0 );
			const int nSecond = FindFarthestVertex( points, polygon.m_Vertices, nFirst );
			for ( size_t i = 0; i < nCount; ++i )
			{
				if ( static_cast<int>( i ) != nFirst && static_cast<int>( i ) != nSecond )
				{
					dropped[polygon.m_Vertices[i]] = 1;
					break;
				}
			}

			bFailed = true;
			continue;
		}

		polygons.push_back( std::move( polygon ) );
	}

	if ( bFailed )
		return {};

	CTemplatePolygon templatePolygon( points );
	std::vector<TemplateLine_t> lines;
	for ( const FacetPolygon_t &polygon : polygons )
	{
		const size_t nCount = polygon.m_Vertices.size();

		lines.clear();
		for ( size_t j = 0; j < nCount; ++j )
		{
			const int nCurrent = polygon.m_Vertices[j];
			const int nNext = polygon.m_Vertices[( j + 1 ) % nCount];
			lines.push_back( { nNext, nCurrent } );
		}

		if ( !templatePolygon.AddSurface( polygon.m_Normal, lines ) )
			return {};
	}

	return WriteCompactLedge( points, templatePolygon.GetTriangles() );
}

std::vector<unsigned char> BuildPointSoupLedge( const std::vector<RealVec3_t> &input )
{
	if ( input.size() < 3 )
		return {};

	if ( input.size() == 3 )
		return WriteTriangleLedge( input[0], input[1], input[2] );

	using PointTable_t = CVectorHash<sizeof( RealVec3_t ) / sizeof( uint32_t )>;

	std::vector<RealVec3_t> points;
	points.reserve( input.size() );
	PointTable_t seen( input.size() );
	for ( RealVec3_t point : input )
	{
		const auto key = std::bit_cast<PointTable_t::Key_t>( point );
		if ( seen.FindOrInsert( key, static_cast<uint32_t>( points.size() ), [&]( uint32_t nIndex ) { return std::bit_cast<PointTable_t::Key_t>( points[nIndex] ); } ).second )
			points.push_back( point );
	}

	const std::lock_guard<std::mutex> lock( s_QhullMutex );

	bool bPlain = true;
	double flJoggle = INITIAL_JOGGLE;
	std::vector<coordT> coordinates;

	for ( ;; )
	{
		coordinates.clear();
		for ( RealVec3_t point : points )
			coordinates.insert( coordinates.end(), { static_cast<coordT>( point.x ), static_cast<coordT>( point.y ), static_cast<coordT>( point.z ) } );

		bool bBuilt = false;
		if ( bPlain )
		{
			bBuilt = RunQhull( PLAIN_COMMAND, coordinates );
			if ( !bBuilt )
				FreeQhull();
		}

		if ( !bBuilt )
		{
			char command[64];
			std::snprintf( command, sizeof( command ), JOGGLE_COMMAND, flJoggle );
			bBuilt = RunQhull( command, coordinates );
			if ( !bBuilt )
			{
				FreeQhull();
				bPlain = false;
				flJoggle = flJoggle * JOGGLE_GROWTH + JOGGLE_INCREMENT;
			}
		}

		if ( bBuilt )
		{
			std::vector<int> usage( points.size(), 0 );
			std::vector<unsigned char> dropped( points.size(), 0 );
			std::vector<unsigned char> ledge = ConvertFacets( points, usage, dropped );
			FreeQhull();
			if ( !ledge.empty() )
				return ledge;

			std::vector<RealVec3_t> kept;
			for ( size_t i = 0; i < points.size(); ++i )
			{
				if ( usage[i] > 0 && !dropped[i] )
					kept.push_back( points[i] );
			}

			if ( kept.size() == points.size() )
			{
				bPlain = false;
				flJoggle = flJoggle * JOGGLE_GROWTH + JOGGLE_INCREMENT;
			}

			points = std::move( kept );
			if ( points.size() == 3 )
				return WriteTriangleLedge( points[0], points[1], points[2] );
			if ( points.size() < 4 )
				return {};
		}

		if ( flJoggle >= MAXIMUM_JOGGLE )
			return {};
	}
}

void CPlaneSet::AddPlane( RealVec3_t normal, real_t flDistance )
{
	std::vector<size_t> replaced;
	for ( size_t i = 0; i < m_Planes.size(); ++i )
	{
		const Plane_t &plane = m_Planes[i];
		if ( Dot( plane.m_Normal, normal ) > PARALLEL_PLANE_COSINE )
		{
			if ( plane.m_flDistance < flDistance )
				return;
			replaced.push_back( i );
		}
	}

	m_Planes.push_back( { normal, flDistance } );
	for ( size_t i = replaced.size(); i-- > 0; )
		m_Planes.erase( m_Planes.begin() + replaced[i] );
}

std::vector<RealVec3_t> CPlaneSet::ComputeVertices( real_t flMergeDistance ) const
{
	const real_t flMergeSquared = flMergeDistance * flMergeDistance;
	auto isNear = [&]( RealVec3_t a, RealVec3_t b ) { return LengthSquared( a - b ) < flMergeSquared; };
	auto isInside = []( const Plane_t &plane, RealVec3_t point ) {
		return ( plane.m_Normal.x * point.x + plane.m_Normal.y * point.y ) + ( plane.m_Normal.z * point.z + plane.m_flDistance ) >= INSIDE_TOLERANCE;
	};

	std::vector<RealVec3_t> vertices;
	const size_t nCount = m_Planes.size();
	size_t nRejecting = 0;
	for ( size_t i = 0; i < nCount; ++i )
	{
		for ( size_t j = i + 1; j < nCount; ++j )
		{
			for ( size_t k = j + 1; k < nCount; ++k )
			{
				const RealVec3_t a = m_Planes[i].m_Normal;
				const RealVec3_t b = m_Planes[j].m_Normal;
				const RealVec3_t c = m_Planes[k].m_Normal;
				const RealVec3_t bc = Cross( b, c );
				const RealVec3_t ca = Cross( c, a );
				const RealVec3_t ab = Cross( a, b );

				const real_t flDeterminant = Dot( a, bc );
				if ( !( REAL_RESOLUTION <= std::fabs( flDeterminant ) ) )
					continue;

				const real_t r = 1 / flDeterminant;
				const real_t di = -m_Planes[i].m_flDistance;
				const real_t dj = -m_Planes[j].m_flDistance;
				const real_t dk = -m_Planes[k].m_flDistance;
				const RealVec3_t point = ( di * ( r * bc ) + dj * ( r * ca ) ) + dk * ( r * ab );

				if ( !isInside( m_Planes[nRejecting], point ) )
					continue;

				bool bInside = true;
				for ( size_t n = 0; n < nCount; ++n )
				{
					if ( !isInside( m_Planes[n], point ) )
					{
						nRejecting = n;
						bInside = false;
						break;
					}
				}

				if ( !bInside )
					continue;

				bool bMerged = false;
				for ( RealVec3_t vertex : vertices )
				{
					if ( isNear( point, vertex ) )
					{
						bMerged = true;
						break;
					}
				}

				if ( !bMerged )
					vertices.push_back( point );
			}
		}
	}

	for ( size_t i = 0; i < vertices.size(); ++i )
	{
		for ( size_t j = vertices.size(); j-- > i + 1; )
		{
			if ( isNear( vertices[i], vertices[j] ) )
				vertices.erase( vertices.begin() + j );
		}
	}

	return vertices;
}
