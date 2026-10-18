#include "template_polygon.h"

#include <algorithm>
#include <cmath>
#include <utility>

static constexpr int MAX_TRIANGULATION_ITERATIONS = 100;
static constexpr float PARALLEL_EPSILON = 1e-9f;
static constexpr float COLLINEAR_EPSILON = 9.999999e-09f;
static constexpr float OVERLAP_EPSILON = 1e-8f;
static constexpr float SEGMENT_END_MARGIN = 1e-4f;

static bool IsSameLine( const TemplateLine_t &edge, int p, int q )
{
	return ( edge.m_nStart == p && edge.m_nEnd == q ) || ( edge.m_nStart == q && edge.m_nEnd == p );
}

CTemplatePolygon::CTemplatePolygon( const std::vector<DoubleVec3_t> &points )
: m_Points( points )
, m_SurfacePoints( points.size() )
{
}

int CTemplatePolygon::GetProjectionAxis( DoubleVec3_t normal )
{
	if ( std::fabs( normal.z ) > std::max( std::fabs( normal.x ), std::fabs( normal.y ) ) )
		return 2;
	return std::fabs( normal.x ) < std::fabs( normal.y ) ? 1 : 0;
}

bool CTemplatePolygon::IsProjectionFlipped( DoubleVec3_t normal, int nAxis )
{
	return ( Component( normal, nAxis ) < 0 ) != ( nAxis == 1 );
}

bool CTemplatePolygon::AddSurface( DoubleVec3_t normal, const std::vector<TemplateLine_t> &lines )
{
	const int nAxis = GetProjectionAxis( normal );
	const bool bFlip = IsProjectionFlipped( normal, nAxis );
	std::vector<TemplateLine_t> edges = lines;
	if ( bFlip )
	{
		for ( TemplateLine_t &edge : edges )
			std::swap( edge.m_nStart, edge.m_nEnd );
	}

	for ( const TemplateLine_t &edge : edges )
	{
		for ( int nPoint : { edge.m_nStart, edge.m_nEnd } )
		{
			const DoubleVec3_t &point = m_Points[nPoint];
			m_SurfacePoints[nPoint] = nAxis == 2 ? SurfacePoint_t{ point.x, point.y } : ( nAxis == 0 ? SurfacePoint_t{ point.y, point.z } : SurfacePoint_t{ point.x, point.z } );
		}
	}

	m_SurfaceTriangles.clear();
	if ( !TriangulateSurface( edges, m_SurfaceTriangles ) )
		return false;

	for ( const std::array<int, 3> &triangle : m_SurfaceTriangles )
	{
		const int a = triangle[0];
		const int c = triangle[1];
		const int b = triangle[2];
		const DoubleVec3_t cross = Cross( m_Points[c] - m_Points[b], m_Points[a] - m_Points[b] );
		if ( Dot( cross, normal ) >= 0 )
			m_Triangles.insert( m_Triangles.begin(), { b, c, a } );
		else
			m_Triangles.insert( m_Triangles.begin(), { b, a, c } );
	}

	return true;
}

bool CTemplatePolygon::TriangulateSurface( std::vector<TemplateLine_t> edges, std::vector<std::array<int, 3>> &triangles ) const
{
	struct Candidate_t
	{
		double m_flKey;
		size_t m_nEdge;
	};

	std::vector<Candidate_t> candidates;
	for ( int nIteration = 0; !edges.empty(); ++nIteration )
	{
		if ( nIteration > MAX_TRIANGULATION_ITERATIONS )
			return false;

		const TemplateLine_t front = edges.front();
		const int a = front.m_nStart;
		const int b = front.m_nEnd;

		candidates.clear();
		for ( size_t i = 0; i < edges.size(); ++i )
			candidates.push_back( { GetKey( edges[i].m_nStart, a, b ), i } );

		std::ranges::stable_sort( candidates, []( const Candidate_t &lhs, const Candidate_t &rhs ) { return lhs.m_flKey < rhs.m_flKey; } );

		for ( const Candidate_t &candidate : candidates )
		{
			const int c = edges[candidate.m_nEdge].m_nStart;
			if ( candidate.m_nEdge == 0 || c == b || !IsEar( edges, front, c ) )
				continue;

			triangles.insert( triangles.begin(), { a, c, b } );
			edges.erase( edges.begin() );

			bool bFoundCA = false;
			bool bFoundCB = false;
			for ( size_t i = 0; i < edges.size(); )
			{
				if ( !bFoundCA && IsSameLine( edges[i], c, a ) )
				{
					bFoundCA = true;
					edges.erase( edges.begin() + i );
				}
				else if ( !bFoundCB && IsSameLine( edges[i], c, b ) )
				{
					bFoundCB = true;
					edges.erase( edges.begin() + i );
				}
				else
				{
					++i;
				}
			}

			if ( !bFoundCA )
				edges.insert( edges.begin(), { a, c } );
			if ( !bFoundCB )
				edges.insert( edges.begin(), { c, b } );
			break;
		}
	}

	return true;
}

double CTemplatePolygon::GetKey( int nPoint, int a, int b ) const
{
	const SurfacePoint_t &p = m_SurfacePoints[nPoint];
	const double dxb = p.x - m_SurfacePoints[b].x;
	const double dyb = p.y - m_SurfacePoints[b].y;
	const double dxa = p.x - m_SurfacePoints[a].x;
	const double dya = p.y - m_SurfacePoints[a].y;
	return ( dxb * dxb + dyb * dyb ) + ( dxa * dxa + dya * dya );
}

double CTemplatePolygon::Left( const TemplateLine_t &edge, int nPoint ) const
{
	const SurfacePoint_t &start = m_SurfacePoints[edge.m_nStart];
	const SurfacePoint_t &end = m_SurfacePoints[edge.m_nEnd];
	const SurfacePoint_t &p = m_SurfacePoints[nPoint];
	return ( p.y - start.y ) * ( end.x - start.x ) + ( start.x - p.x ) * ( end.y - start.y );
}

double CTemplatePolygon::GetLineDistance( const TemplateLine_t &edge, int nPoint ) const
{
	const SurfacePoint_t &start = m_SurfacePoints[edge.m_nStart];
	const SurfacePoint_t &end = m_SurfacePoints[edge.m_nEnd];
	const SurfacePoint_t &p = m_SurfacePoints[nPoint];
	const double dx = end.x - start.x;
	const double dy = end.y - start.y;
	const double flLengthSquared = dx * dx + dy * dy;
	if ( flLengthSquared < DOUBLE_RESOLUTION )
		return ( start.x - p.x ) * ( start.x - p.x ) + ( start.y - p.y ) * ( start.y - p.y );
	return ( ( start.x - p.x ) * dy + ( p.y - start.y ) * dx ) / std::sqrt( flLengthSquared );
}

bool CTemplatePolygon::IsInsideSegment( const TemplateLine_t &edge, int nPoint ) const
{
	const SurfacePoint_t &start = m_SurfacePoints[edge.m_nStart];
	const SurfacePoint_t &end = m_SurfacePoints[edge.m_nEnd];
	const SurfacePoint_t &p = m_SurfacePoints[nPoint];

	double flDelta = end.x - start.x;
	double flOffset = p.x - start.x;
	if ( std::fabs( flDelta ) <= DOUBLE_RESOLUTION )
	{
		flDelta = end.y - start.y;
		flOffset = p.y - start.y;
		if ( std::fabs( flDelta ) <= DOUBLE_RESOLUTION )
			return false;
	}

	const double flAlong = flOffset * static_cast<double>( 0 < flDelta ? 1 : -1 );
	return SEGMENT_END_MARGIN < flAlong && flAlong < std::fabs( flDelta ) - SEGMENT_END_MARGIN;
}

bool CTemplatePolygon::Overlaps( const TemplateLine_t &f, const TemplateLine_t &g ) const
{
	const SurfacePoint_t &fa = m_SurfacePoints[f.m_nStart];
	const SurfacePoint_t &fb = m_SurfacePoints[f.m_nEnd];
	const SurfacePoint_t &ga = m_SurfacePoints[g.m_nStart];
	const SurfacePoint_t &gb = m_SurfacePoints[g.m_nEnd];

	double flMin, flMax, flG0, flG1;
	if ( std::fabs( fb.x - fa.x ) > OVERLAP_EPSILON )
	{
		flMin = std::min( fa.x, fb.x );
		flMax = std::max( fa.x, fb.x );
		flG0 = ga.x;
		flG1 = gb.x;
	}
	else if ( std::fabs( fb.y - fa.y ) > OVERLAP_EPSILON )
	{
		flMin = std::min( fa.y, fb.y );
		flMax = std::max( fa.y, fb.y );
		flG0 = ga.y;
		flG1 = gb.y;
	}
	else
	{
		return IsInsideSegment( g, f.m_nStart );
	}

	const bool bBelow = flG0 <= flMin + OVERLAP_EPSILON && flG1 <= flMin + OVERLAP_EPSILON;
	const bool bAbove = flG0 >= flMax - OVERLAP_EPSILON && flG1 >= flMax - OVERLAP_EPSILON;
	return !bBelow && !bAbove;
}

bool CTemplatePolygon::Intersects( const TemplateLine_t &f, const TemplateLine_t &g ) const
{
	const SurfacePoint_t &fa = m_SurfacePoints[f.m_nStart];
	const SurfacePoint_t &fb = m_SurfacePoints[f.m_nEnd];
	const SurfacePoint_t &ga = m_SurfacePoints[g.m_nStart];
	const SurfacePoint_t &gb = m_SurfacePoints[g.m_nEnd];
	const double fdx = fb.x - fa.x;
	const double fdy = fb.y - fa.y;
	const double gdx = gb.x - ga.x;
	const double gdy = gb.y - ga.y;

	const double flDenominator = fdy * gdx - fdx * gdy;
	if ( std::fabs( flDenominator ) >= PARALLEL_EPSILON )
	{
		if ( f.m_nStart == g.m_nStart || f.m_nStart == g.m_nEnd || f.m_nEnd == g.m_nStart || f.m_nEnd == g.m_nEnd )
			return false;

		const double t1 = gdx * ( ga.y - fa.y ) - gdy * ( ga.x - fa.x );
		const double t2 = fdx * ( ga.y - fa.y ) - fdy * ( ga.x - fa.x );
		const double flLow = std::min( flDenominator, 0.0 );
		const double flHigh = std::max( flDenominator, 0.0 );
		return t1 >= flLow && t1 <= flHigh && t2 >= flLow && t2 <= flHigh;
	}

	return std::fabs( GetLineDistance( f, g.m_nStart ) ) < COLLINEAR_EPSILON && Overlaps( f, g );
}

bool CTemplatePolygon::IsEar( const std::vector<TemplateLine_t> &edges, const TemplateLine_t &front, int c ) const
{
	if ( !( Left( front, c ) > DOUBLE_RESOLUTION ) )
		return false;

	const TemplateLine_t closing = { c, front.m_nStart };
	const TemplateLine_t opening = { front.m_nEnd, c };
	for ( const TemplateLine_t &edge : edges )
	{
		if ( !IsSameLine( edge, closing.m_nStart, closing.m_nEnd ) && Intersects( edge, closing ) )
			return false;
		if ( !IsSameLine( edge, opening.m_nStart, opening.m_nEnd ) && Intersects( edge, opening ) )
			return false;
	}

	for ( const TemplateLine_t &edge : edges )
	{
		const int p = edge.m_nStart;
		if ( Left( front, p ) > DOUBLE_RESOLUTION && Left( opening, p ) > DOUBLE_RESOLUTION && Left( closing, p ) > DOUBLE_RESOLUTION )
			return false;
	}

	return true;
}
