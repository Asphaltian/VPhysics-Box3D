#include "physics_trace.h"

#include "compact_builder/compact_surface.h"
#include "compact_builder/math_util.h"
#include "conversion.h"
#include "physics_collide.h"

#include "bspflags.h"
#include "coordsize.h"
#include "vphysics_interface.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <numbers>
#include <span>
#include <utility>
#include <vector>

#include <emmintrin.h>

static constexpr float TRACE_BOX_EPSILON = static_cast<float>( DIST_EPSILON );
static constexpr float TRACE_COLLIDE_EPSILON = 0.25f;
static constexpr float MINIMUM_TRACE_LENGTH = 1e-8f;

static constexpr int MAX_SOLVER_ITERATIONS = 100;
static constexpr int MAX_GRAZE_ITERATIONS = 20;
static constexpr float SOLVER_LENGTH_EPSILON = 1e-10f;
static constexpr float SOLVER_FAR_DISTANCE = 1e24f;
static constexpr float EXIT_PROGRESS_TOLERANCE = 0.00390625f;
static constexpr float GRAZE_PROGRESS_TOLERANCE = -0.0001f;
static constexpr float GRAZE_MINIMUM_APPROACH = -0.0001f;
static constexpr double GRAZE_APPROACH_FACTOR = -0.1;
static constexpr double GRAZE_DISTANCE_FACTOR = 0.9;
static constexpr unsigned short SEGMENT_END_FLAG = 0x8000;
static constexpr unsigned short VERTEX_INDEX_MASK = 0x7FFF;

static constexpr int LANE_COUNT = 4;
static constexpr int MAX_POINT_BLOCKS = MAX_COLLIDE_MAP_SPAN / LANE_COUNT;
static constexpr int VISITED_SLOT_COUNT = 512;
static constexpr float SPHERE_BOUND_SCALE = 1.00001f;
static constexpr float SEPARATION_SLACK = 1e-5f;

static_assert( MAX_COLLIDE_MAP_LEDGES <= 32 );

struct SupportPoint_t
{
	b3Vec3 m_Point;
	int m_nIndex;
};

struct SupportBound_t
{
	float m_flValue;
	float m_flMagnitude;
};

struct SolverVertex_t
{
	b3Vec3 m_Point;
	unsigned short m_nMovingIndex;
	unsigned short m_nTargetIndex;
};

struct SolverSimplex_t
{
	SolverVertex_t m_Vertices[4];
	int m_nCount = 0;
};

struct TraceRay_t
{
	b3Vec3 m_Start;
	b3Vec3 m_End;
	b3Vec3 m_Delta;
	b3Vec3 m_Direction;
	float m_flCurrentLength;
	float m_flLength;
	float m_flInverseLength;
};

struct LedgeHit_t
{
	bool m_bHit = false;
	bool m_bStartSolid = false;
	float m_flDistance = 0.0f;
	b3Vec3 m_Normal = b3Vec3_zero;
};

static float SumAbsolute( b3Vec3 v )
{
	return ( std::fabs( v.x ) + std::fabs( v.y ) ) + std::fabs( v.z );
}

static __m128 Dot( __m128 ax, __m128 ay, __m128 az, __m128 bx, __m128 by, __m128 bz )
{
	return _mm_add_ps( _mm_add_ps( _mm_mul_ps( ax, bx ), _mm_mul_ps( ay, by ) ), _mm_mul_ps( az, bz ) );
}

static float ReciprocalLength( float flLengthSquared )
{
	const float r = _mm_cvtss_f32( _mm_rsqrt_ss( _mm_set_ss( flLengthSquared ) ) );
	return r * ( 3.0f - r * r * flLengthSquared ) * 0.5f;
}

static b3Vec3 Normalize( b3Vec3 v )
{
	return b3MulSV( ReciprocalLength( b3LengthSquared( v ) + SOLVER_LENGTH_EPSILON ), v );
}

static b3Vec3 SegmentSearchVector( b3Vec3 edge, b3Vec3 a )
{
	return b3Cross( b3Cross( edge, a ), edge );
}

static void UpdateLine( SolverSimplex_t &simplex, const SolverVertex_t &a, b3Vec3 &v )
{
	const b3Vec3 A = a.m_Point;
	const b3Vec3 ab = b3Sub( simplex.m_Vertices[0].m_Point, A );
	if ( b3Dot( ab, A ) >= 0.0f )
	{
		simplex.m_nCount = 1;
		simplex.m_Vertices[0] = a;
		v = A;
		return;
	}

	simplex.m_nCount = 2;
	simplex.m_Vertices[1] = a;
	v = SegmentSearchVector( ab, A );
}

static void UpdateTriangle( SolverSimplex_t &simplex, const SolverVertex_t &a, b3Vec3 &v )
{
	const b3Vec3 A = a.m_Point;
	const b3Vec3 ab = b3Sub( simplex.m_Vertices[1].m_Point, A );
	const b3Vec3 ac = b3Sub( simplex.m_Vertices[0].m_Point, A );
	const b3Vec3 n = b3Cross( ac, ab );

	if ( 0.0f <= b3Dot( b3Cross( n, ab ), A ) )
	{
		if ( 0.0f < b3Dot( b3Cross( ac, n ), A ) )
		{
			simplex.m_nCount = 3;
			if ( 0.0f <= b3Dot( n, A ) )
			{
				simplex.m_Vertices[2] = simplex.m_Vertices[1];
				simplex.m_Vertices[1] = a;
				v = n;
			}
			else
			{
				simplex.m_Vertices[2] = a;
				v = b3Neg( n );
			}
			return;
		}
	}
	else if ( b3Dot( ab, A ) < 0.0f )
	{
		simplex.m_nCount = 2;
		simplex.m_Vertices[0] = a;
		v = SegmentSearchVector( ab, A );
		return;
	}

	if ( b3Dot( ac, A ) >= 0.0f )
	{
		simplex.m_nCount = 1;
		simplex.m_Vertices[0] = a;
		v = A;
		return;
	}

	simplex.m_nCount = 2;
	simplex.m_Vertices[1] = a;
	v = SegmentSearchVector( ac, A );
}

static bool UpdateTetrahedron( SolverSimplex_t &simplex, const SolverVertex_t &a, b3Vec3 &v )
{
	const b3Vec3 A = a.m_Point;
	const b3Vec3 ab = b3Sub( simplex.m_Vertices[1].m_Point, A );
	const b3Vec3 ac = b3Sub( simplex.m_Vertices[0].m_Point, A );
	const b3Vec3 ad = b3Sub( simplex.m_Vertices[2].m_Point, A );

	const b3Vec3 n1 = b3Cross( ac, ab );
	const b3Vec3 n2 = b3Cross( ab, ad );
	const b3Vec3 n3 = b3Cross( ad, ac );

	const auto edge1 = [&] { return b3Dot( b3Cross( n1, ab ), A ); };
	const auto face1 = [&] { return b3Dot( b3Cross( ac, n1 ), A ); };
	const auto edge2 = [&] { return b3Dot( b3Cross( n2, ad ), A ); };
	const auto face2 = [&] { return b3Dot( b3Cross( ab, n2 ), A ); };
	const auto edge3 = [&] { return b3Dot( b3Cross( n3, ac ), A ); };
	const auto face3 = [&] { return b3Dot( b3Cross( ad, n3 ), A ); };

	const auto vertexA = [&] {
		simplex.m_nCount = 1;
		simplex.m_Vertices[0] = a;
		v = A;
	};
	const auto segmentAB = [&] {
		simplex.m_nCount = 2;
		simplex.m_Vertices[0] = a;
		v = SegmentSearchVector( ab, A );
	};
	const auto segmentCA = [&] {
		simplex.m_nCount = 2;
		simplex.m_Vertices[1] = a;
		v = SegmentSearchVector( ac, A );
	};
	const auto segmentDA = [&] {
		simplex.m_Vertices[0] = simplex.m_Vertices[2];
		simplex.m_nCount = 2;
		simplex.m_Vertices[1] = a;
		v = SegmentSearchVector( ad, A );
	};
	const auto triangleCBA = [&] {
		simplex.m_nCount = 3;
		simplex.m_Vertices[2] = a;
		v = b3Cross( ab, ac );
	};
	const auto triangleABD = [&] {
		simplex.m_nCount = 3;
		simplex.m_Vertices[0] = a;
		v = b3Cross( ad, ab );
	};
	const auto triangleCAD = [&] {
		simplex.m_nCount = 3;
		simplex.m_Vertices[1] = a;
		v = b3Cross( ac, ad );
	};

	const int nRegion = ( b3Dot( n1, A ) < 0.0f ? 1 : 0 ) | ( b3Dot( n2, A ) < 0.0f ? 2 : 0 ) | ( b3Dot( n3, A ) < 0.0f ? 4 : 0 );
	switch ( nRegion )
	{
	case 0:
		simplex.m_Vertices[3] = a;
		simplex.m_nCount = 4;
		return true;
	case 1:
		UpdateTriangle( simplex, a, v );
		break;
	case 2:
		simplex.m_Vertices[0] = simplex.m_Vertices[2];
		UpdateTriangle( simplex, a, v );
		break;
	case 4:
		simplex.m_Vertices[1] = simplex.m_Vertices[2];
		UpdateTriangle( simplex, a, v );
		break;
	case 3:
		if ( 0.0f <= edge1() )
		{
			if ( 0.0f <= face1() )
				triangleCBA();
			else if ( b3Dot( ac, A ) < 0.0f )
				segmentCA();
			else
				vertexA();
		}
		else if ( face2() < 0.0f )
		{
			if ( 0.0f <= b3Dot( ab, A ) )
				vertexA();
			else
				segmentAB();
		}
		else if ( 0.0f <= edge2() )
			triangleABD();
		else
			segmentDA();
		break;
	case 5:
		if ( 0.0f <= edge3() )
		{
			if ( 0.0f <= face3() )
				triangleCAD();
			else if ( b3Dot( ad, A ) < 0.0f )
				segmentDA();
			else
				vertexA();
		}
		else if ( 0.0f <= face1() )
		{
			if ( 0.0f <= edge1() )
				triangleCBA();
			else
				segmentAB();
		}
		else if ( b3Dot( ac, A ) < 0.0f )
			segmentCA();
		else
			vertexA();
		break;
	case 6:
		if ( 0.0f <= edge2() )
		{
			if ( 0.0f <= face2() )
				triangleABD();
			else if ( 0.0f <= b3Dot( ab, A ) )
				vertexA();
			else
				segmentAB();
		}
		else if ( face3() < 0.0f )
		{
			if ( b3Dot( ad, A ) < 0.0f )
				segmentDA();
			else
				vertexA();
		}
		else if ( 0.0f <= edge3() )
			triangleCAD();
		else
			segmentCA();
		break;
	default:
		if ( b3Dot( ac, A ) < 0.0f )
			segmentCA();
		else if ( b3Dot( ab, A ) < 0.0f )
			segmentAB();
		else if ( 0.0f <= b3Dot( ad, A ) )
			vertexA();
		else
			segmentDA();
		break;
	}

	return false;
}

static bool AddSimplexVertex( SolverSimplex_t &simplex, const SolverVertex_t &vertex, b3Vec3 &v )
{
	switch ( simplex.m_nCount )
	{
	case 0:
		simplex.m_nCount = 1;
		simplex.m_Vertices[0] = vertex;
		v = vertex.m_Point;
		return false;
	case 1:
		UpdateLine( simplex, vertex, v );
		return false;
	case 2:
		UpdateTriangle( simplex, vertex, v );
		return false;
	case 3:
		return UpdateTetrahedron( simplex, vertex, v );
	default:
		return false;
	}
}

template<int nCount>
static int SelectExitFace( const float *pDistances )
{
	int nBest = nCount - 1;
	float flBest = pDistances[nBest];
	for ( int i = 0; i < nCount - 1; ++i )
	{
		const bool bCloser = !( flBest <= pDistances[i] );
		nBest = bCloser ? i : nBest;
		flBest = bCloser ? pDistances[i] : flBest;
	}
	return nBest;
}

struct ExitFaces_t
{
	alignas( 16 ) float m_NormalX[4];
	alignas( 16 ) float m_NormalY[4];
	alignas( 16 ) float m_NormalZ[4];
	alignas( 16 ) float m_Distances[4];

	b3Vec3 GetNormal( int nFace ) const { return { m_NormalX[nFace], m_NormalY[nFace], m_NormalZ[nFace] }; }
};

static ExitFaces_t ComputeExitFaces( const b3Vec3 ( &a )[4], const b3Vec3 ( &b )[4], const b3Vec3 ( &points )[4], b3Vec3 direction )
{
	const __m128 ax = _mm_setr_ps( a[0].x, a[1].x, a[2].x, a[3].x );
	const __m128 ay = _mm_setr_ps( a[0].y, a[1].y, a[2].y, a[3].y );
	const __m128 az = _mm_setr_ps( a[0].z, a[1].z, a[2].z, a[3].z );
	const __m128 bx = _mm_setr_ps( b[0].x, b[1].x, b[2].x, b[3].x );
	const __m128 by = _mm_setr_ps( b[0].y, b[1].y, b[2].y, b[3].y );
	const __m128 bz = _mm_setr_ps( b[0].z, b[1].z, b[2].z, b[3].z );
	const __m128 nx = _mm_sub_ps( _mm_mul_ps( ay, bz ), _mm_mul_ps( az, by ) );
	const __m128 ny = _mm_sub_ps( _mm_mul_ps( az, bx ), _mm_mul_ps( ax, bz ) );
	const __m128 nz = _mm_sub_ps( _mm_mul_ps( ax, by ), _mm_mul_ps( ay, bx ) );

	const __m128 px = _mm_setr_ps( points[0].x, points[1].x, points[2].x, points[3].x );
	const __m128 py = _mm_setr_ps( points[0].y, points[1].y, points[2].y, points[3].y );
	const __m128 pz = _mm_setr_ps( points[0].z, points[1].z, points[2].z, points[3].z );
	const __m128 approach = Dot( _mm_set1_ps( direction.x ), _mm_set1_ps( direction.y ), _mm_set1_ps( direction.z ), nx, ny, nz );
	const __m128 distance = _mm_div_ps( Dot( px, py, pz, nx, ny, nz ), approach );
	const __m128 receding = _mm_cmple_ps( approach, _mm_setzero_ps() );

	ExitFaces_t faces;
	_mm_store_ps( faces.m_NormalX, nx );
	_mm_store_ps( faces.m_NormalY, ny );
	_mm_store_ps( faces.m_NormalZ, nz );
	_mm_store_ps( faces.m_Distances, _mm_or_ps( _mm_and_ps( receding, _mm_set1_ps( SOLVER_FAR_DISTANCE ) ), _mm_andnot_ps( receding, distance ) ) );
	return faces;
}

static float RayPlaneDistance( b3Vec3 normal, b3Vec3 point, b3Vec3 direction )
{
	const float flApproach = b3Dot( direction, normal );
	if ( flApproach <= 0.0f )
		return SOLVER_FAR_DISTANCE;
	return b3Dot( point, normal ) / flApproach;
}

static b3Vec3 BeginExitSearch( SolverSimplex_t &simplex, b3Vec3 direction )
{
	SolverVertex_t *pVertices = simplex.m_Vertices;
	const b3Vec3 apex = pVertices[3].m_Point;
	const b3Vec3 d0 = b3Sub( pVertices[0].m_Point, apex );
	const b3Vec3 d1 = b3Sub( pVertices[1].m_Point, apex );
	const b3Vec3 d2 = b3Sub( pVertices[2].m_Point, apex );
	const b3Vec3 e1 = b3Sub( pVertices[1].m_Point, pVertices[0].m_Point );
	const b3Vec3 e2 = b3Sub( pVertices[2].m_Point, pVertices[0].m_Point );

	const ExitFaces_t faces = ComputeExitFaces( { d0, d1, d2, e2 }, { d1, d2, d0, e1 }, { apex, apex, apex, pVertices[0].m_Point }, direction );
	const int nFace = SelectExitFace<4>( faces.m_Distances );
	simplex.m_nCount = 3;
	switch ( nFace )
	{
	case 0:
		pVertices[2] = pVertices[3];
		break;
	case 1:
		pVertices[0] = pVertices[3];
		break;
	case 2:
		pVertices[1] = pVertices[3];
		break;
	default:
		std::swap( pVertices[1], pVertices[2] );
		break;
	}
	return faces.GetNormal( nFace );
}

static b3Vec3 ContinueExitSearch( SolverSimplex_t &simplex, b3Vec3 direction )
{
	static constexpr int s_ReplacedVertex[3] = { 2, 0, 1 };

	SolverVertex_t *pVertices = simplex.m_Vertices;
	const b3Vec3 apex = pVertices[3].m_Point;
	const b3Vec3 d0 = b3Sub( pVertices[0].m_Point, apex );
	const b3Vec3 d1 = b3Sub( pVertices[1].m_Point, apex );
	const b3Vec3 d2 = b3Sub( pVertices[2].m_Point, apex );

	const ExitFaces_t faces = ComputeExitFaces( { d0, d1, d2, d2 }, { d1, d2, d0, d0 }, { apex, apex, apex, apex }, direction );
	const int nFace = SelectExitFace<3>( faces.m_Distances );
	simplex.m_nCount = 3;
	pVertices[s_ReplacedVertex[nFace]] = pVertices[3];
	return faces.GetNormal( nFace );
}

static float GrazeExitDistance( const SolverSimplex_t &simplex, b3Vec3 direction, float flEpsilon )
{
	const b3Vec3 v0 = simplex.m_Vertices[0].m_Point;
	const b3Vec3 v1 = simplex.m_Vertices[1].m_Point;
	const b3Vec3 v2 = simplex.m_Vertices[2].m_Point;

	const b3Vec3 a = b3Sub( v0, v2 );
	const b3Vec3 b = b3Sub( v1, v2 );
	const b3Vec3 c = b3Sub( v1, v0 );
	const b3Vec3 unit = Normalize( b3Cross( a, b ) );

	const b3Vec3 sides[3] = { b3Cross( a, unit ), b3Cross( unit, b ), b3Cross( c, unit ) };
	const b3Vec3 origins[3] = { v0, v1, v1 };

	b3Vec3 units[3];
	float distances[3];
	for ( int i = 0; i < 3; ++i )
	{
		const float s = ReciprocalLength( b3LengthSquared( sides[i] ) + SOLVER_LENGTH_EPSILON );
		units[i] = b3MulSV( s, sides[i] );
		const float flApproach = b3Dot( units[i], direction );
		const float flOffset = i == 0 ? b3Dot( units[i], origins[i] ) : b3Dot( b3MulSV( s, origins[i] ), sides[i] );
		distances[i] = flApproach <= 0.0f ? SOLVER_FAR_DISTANCE : flOffset / flApproach;
	}

	const float flNearest = distances[0] <= distances[1] ? distances[0] : distances[1];
	int nFace = distances[1] < distances[0] ? 1 : 0;
	if ( distances[2] < flNearest )
		nFace = 2;

	const float flApproach = b3Dot( direction, units[nFace] );
	if ( !( 0.0f < flApproach ) )
		return SOLVER_FAR_DISTANCE;

	return ( distances[2] <= flNearest ? distances[2] : flNearest ) + flEpsilon / flApproach;
}

template<typename MovingShape, typename TargetShape>
static b3Vec3 ComputeContactNormal( const MovingShape &moving, const TargetShape &target, const TraceRay_t &ray, const SolverSimplex_t &simplex )
{
	const auto contains = []( const int *pIndices, int nCount, int nIndex ) {
		for ( int i = 0; i < nCount; ++i )
		{
			if ( pIndices[i] == nIndex )
				return true;
		}
		return false;
	};

	int movingIndices[4] = { simplex.m_Vertices[0].m_nMovingIndex & VERTEX_INDEX_MASK };
	int targetIndices[4] = { simplex.m_Vertices[0].m_nTargetIndex };
	int nMoving = 1;
	int nTarget = 1;
	for ( int i = 1; i < simplex.m_nCount; ++i )
	{
		const int nTargetIndex = simplex.m_Vertices[i].m_nTargetIndex;
		if ( !contains( targetIndices, nTarget, nTargetIndex ) )
			targetIndices[nTarget++] = nTargetIndex;

		const int nMovingIndex = simplex.m_Vertices[i].m_nMovingIndex & VERTEX_INDEX_MASK;
		if ( !contains( movingIndices, nMoving, nMovingIndex ) )
			movingIndices[nMoving++] = nMovingIndex;
	}

	const auto facingAgainstRay = [&]( b3Vec3 n ) {
		const b3Vec3 normal = Normalize( n );
		return 0.0f < b3Dot( normal, ray.m_Delta ) ? b3Neg( normal ) : normal;
	};

	if ( simplex.m_nCount == 2 && nMoving == 2 )
	{
		const b3Vec3 edge = b3Sub( moving.GetVertex( movingIndices[1] ), moving.GetVertex( movingIndices[0] ) );
		return Normalize( b3Cross( edge, b3Cross( edge, ray.m_Delta ) ) );
	}

	if ( nMoving == 3 )
	{
		const b3Vec3 m0 = moving.GetVertex( movingIndices[0] );
		return facingAgainstRay( b3Cross( b3Sub( moving.GetVertex( movingIndices[1] ), m0 ), b3Sub( moving.GetVertex( movingIndices[2] ), m0 ) ) );
	}

	if ( nMoving == 2 && nTarget == 2 )
	{
		const b3Vec3 movingEdge = b3Sub( moving.GetVertex( movingIndices[1] ), moving.GetVertex( movingIndices[0] ) );
		const b3Vec3 targetEdge = b3Sub( target.GetVertex( targetIndices[1] ), target.GetVertex( targetIndices[0] ) );
		return facingAgainstRay( b3Cross( movingEdge, targetEdge ) );
	}

	if ( nTarget == 3 )
	{
		const b3Vec3 t0 = target.GetVertex( targetIndices[0] );
		return facingAgainstRay( b3Cross( b3Sub( target.GetVertex( targetIndices[1] ), t0 ), b3Sub( target.GetVertex( targetIndices[2] ), t0 ) ) );
	}

	return b3Neg( ray.m_Direction );
}

template<typename MovingShape, typename TargetShape>
static LedgeHit_t SolveLedge( MovingShape &moving, TargetShape &target, const TraceRay_t &ray, float flEpsilon, b3Vec3 &searchVector )
{
	const auto support = [&]( b3Vec3 direction ) {
		const SupportPoint_t movingPoint = moving.Support( direction );
		const bool bEnd = b3Dot( direction, ray.m_Delta ) > 0.0f;
		const b3Vec3 movedPoint = b3Add( movingPoint.m_Point, bEnd ? ray.m_End : ray.m_Start );
		const SupportPoint_t targetPoint = target.Support( b3Neg( direction ) );
		return SolverVertex_t{
			b3Sub( movedPoint, targetPoint.m_Point ),
			static_cast<unsigned short>( ( movingPoint.m_nIndex & VERTEX_INDEX_MASK ) | ( bEnd ? SEGMENT_END_FLAG : 0 ) ),
			static_cast<unsigned short>( targetPoint.m_nIndex ),
		};
	};

	LedgeHit_t hit;
	SolverSimplex_t simplex;
	b3Vec3 direction = Normalize( b3Neg( searchVector ) );
	SolverVertex_t w;
	float flSeparation;
	bool bEnclosed = false;
	for ( int nIteration = 0;; )
	{
		w = support( direction );
		flSeparation = b3Dot( w.m_Point, direction );
		if ( flSeparation < 0.0f )
			break;

		if ( AddSimplexVertex( simplex, w, searchVector ) )
		{
			bEnclosed = true;
			break;
		}

		if ( ++nIteration >= MAX_SOLVER_ITERATIONS )
			return hit;

		direction = Normalize( b3Neg( searchVector ) );
	}

	if ( bEnclosed )
	{
		hit.m_bHit = true;
		if ( ray.m_flCurrentLength == 0.0f )
		{
			hit.m_bStartSolid = true;
			return hit;
		}

		float flExit = 0.0f;
		b3Vec3 normal = BeginExitSearch( simplex, ray.m_Direction );
		for ( int nIteration = 0;; )
		{
			normal = Normalize( normal );

			const SolverVertex_t exit = support( normal );
			const b3Vec3 v0 = simplex.m_Vertices[0].m_Point;
			if ( b3Dot( exit.m_Point, normal ) <= b3Dot( v0, normal ) + EXIT_PROGRESS_TOLERANCE )
			{
				const b3Vec3 e1 = b3Sub( simplex.m_Vertices[1].m_Point, v0 );
				const b3Vec3 e2 = b3Sub( simplex.m_Vertices[2].m_Point, v0 );
				flExit = RayPlaneDistance( b3Cross( e1, e2 ), v0, ray.m_Direction );
				break;
			}

			simplex.m_Vertices[simplex.m_nCount++] = exit;
			normal = ContinueExitSearch( simplex, ray.m_Direction );
			if ( ++nIteration >= MAX_SOLVER_ITERATIONS )
			{
				flExit = 0.0f;
				break;
			}
		}

		if ( !( flExit < ray.m_flCurrentLength && 0.0f < flExit ) )
		{
			hit.m_bStartSolid = true;
			return hit;
		}

		hit.m_flDistance = ray.m_flCurrentLength - flExit;
		hit.m_Normal = ComputeContactNormal( moving, target, ray, simplex );
		const float flApproach = b3Dot( hit.m_Normal, ray.m_Direction );
		if ( flApproach < 0.0f )
			hit.m_flDistance = flEpsilon / flApproach + hit.m_flDistance;
		if ( hit.m_flDistance < 0.0f )
			hit.m_flDistance = 0.0f;
		return hit;
	}

	float flGap = std::fabs( flSeparation );
	if ( flEpsilon <= flGap || ray.m_flCurrentLength <= 0.0f )
		return hit;

	b3Vec3 grazeDirection = direction;
	if ( 0.0f < flGap )
	{
		for ( int nIteration = 0; nIteration < MAX_GRAZE_ITERATIONS; ++nIteration )
		{
			const b3Vec3 previous = w.m_Point;
			AddSimplexVertex( simplex, w, searchVector );
			direction = Normalize( b3Neg( searchVector ) );
			w = support( direction );

			const float flStep = -b3Dot( w.m_Point, direction );
			if ( flEpsilon < flStep )
				return hit;

			if ( flGap < flStep )
			{
				flGap = flStep;
				grazeDirection = direction;
			}

			if ( GRAZE_PROGRESS_TOLERANCE < flStep + b3Dot( direction, previous ) )
				break;
		}
	}

	const float flApproach = -b3Dot( grazeDirection, ray.m_Delta );
	if ( !( static_cast<double>( flApproach ) < static_cast<double>( flEpsilon ) * GRAZE_APPROACH_FACTOR ) && ( GRAZE_MINIMUM_APPROACH <= flApproach || static_cast<double>( flEpsilon ) * GRAZE_DISTANCE_FACTOR <= static_cast<double>( flGap ) ) )
		return hit;

	hit.m_bHit = true;
	hit.m_flDistance = ray.m_flCurrentLength - ( ( flGap - flEpsilon ) * ray.m_flLength ) / flApproach;
	if ( hit.m_flDistance < 0.0f )
	{
		hit.m_flDistance = 0.0f;
		const float flExit = simplex.m_nCount == 3 ? GrazeExitDistance( simplex, ray.m_Direction, flEpsilon ) : 0.0f;
		if ( flExit < ray.m_flCurrentLength )
			hit.m_flDistance = ray.m_flCurrentLength - flExit;
	}
	hit.m_Normal = b3Neg( grazeDirection );
	return hit;
}

struct VisitedPoints_t
{
	uint32_t m_Slots[VISITED_SLOT_COUNT] = {};
	uint16_t m_nGeneration = 1;

	void Begin()
	{
		if ( ++m_nGeneration == 0 )
			std::memset( m_Slots, 0, sizeof( m_Slots ) );
	}

	bool Visit( uint32_t nPoint )
	{
		const uint32_t nStamp = m_nGeneration | ( ( nPoint & 0xFFFF ) << 16 );
		uint32_t &nSlot = m_Slots[nPoint % VISITED_SLOT_COUNT];
		if ( nSlot == nStamp )
			return false;
		nSlot = nStamp;
		return true;
	}
};

static thread_local VisitedPoints_t s_VisitedPoints;

struct alignas( 16 ) PointBlock_t
{
	float m_X[LANE_COUNT];
	float m_Y[LANE_COUNT];
	float m_Z[LANE_COUNT];
};

static thread_local std::vector<PointBlock_t> s_ReusedPointBlocks;

static const Vector ZERO_ORIGIN( 0.0f, 0.0f, 0.0f );

class CTraceCollide
{
public:
	CTraceCollide( const CPhysCollideCompactSurface &collide, const Vector &origin, const QAngle &angles )
	: m_Collide( collide )
	, m_bHasOrigin( origin.x != 0.0f || origin.y != 0.0f || origin.z != 0.0f )
	, m_pVisited( &s_VisitedPoints )
	{
		b3Vec3 forward, right, up;
		AngleVectors( angles, forward, right, up );
		m_Rotation[0] = { forward.x, -up.x, -right.x };
		m_Rotation[1] = { -forward.z, up.z, right.z };
		m_Rotation[2] = { forward.y, -up.y, -right.y };

		const b3Vec3 rows[3] = {
			b3MulSV( IVP_TO_SOURCE, ToFloat( m_Rotation[0] ) ),
			b3MulSV( IVP_TO_SOURCE, ToFloat( m_Rotation[2] ) ),
			b3Neg( b3MulSV( IVP_TO_SOURCE, ToFloat( m_Rotation[1] ) ) ),
		};
		const b3Vec3 translation = IVPToSource( SourceToIVP( origin ) );
		for ( int i = 0; i < 3; ++i )
		{
			m_World[i][0] = rows[i].x;
			m_World[i][1] = rows[i].y;
			m_World[i][2] = rows[i].z;
		}
		m_World[0][3] = translation.x;
		m_World[1][3] = translation.y;
		m_World[2][3] = translation.z;
	}

	const CPhysCollideCompactSurface &GetCollide() const { return m_Collide; }

	b3Vec3 RotateToLocal( b3Vec3 floatDirection ) const
	{
		const DoubleVec3_t direction = ToDouble( floatDirection );
		return ToFloat( {
			( direction.x * m_Rotation[0].x + direction.y * m_Rotation[1].x ) + direction.z * m_Rotation[2].x,
			( direction.x * m_Rotation[0].y + direction.y * m_Rotation[1].y ) + direction.z * m_Rotation[2].y,
			( direction.x * m_Rotation[0].z + direction.y * m_Rotation[1].z ) + direction.z * m_Rotation[2].z,
		} );
	}

	CPhysCollideCompactSurface::LedgeSphere_t GetWorldSphere( int nLedge ) const
	{
		const CPhysCollideCompactSurface::LedgeSphere_t &sphere = m_Collide.GetLedgeSphere( nLedge );
		return { TransformToWorld( sphere.m_Center ), sphere.m_flRadius * ( IVP_TO_SOURCE * SPHERE_BOUND_SCALE ) };
	}

	SupportBound_t GetSupportBound( b3Vec3 direction ) const
	{
		return { b3Dot( m_WorldSphere.m_Center, direction ) + m_WorldSphere.m_flRadius * SPHERE_BOUND_SCALE, SumAbsolute( m_WorldSphere.m_Center ) + m_WorldSphere.m_flRadius };
	}

	void ReuseWorldPoints()
	{
		if ( m_Collide.GetPointBlockCount() == 0 )
			return;

		std::vector<PointBlock_t> &blocks = s_ReusedPointBlocks;
		if ( blocks.size() < static_cast<size_t>( m_Collide.GetPointBlockCount() ) )
			blocks.resize( m_Collide.GetPointBlockCount() );
		m_pReusedBlocks = blocks.data();
		m_nBuiltLedges = 0;
	}

	void SetLedge( int nLedge )
	{
		m_pLedge = m_Collide.GetLedge( nLedge );
		m_pPoints = m_pLedge + GetLedgePointOffset( m_pLedge );
		m_nTriangleCount = GetLedgeTriangleCount( m_pLedge );
		m_pInfo = m_Collide.GetCollideMapEntry( nLedge );
		m_WorldSphere = GetWorldSphere( nLedge );
		m_nBlockCount = 0;
		if ( m_bHasOrigin || !m_pInfo || !( m_pInfo->m_nFlags & ( COLLIDE_MAP_CONTIGUOUS | COLLIDE_MAP_RUNS ) ) )
			return;

		m_nBlockCount = ( m_pInfo->m_nCount + LANE_COUNT - 1 ) / LANE_COUNT;
		if ( !m_pReusedBlocks )
		{
			m_pBlocks = m_Blocks;
			BuildWorldPoints();
			return;
		}

		m_pBlocks = m_pReusedBlocks + m_pInfo->m_nPointBlockOffset;
		const uint32_t nLedgeBit = 1u << nLedge;
		if ( !( m_nBuiltLedges & nLedgeBit ) )
		{
			BuildWorldPoints();
			m_nBuiltLedges |= nLedgeBit;
		}
	}

	SupportPoint_t Support( b3Vec3 direction ) const
	{
		return m_nBlockCount > 0 ? ScanWorldPoints( direction ) : ClimbLedge( direction );
	}

	b3Vec3 GetVertex( int nIndex ) const
	{
		if ( m_nBlockCount > 0 )
		{
			const PointBlock_t &block = m_pBlocks[static_cast<unsigned int>( nIndex ) / LANE_COUNT];
			const unsigned int nLane = static_cast<unsigned int>( nIndex ) % LANE_COUNT;
			return { block.m_X[nLane], block.m_Y[nLane], block.m_Z[nLane] };
		}
		return TransformToWorld( LoadLedgePoint( static_cast<uint32_t>( nIndex ) ) );
	}

private:
	b3Vec3 LoadLedgePoint( uint32_t nPoint ) const { return LoadPoint( m_pPoints + nPoint * COMPACT_POINT_SIZE ); }

	b3Vec3 TransformToWorld( b3Vec3 p ) const
	{
		const auto row = [&]( const float ( &m )[4] ) { return ( m[0] * p.x + m[1] * p.y ) + ( m[2] * p.z + m[3] ); };
		return { row( m_World[0] ), row( m_World[1] ), row( m_World[2] ) };
	}

	void BuildWorldPoints()
	{
		uint32_t points[MAX_COLLIDE_MAP_SPAN + LANE_COUNT];
		int nCount = 0;
		if ( m_pInfo->m_nFlags & COLLIDE_MAP_CONTIGUOUS )
		{
			for ( ; nCount < m_pInfo->m_nCount; ++nCount )
				points[nCount] = m_pInfo->m_nStart + static_cast<uint32_t>( nCount );
		}
		else
		{
			uint32_t nPoint = m_pInfo->m_nStart;
			for ( int nRun = 0; nRun < m_pInfo->m_nRunCount; ++nRun )
			{
				for ( int i = 0; i < m_pInfo->m_Runs[nRun]; ++i, ++nPoint )
				{
					if ( nRun % 2 == 0 )
						points[nCount++] = nPoint;
				}
			}
		}

		for ( int i = nCount; i < m_nBlockCount * LANE_COUNT; ++i )
			points[i] = points[nCount - 1];

		const __m128 m00 = _mm_set1_ps( m_World[0][0] ), m01 = _mm_set1_ps( m_World[0][1] ), m02 = _mm_set1_ps( m_World[0][2] );
		const __m128 m10 = _mm_set1_ps( m_World[1][0] ), m11 = _mm_set1_ps( m_World[1][1] ), m12 = _mm_set1_ps( m_World[1][2] );
		const __m128 m20 = _mm_set1_ps( m_World[2][0] ), m21 = _mm_set1_ps( m_World[2][1] ), m22 = _mm_set1_ps( m_World[2][2] );
		for ( int nBlock = 0; nBlock < m_nBlockCount; ++nBlock )
		{
			const uint32_t *pBlockPoints = points + nBlock * LANE_COUNT;
			__m128 x = _mm_loadu_ps( reinterpret_cast<const float *>( m_pPoints + pBlockPoints[0] * COMPACT_POINT_SIZE ) );
			__m128 y = _mm_loadu_ps( reinterpret_cast<const float *>( m_pPoints + pBlockPoints[1] * COMPACT_POINT_SIZE ) );
			__m128 z = _mm_loadu_ps( reinterpret_cast<const float *>( m_pPoints + pBlockPoints[2] * COMPACT_POINT_SIZE ) );
			__m128 w = _mm_loadu_ps( reinterpret_cast<const float *>( m_pPoints + pBlockPoints[3] * COMPACT_POINT_SIZE ) );
			_MM_TRANSPOSE4_PS( x, y, z, w );
			_mm_store_ps( m_pBlocks[nBlock].m_X, _mm_add_ps( _mm_add_ps( _mm_mul_ps( x, m00 ), _mm_mul_ps( y, m01 ) ), _mm_mul_ps( z, m02 ) ) );
			_mm_store_ps( m_pBlocks[nBlock].m_Y, _mm_add_ps( _mm_add_ps( _mm_mul_ps( x, m10 ), _mm_mul_ps( y, m11 ) ), _mm_mul_ps( z, m12 ) ) );
			_mm_store_ps( m_pBlocks[nBlock].m_Z, _mm_add_ps( _mm_add_ps( _mm_mul_ps( x, m20 ), _mm_mul_ps( y, m21 ) ), _mm_mul_ps( z, m22 ) ) );
		}
	}

	SupportPoint_t ScanWorldPoints( b3Vec3 direction ) const
	{
		const __m128 dx = _mm_set1_ps( direction.x );
		const __m128 dy = _mm_set1_ps( direction.y );
		const __m128 dz = _mm_set1_ps( direction.z );
		const auto dot = [&]( int nBlock ) {
			return _mm_add_ps( _mm_add_ps( _mm_mul_ps( dx, _mm_load_ps( m_pBlocks[nBlock].m_X ) ), _mm_mul_ps( dy, _mm_load_ps( m_pBlocks[nBlock].m_Y ) ) ), _mm_mul_ps( dz, _mm_load_ps( m_pBlocks[nBlock].m_Z ) ) );
		};

		__m128 best = dot( 0 );
		__m128i bestIndex = _mm_setr_epi32( 0, 1, 2, 3 );
		__m128i index = bestIndex;
		const __m128i step = _mm_set1_epi32( LANE_COUNT );
		for ( int nBlock = 1; nBlock < m_nBlockCount; ++nBlock )
		{
			index = _mm_add_epi32( index, step );
			const __m128 value = dot( nBlock );
			const __m128i greater = _mm_castps_si128( _mm_cmplt_ps( best, value ) );
			bestIndex = _mm_or_si128( _mm_and_si128( greater, index ), _mm_andnot_si128( greater, bestIndex ) );
			best = _mm_max_ps( value, best );
		}

		const __m128 swappedBest = _mm_shuffle_ps( best, best, _MM_SHUFFLE( 1, 0, 3, 2 ) );
		const __m128i swappedIndex = _mm_shuffle_epi32( bestIndex, _MM_SHUFFLE( 1, 0, 3, 2 ) );
		const __m128i swapped = _mm_castps_si128( _mm_cmplt_ps( best, swappedBest ) );
		const __m128i pairIndex = _mm_or_si128( _mm_and_si128( swapped, swappedIndex ), _mm_andnot_si128( swapped, bestIndex ) );
		const __m128 pairBest = _mm_max_ps( swappedBest, best );
		const __m128i odd = _mm_castps_si128( _mm_cmplt_ss( pairBest, _mm_shuffle_ps( pairBest, pairBest, _MM_SHUFFLE( 1, 1, 1, 1 ) ) ) );
		const __m128i oddIndex = _mm_shuffle_epi32( pairIndex, _MM_SHUFFLE( 1, 1, 1, 1 ) );
		const int nIndex = _mm_cvtsi128_si32( _mm_or_si128( _mm_and_si128( odd, oddIndex ), _mm_andnot_si128( odd, pairIndex ) ) );
		return { GetVertex( nIndex ), nIndex };
	}

	SupportPoint_t ClimbLedge( b3Vec3 direction ) const
	{
		const b3Vec3 local = RotateToLocal( { direction.x, -direction.z, direction.y } );
		const auto dot = [&]( uint32_t nPoint ) {
			const b3Vec3 point = LoadLedgePoint( nPoint );
			return b3Dot( local, point );
		};

		size_t nEdge = FIRST_EDGE_OFFSET;
		if ( m_pInfo && ( m_pInfo->m_nFlags & COLLIDE_MAP_OCTANTS ) )
		{
			const int nOctant = ( local.x < 0.0f ? 1 : 0 ) + ( local.y < 0.0f ? 2 : 0 ) + ( local.z < 0.0f ? 4 : 0 );
			const uint16_t nCode = m_pInfo->m_OctantEdges[nOctant];
			nEdge = FIRST_EDGE_OFFSET + 4 * ( ( nCode & 3 ) + ( nCode >> 2 ) * 4 );
		}

		VisitedPoints_t &visited = *m_pVisited;
		visited.Begin();

		uint32_t nBest = GetEdgeStartPointAt( m_pLedge, nEdge );
		visited.Visit( nBest );
		float flBest = dot( nBest );

		for ( int n = 0; n < m_nTriangleCount; ++n )
		{
			nEdge = GetPreviousEdge( nEdge );
			const uint32_t nFirst = GetEdgeStartPointAt( m_pLedge, nEdge );
			uint32_t nPoint = nFirst;
			bool bImproved = false;
			do
			{
				if ( visited.Visit( nPoint ) )
				{
					const float flDot = dot( nPoint );
					if ( flBest < flDot )
					{
						nBest = nPoint;
						flBest = flDot;
						bImproved = true;
						break;
					}
				}

				nEdge = GetPreviousEdge( GetOppositeEdge( m_pLedge, nEdge ) );
				nPoint = GetEdgeStartPointAt( m_pLedge, nEdge );
			} while ( nPoint != nFirst );

			if ( !bImproved && nPoint != nBest )
				break;
		}

		return { TransformToWorld( LoadLedgePoint( nBest ) ), static_cast<int>( nBest ) };
	}

	const CPhysCollideCompactSurface &m_Collide;
	bool m_bHasOrigin;
	VisitedPoints_t *m_pVisited;
	DoubleVec3_t m_Rotation[3];
	float m_World[3][4];
	const unsigned char *m_pLedge = nullptr;
	const unsigned char *m_pPoints = nullptr;
	int m_nTriangleCount = 0;
	const CollideMapEntry_t *m_pInfo = nullptr;
	CPhysCollideCompactSurface::LedgeSphere_t m_WorldSphere = {};
	int m_nBlockCount = 0;
	PointBlock_t *m_pBlocks = nullptr;
	PointBlock_t *m_pReusedBlocks = nullptr;
	uint32_t m_nBuiltLedges = 0;
	PointBlock_t m_Blocks[MAX_POINT_BLOCKS];
};

class CTracePoint
{
public:
	float GetRadius() const { return 0.0f; }

	SupportPoint_t Support( b3Vec3 ) const { return { b3Vec3_zero, 0 }; }

	b3Vec3 GetVertex( int ) const { return b3Vec3_zero; }

	SupportBound_t GetSupportBound( b3Vec3 ) const { return { 0.0f, 0.0f }; }
};

class CTraceBox
{
public:
	explicit CTraceBox( const Vector &extents )
	{
		m_Extents[0][0] = extents.x;
		m_Extents[0][1] = -extents.x;
		m_Extents[1][0] = extents.y;
		m_Extents[1][1] = -extents.y;
		m_Extents[2][0] = extents.z;
		m_Extents[2][1] = -extents.z;
		m_flRadius = std::sqrt( ( extents.x * extents.x + extents.y * extents.y ) + extents.z * extents.z );
	}

	float GetRadius() const { return m_flRadius; }

	SupportPoint_t Support( b3Vec3 direction ) const
	{
		const int nX = static_cast<int>( BitCast<uint32_t>( direction.x ) >> 31 );
		const int nY = static_cast<int>( BitCast<uint32_t>( direction.y ) >> 31 );
		const int nZ = static_cast<int>( BitCast<uint32_t>( direction.z ) >> 31 );
		return { { m_Extents[0][nX], m_Extents[1][nY], m_Extents[2][nZ] }, ( nZ * 2 | nY ) * 2 | nX };
	}

	b3Vec3 GetVertex( int nIndex ) const { return { m_Extents[0][nIndex & 1], m_Extents[1][( nIndex >> 1 ) & 1], m_Extents[2][( nIndex >> 2 ) & 1] }; }

	SupportBound_t GetSupportBound( b3Vec3 direction ) const
	{
		const b3Vec3 point = Support( direction ).m_Point;
		return { b3Dot( point, direction ), SumAbsolute( point ) };
	}

private:
	float m_Extents[3][2];
	float m_flRadius;
};

struct TraceResult_t
{
	bool m_bHit = false;
	bool m_bStartSolid = false;
	float m_flFraction = 1.0f;
	b3Vec3 m_Normal = b3Vec3_zero;
	unsigned int m_nContents = 0;
};

static TraceRay_t CreateTraceRay( b3Vec3 start, b3Vec3 end, b3Vec3 delta )
{
	TraceRay_t ray = { start, end, delta, delta, 0.0f, 0.0f, 0.0f };
	ray.m_flLength = b3Length( delta );
	ray.m_flCurrentLength = ray.m_flLength;
	if ( 0.0f < ray.m_flLength )
	{
		ray.m_flInverseLength = 1.0f / ray.m_flLength;
		ray.m_Direction = b3MulSV( ray.m_flInverseLength, delta );
	}
	return ray;
}

static void ShortenTraceRay( TraceRay_t &ray, float flFraction )
{
	ray.m_flCurrentLength = flFraction * ray.m_flLength;
	ray.m_End = b3MulAdd( ray.m_Start, flFraction, ray.m_Delta );
}

class CLedgeTreeCuller
{
public:
	CLedgeTreeCuller( const CTraceCollide &cache, const TraceRay_t &ray, float flMovingRadius )
	: m_Start( cache.RotateToLocal( SourceToIVP( ray.m_Start ) ) )
	, m_Delta( cache.RotateToLocal( SourceToIVP( ray.m_Delta ) ) )
	, m_flMovingRadius( flMovingRadius )
	, m_flLength( SOURCE_TO_IVP * ray.m_flCurrentLength )
	{
		m_Direction = m_Delta;
		const float flLengthSquared = b3LengthSquared( m_Delta );
		if ( !( flLengthSquared < DOUBLE_RESOLUTION ) )
		{
			m_Direction = ToFloat( ApproximateInverseSquareRoot( flLengthSquared, 4 ) * ToDouble( m_Delta ) );
		}
		m_Midpoint = b3MulAdd( m_Start, 0.5f, m_Delta );
	}

	void Shorten( float flFraction, float flCurrentLength )
	{
		const float flHalf = flFraction * 0.5f;
		m_flLength = SOURCE_TO_IVP * flCurrentLength;
		m_Midpoint = b3MulAdd( m_Start, flHalf, m_Delta );
	}

	bool Passes( const CPhysCollideCompactSurface::LedgeTreeNode_t &node ) const
	{
		const b3Vec3 d = b3Sub( node.m_Center, m_Midpoint );
		const float flDistanceSquared = 0.0f < m_flLength ? b3LengthSquared( b3Cross( m_Direction, d ) ) : b3LengthSquared( d );
		const float flReach = node.m_flRadius + m_flMovingRadius;
		return flDistanceSquared < flReach * flReach;
	}

	float GetStartDistanceSquared( const CPhysCollideCompactSurface::LedgeTreeNode_t &node ) const
	{
		return static_cast<float>( LengthSquared( ToDouble( b3Sub( m_Start, node.m_Center ) ) ) );
	}

private:
	b3Vec3 m_Start;
	b3Vec3 m_Delta;
	b3Vec3 m_Direction;
	b3Vec3 m_Midpoint;
	float m_flMovingRadius;
	float m_flLength;
};

template<typename VisitFunction>
static void WalkLedgeTree( const CPhysCollideCompactSurface &collide, const CLedgeTreeCuller &culler, VisitFunction &&visit )
{
	static constexpr float REJECTED_DISTANCE = 1e24f;

	const std::span<const CPhysCollideCompactSurface::LedgeTreeNode_t> nodes = collide.GetLedgeTree();
	if ( nodes.empty() || !culler.Passes( nodes[0] ) )
		return;

	int stack[MAX_LEDGETREE_DEPTH + 1];
	int nStackCount = 0;

	int nNode = 0;
	for ( ;; )
	{
		while ( nNode >= 0 && nodes[nNode].m_nRightChild >= 0 )
		{
			const int nLeft = nNode + 1;
			const int nRight = nodes[nNode].m_nRightChild;
			const bool bLeft = culler.Passes( nodes[nLeft] );
			const float flLeft = bLeft ? culler.GetStartDistanceSquared( nodes[nLeft] ) : REJECTED_DISTANCE;
			const bool bRight = culler.Passes( nodes[nRight] );

			if ( bRight && bLeft )
			{
				if ( culler.GetStartDistanceSquared( nodes[nRight] ) <= flLeft )
				{
					stack[nStackCount++] = nLeft;
					nNode = nRight;
				}
				else
				{
					stack[nStackCount++] = nRight;
					nNode = nLeft;
				}
			}
			else if ( bRight )
				nNode = nRight;
			else if ( bLeft )
				nNode = nLeft;
			else
				nNode = -1;
		}

		if ( nNode >= 0 )
			visit( nodes[nNode].m_nConvex );

		if ( nStackCount == 0 )
			return;

		nNode = stack[--nStackCount];
	}
}

template<typename MovingShape>
static bool IsFirstSupportSeparated( const CTraceCollide &target, const MovingShape &moving, int nLedge, const TraceRay_t &ray, b3Vec3 searchVector, float flEpsilon )
{
	const b3Vec3 direction = Normalize( b3Neg( searchVector ) );
	const SupportBound_t movingBound = moving.GetSupportBound( direction );
	const b3Vec3 segmentPoint = b3Dot( direction, ray.m_Delta ) > 0.0f ? ray.m_End : ray.m_Start;
	const CPhysCollideCompactSurface::LedgeSphere_t sphere = target.GetWorldSphere( nLedge );

	const float flTargetLower = b3Dot( sphere.m_Center, direction ) - sphere.m_flRadius * SPHERE_BOUND_SCALE;
	const float flSeparation = ( movingBound.m_flValue + b3Dot( segmentPoint, direction ) ) - flTargetLower;
	const float flSlack = SEPARATION_SLACK * ( ( movingBound.m_flMagnitude + SumAbsolute( segmentPoint ) ) + ( SumAbsolute( sphere.m_Center ) + sphere.m_flRadius ) + 1.0f );
	return flSeparation + flSlack <= -flEpsilon;
}

template<typename MovingShape>
static void TraverseCollide( CTraceCollide &target, MovingShape &moving, TraceRay_t &ray, float flMovingRadius, float flEpsilon, unsigned int contentsMask, IConvexInfo *pConvexInfo, TraceResult_t &result )
{
	const CPhysCollideCompactSurface &collide = target.GetCollide();
	float flBest = std::max( ray.m_flLength, MINIMUM_TRACE_LENGTH );
	b3Vec3 searchVector = ray.m_Start;
	CLedgeTreeCuller culler( target, ray, flMovingRadius );

	WalkLedgeTree( collide, culler, [&]( int nLedge ) {
		const unsigned int contents = pConvexInfo ? pConvexInfo->GetContents( GetLedgeClientData( collide.GetLedge( nLedge ) ) ) : CONTENTS_SOLID;
		if ( !( contentsMask & contents ) )
			return;

		if ( searchVector.x == 0.0f && searchVector.y == 0.0f && searchVector.z == 0.0f )
			searchVector = { 1.0f, 0.0f, 0.0f };
		if ( IsFirstSupportSeparated( target, moving, nLedge, ray, searchVector, flEpsilon ) )
			return;

		target.SetLedge( nLedge );
		const LedgeHit_t hit = SolveLedge( moving, target, ray, flEpsilon, searchVector );
		if ( !hit.m_bHit || !( hit.m_flDistance < flBest ) )
			return;

		result.m_bHit = true;
		result.m_bStartSolid = hit.m_bStartSolid;
		result.m_flFraction = hit.m_flDistance * ray.m_flInverseLength;
		result.m_Normal = hit.m_Normal;
		result.m_nContents = contents;
		flBest = hit.m_flDistance;

		const float flFraction = ( ( flEpsilon + flEpsilon ) + hit.m_flDistance ) * ray.m_flInverseLength;
		if ( flFraction < 1.0f )
		{
			ShortenTraceRay( ray, flFraction );
			culler.Shorten( flFraction, ray.m_flCurrentLength );
		}
	} );
}

static void ClearTrace( trace_t *pTrace )
{
	std::memset( static_cast<void *>( pTrace ), 0, sizeof( *pTrace ) );
	pTrace->fraction = 1.0f;
	pTrace->surface.name = "**empty**";
}

static void ApplyTraceResult( const TraceResult_t &result, trace_t *pTrace )
{
	if ( !result.m_bHit )
		return;

	pTrace->fraction = result.m_flFraction;
	pTrace->contents = result.m_nContents;
	pTrace->plane.normal = ToSource( result.m_Normal );
	pTrace->startsolid = result.m_bStartSolid;
	pTrace->allsolid = result.m_bStartSolid;
}

static void FinishTrace( trace_t *pTrace )
{
	if ( pTrace->fraction < 1.0f || pTrace->allsolid || pTrace->startsolid )
		pTrace->plane.dist = pTrace->endpos.Dot( pTrace->plane.normal );
}

void TraceBoxAgainstCollide( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	ClearTrace( pTrace );

	const CPhysCollideCompactSurface *pSurface = pCollide->GetCompactSurface();
	if ( pSurface )
	{
		CTraceCollide target( *pSurface, ZERO_ORIGIN, collideAngles );

		const b3Vec3 start = ToBox3D( ray.m_Start - collideOrigin );
		const b3Vec3 delta = ToBox3D( ray.m_Delta );
		TraceRay_t traceRay = CreateTraceRay( start, b3Add( start, delta ), delta );

		TraceResult_t result;
		if ( ray.m_IsRay )
		{
			CTracePoint point;
			TraverseCollide( target, point, traceRay, point.GetRadius(), TRACE_BOX_EPSILON, contentsMask, pConvexInfo, result );
		}
		else
		{
			CTraceBox box( ray.m_Extents );
			TraverseCollide( target, box, traceRay, box.GetRadius(), TRACE_BOX_EPSILON, contentsMask, pConvexInfo, result );
		}
		ApplyTraceResult( result, pTrace );
	}

	const float flFraction = pTrace->fraction;
	pTrace->startpos = ray.m_Start + ray.m_StartOffset;
	pTrace->endpos = Vector( ( flFraction * ray.m_Delta.x + ray.m_Start.x ) + ray.m_StartOffset.x, ( flFraction * ray.m_Delta.y + ray.m_Start.y ) + ray.m_StartOffset.y, ( flFraction * ray.m_Delta.z + ray.m_Start.z ) + ray.m_StartOffset.z );
	FinishTrace( pTrace );
}

void TraceCollideAgainstCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	ClearTrace( pTrace );

	const CPhysCollideCompactSurface *pSweepSurface = pSweepCollide->GetCompactSurface();
	const CPhysCollideCompactSurface *pSurface = pCollide->GetCompactSurface();
	if ( pSweepSurface && pSurface )
	{
		CTraceCollide moving( *pSweepSurface, ZERO_ORIGIN, sweepAngles );
		CTraceCollide target( *pSurface, ZERO_ORIGIN, collideAngles );
		target.ReuseWorldPoints();

		const b3Vec3 localStart = ToBox3D( start - collideOrigin );
		const b3Vec3 localEnd = ToBox3D( end - collideOrigin );
		TraceRay_t traceRay = CreateTraceRay( localStart, localEnd, b3Sub( localEnd, localStart ) );
		const float flMovingRadius = pSweepSurface->GetBoundingRadius();

		TraceResult_t best;
		for ( int nLedge = pSweepSurface->GetLedgeCount(); nLedge-- > 0; )
		{
			moving.SetLedge( nLedge );
			TraceResult_t local;
			TraverseCollide( target, moving, traceRay, flMovingRadius, TRACE_COLLIDE_EPSILON, MASK_ALL, nullptr, local );
			if ( local.m_flFraction < best.m_flFraction )
				best = local;
		}

		ApplyTraceResult( best, pTrace );
	}

	const float flFraction = pTrace->fraction;
	pTrace->endpos = ( 1.0f - flFraction ) * start + flFraction * end;
	FinishTrace( pTrace );
}

struct Vec3d_t
{
	double x, y, z;
};

static Vec3d_t operator-( const Vec3d_t &a, const Vec3d_t &b )
{
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}

static Vec3d_t operator-( const Vec3d_t &a )
{
	return { -a.x, -a.y, -a.z };
}

static double Dot( const Vec3d_t &a, const Vec3d_t &b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static Vec3d_t Cross( const Vec3d_t &a, const Vec3d_t &b )
{
	return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

static Vec3d_t TripleCross( const Vec3d_t &a, const Vec3d_t &b, const Vec3d_t &c )
{
	return Cross( Cross( a, b ), c );
}

class CSimplex
{
public:
	void Push( const Vec3d_t &point )
	{
		for ( int i = m_nCount; i > 0; --i )
			m_Points[i] = m_Points[i - 1];
		m_Points[0] = point;
		++m_nCount;
	}

	bool Update( Vec3d_t &direction )
	{
		switch ( m_nCount )
		{
		case 2:
			return UpdateLine( direction );
		case 3:
			return UpdateTriangle( direction );
		case 4:
			return UpdateTetrahedron( direction );
		default:
			return false;
		}
	}

private:
	void Set( std::initializer_list<Vec3d_t> points )
	{
		m_nCount = 0;
		for ( const Vec3d_t &point : points )
			m_Points[m_nCount++] = point;
	}

	bool UpdateLine( Vec3d_t &direction )
	{
		const Vec3d_t a = m_Points[0];
		const Vec3d_t b = m_Points[1];
		const Vec3d_t ab = b - a;
		const Vec3d_t ao = -a;

		if ( Dot( ab, ao ) > 0.0 )
		{
			direction = TripleCross( ab, ao, ab );
			return Dot( direction, direction ) == 0.0;
		}

		Set( { a } );
		direction = ao;
		return false;
	}

	bool UpdateTriangle( Vec3d_t &direction )
	{
		const Vec3d_t a = m_Points[0];
		const Vec3d_t b = m_Points[1];
		const Vec3d_t c = m_Points[2];
		const Vec3d_t ab = b - a;
		const Vec3d_t ac = c - a;
		const Vec3d_t ao = -a;
		const Vec3d_t abc = Cross( ab, ac );

		if ( Dot( Cross( abc, ac ), ao ) > 0.0 )
		{
			if ( Dot( ac, ao ) > 0.0 )
			{
				Set( { a, c } );
				direction = TripleCross( ac, ao, ac );
				return Dot( direction, direction ) == 0.0;
			}

			Set( { a, b } );
			return UpdateLine( direction );
		}

		if ( Dot( Cross( ab, abc ), ao ) > 0.0 )
		{
			Set( { a, b } );
			return UpdateLine( direction );
		}

		const double flSide = Dot( abc, ao );
		if ( flSide == 0.0 )
			return true;

		if ( flSide > 0.0 )
		{
			direction = abc;
		}
		else
		{
			Set( { a, c, b } );
			direction = -abc;
		}
		return false;
	}

	bool UpdateTetrahedron( Vec3d_t &direction )
	{
		const Vec3d_t a = m_Points[0];
		const Vec3d_t b = m_Points[1];
		const Vec3d_t c = m_Points[2];
		const Vec3d_t d = m_Points[3];
		const Vec3d_t ao = -a;

		const Vec3d_t abc = Cross( b - a, c - a );
		const Vec3d_t acd = Cross( c - a, d - a );
		const Vec3d_t adb = Cross( d - a, b - a );

		if ( Dot( abc, ao ) > 0.0 )
		{
			Set( { a, b, c } );
			return UpdateTriangle( direction );
		}

		if ( Dot( acd, ao ) > 0.0 )
		{
			Set( { a, c, d } );
			return UpdateTriangle( direction );
		}

		if ( Dot( adb, ao ) > 0.0 )
		{
			Set( { a, d, b } );
			return UpdateTriangle( direction );
		}

		return true;
	}

	std::array<Vec3d_t, 4> m_Points = {};
	int m_nCount = 0;
};

bool IsBoxIntersectingTruncatedCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone )
{
	const double flTheta = cone.theta * ( std::numbers::pi / 180.0 );
	const double flSinTheta = std::sin( flTheta );
	const double flRadius = cone.h * std::tan( flTheta );

	const Vec3d_t apex = { cone.origin.x, cone.origin.y, cone.origin.z };
	const Vec3d_t axis = { cone.normal.x, cone.normal.y, cone.normal.z };
	const Vec3d_t baseCenter = { apex.x + cone.h * axis.x, apex.y + cone.h * axis.y, apex.z + cone.h * axis.z };

	auto supportBox = [&]( const Vec3d_t &direction ) -> Vec3d_t {
		return {
			direction.x >= 0.0 ? boxAbsMaxs.x : boxAbsMins.x,
			direction.y >= 0.0 ? boxAbsMaxs.y : boxAbsMins.y,
			direction.z >= 0.0 ? boxAbsMaxs.z : boxAbsMins.z,
		};
	};

	auto supportCone = [&]( const Vec3d_t &direction ) -> Vec3d_t {
		const double flLength = std::sqrt( Dot( direction, direction ) );
		if ( flLength == 0.0 )
			return apex;

		const double flAlong = Dot( direction, axis ) / flLength;
		if ( flAlong <= -flSinTheta )
			return apex;

		const Vec3d_t radial = {
			direction.x / flLength - axis.x * flAlong,
			direction.y / flLength - axis.y * flAlong,
			direction.z / flLength - axis.z * flAlong,
		};
		const double flRadialLength = std::sqrt( Dot( radial, radial ) );
		if ( flRadialLength <= 0.0001 )
			return baseCenter;

		const double flScale = flRadius / flRadialLength;
		return { baseCenter.x + radial.x * flScale, baseCenter.y + radial.y * flScale, baseCenter.z + radial.z * flScale };
	};

	auto support = [&]( const Vec3d_t &direction ) { return supportBox( direction ) - supportCone( -direction ); };

	constexpr int MAX_GJK_ITERATIONS = 64;

	CSimplex simplex;
	const Vec3d_t initial = support( { 1.0, 0.0, 0.0 } );
	simplex.Push( initial );
	Vec3d_t direction = -initial;

	for ( int nIteration = 0; nIteration < MAX_GJK_ITERATIONS; ++nIteration )
	{
		if ( Dot( direction, direction ) == 0.0 )
			return true;

		const Vec3d_t point = support( direction );
		if ( Dot( point, direction ) < 0.0 )
			return false;

		simplex.Push( point );
		if ( simplex.Update( direction ) )
			return true;
	}

	return false;
}

void GetCompactSurfaceAABB( const CPhysCollideCompactSurface &collide, const Vector &origin, const QAngle &angles, Vector *pMins, Vector *pMaxs )
{
	static constexpr float CLEARED_BOUND = 99999.0f;
	static const b3Vec3 s_Directions[6] = { { -1.0f, 0.0f, 0.0f }, { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, -1.0f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };

	CTraceCollide cache( collide, origin, angles );
	const std::span<const CPhysCollideCompactSurface::LedgeTreeNode_t> nodes = collide.GetLedgeTree();
	if ( nodes[0].m_nRightChild < 0 )
	{
		cache.SetLedge( nodes[0].m_nConvex );
		pMins->x = cache.Support( s_Directions[0] ).m_Point.x;
		pMins->y = cache.Support( s_Directions[1] ).m_Point.y;
		pMins->z = cache.Support( s_Directions[2] ).m_Point.z;
		pMaxs->x = cache.Support( s_Directions[3] ).m_Point.x;
		pMaxs->y = cache.Support( s_Directions[4] ).m_Point.y;
		pMaxs->z = cache.Support( s_Directions[5] ).m_Point.z;
		return;
	}

	b3Vec3 mins = { CLEARED_BOUND, CLEARED_BOUND, CLEARED_BOUND };
	b3Vec3 maxs = { -CLEARED_BOUND, -CLEARED_BOUND, -CLEARED_BOUND };
	collide.VisitLeafLedges( [&]( int nLedge ) {
		cache.SetLedge( nLedge );
		for ( b3Vec3 direction : s_Directions )
		{
			const b3Vec3 point = cache.Support( direction ).m_Point;
			mins = { std::min( mins.x, point.x ), std::min( mins.y, point.y ), std::min( mins.z, point.z ) };
			maxs = { std::max( maxs.x, point.x ), std::max( maxs.y, point.y ), std::max( maxs.z, point.z ) };
		}
	} );
	*pMins = ToSource( mins );
	*pMaxs = ToSource( maxs );
}

Vector GetCompactSurfaceExtent( const CPhysCollideCompactSurface &collide, const Vector &origin, const QAngle &angles, const Vector &direction )
{
	static constexpr float INITIAL_EXTENT_DISTANCE = -1e6f;

	CTraceCollide cache( collide, origin, angles );
	const b3Vec3 searchDirection = ToBox3D( direction );
	const std::span<const CPhysCollideCompactSurface::LedgeTreeNode_t> nodes = collide.GetLedgeTree();
	if ( nodes[0].m_nRightChild < 0 )
	{
		cache.SetLedge( nodes[0].m_nConvex );
		return ToSource( cache.Support( searchDirection ).m_Point );
	}

	Vector extent( 0.0f, 0.0f, 0.0f );
	float flBestDistance = INITIAL_EXTENT_DISTANCE;
	collide.VisitLeafLedges( [&]( int nLedge ) {
		cache.SetLedge( nLedge );
		const b3Vec3 point = cache.Support( searchDirection ).m_Point;
		const float flDistance = ( point.y * direction.y + point.x * direction.x ) + point.z * direction.z;
		if ( flDistance > flBestDistance )
		{
			flBestDistance = flDistance;
			extent = ToSource( point );
		}
	} );
	return extent;
}
