#include "physics_fluid_controller.h"

#include "compact_builder/compact_surface.h"
#include "compact_builder/precision.h"
#include "conversion.h"
#include "physics_collide.h"
#include "physics_environment.h"
#include "physics_object.h"
#include "physics_surface_props.h"

#include <algorithm>
#include <cmath>
#include <numbers>

static constexpr float SKIN_FRICTION_COEFFICIENT = 0.05f;
static constexpr float TANGENTIAL_TORQUE_COEFFICIENT = 0.01f;
static constexpr float TANGENTIAL_TORQUE_SCALE = 0.1f;
static constexpr float SPHERE_SPIN_DAMPING = 0.01f;
static constexpr float BUOYANCY_EPSILON = 1e-10f;
static constexpr float FLUID_VELOCITY_DRAG = 0.1f;
static constexpr float PARALLEL_TOLERANCE = 1e-9f;
static constexpr real_t PI = std::numbers::pi_v<real_t>;
static constexpr real_t THIRD = real_t( 1 ) / real_t( 3 );
static constexpr real_t TWELFTH = real_t( 1 ) / real_t( 12 );
static constexpr real_t NEGATIVE_SIXTH = real_t( -1 ) / real_t( 6 );

struct Submersion_t
{
	float m_flVolume = 0.0f;
	b3Vec3 m_Center = b3Vec3_zero;
	b3Vec3 m_Force = b3Vec3_zero;
	b3Vec3 m_Torque = b3Vec3_zero;
	b3Vec3 m_TangentTorque = b3Vec3_zero;
	float m_flArea = 0.0f;
};

struct Plane_t
{
	b3Vec3 m_Normal;
	float m_flDistance;
};

struct RealPlane_t
{
	RealVec3_t m_Normal;
	real_t m_flDistance;
};

static float Dot3( b3Vec3 a, b3Vec3 b )
{
	return ( a.x * b.x + a.y * b.y ) + a.z * b.z;
}

static float LengthSquared3( b3Vec3 v )
{
	return Dot3( v, v );
}

static real_t Length3( b3Vec3 v )
{
	return std::sqrt( real_t( LengthSquared3( v ) ) );
}

static real_t ReciprocalSquareRoot( real_t x )
{
#if defined( VPHYSICS_GMOD )
	return ApproximateInverseSquareRoot( x, 5 );
#else
	return 1.0f / std::sqrt( x );
#endif
}

static b3Vec3 Combine( b3Vec3 u, real_t flWeightU, b3Vec3 v, real_t flWeightV, real_t flInverse )
{
	const auto combine = [&]( float a, float b ) {
		const float flSum = static_cast<float>( real_t( static_cast<float>( real_t( a ) * flWeightU ) ) + real_t( b ) * flWeightV );
		return static_cast<float>( real_t( flSum ) * flInverse );
	};
	return { combine( u.x, v.x ), combine( u.y, v.y ), combine( u.z, v.z ) };
}

static bool Normalize( b3Vec3 &v )
{
	const float flLengthSquared = LengthSquared3( v );
	if ( real_t( flLengthSquared ) < REAL_RESOLUTION )
		return false;

	v = Scale( v, ReciprocalSquareRoot( flLengthSquared ) );
	return true;
}

static real_t NormalizeLength( b3Vec3 &v )
{
	const float flLengthSquared = LengthSquared3( v );
	if ( real_t( flLengthSquared ) < REAL_RESOLUTION )
		return 0;

	const real_t flInverse = ReciprocalSquareRoot( flLengthSquared );
	v = Scale( v, flInverse );
#if defined( VPHYSICS_GMOD )
	return flInverse * real_t( flLengthSquared );
#else
	return std::sqrt( flLengthSquared );
#endif
}

static void NormalizePlane( Plane_t &plane )
{
	const real_t flInverse = real_t( 1 ) / std::sqrt( real_t( LengthSquared3( plane.m_Normal ) ) );
	plane.m_Normal = Scale( plane.m_Normal, flInverse );
	plane.m_flDistance = static_cast<float>( real_t( plane.m_flDistance ) * flInverse );
}

static b3Vec3 ProjectOntoPlane( const Plane_t &plane, b3Vec3 point )
{
	const real_t t = -( ( real_t( Dot3( plane.m_Normal, point ) ) + real_t( plane.m_flDistance ) ) / real_t( LengthSquared3( plane.m_Normal ) ) );
	return ScaleAdd( plane.m_Normal, t, point );
}

static RealVec3_t ProjectOntoPlane( const RealPlane_t &plane, RealVec3_t point )
{
	const real_t t = -( ( Dot( plane.m_Normal, point ) + plane.m_flDistance ) / LengthSquared( plane.m_Normal ) );
	return t * plane.m_Normal + point;
}

static void Normalize( RealVec3_t &v )
{
	const real_t flLengthSquared = LengthSquared( v );
	if ( flLengthSquared >= REAL_RESOLUTION )
		v = ReciprocalSquareRoot( flLengthSquared ) * v;
}

static bool IsParallel( RealVec3_t a, RealVec3_t b )
{
	const RealVec3_t c = Cross( a, b );
	const real_t flTolerance = PARALLEL_TOLERANCE;
	return ( LengthSquared( b ) * flTolerance ) * ( LengthSquared( a ) * flTolerance ) >= ( c.x * c.x + c.y * c.y ) + c.z * c.z;
}

static bool IntersectRayPlane( const RealPlane_t &plane, RealVec3_t direction, RealVec3_t origin, RealVec3_t &out )
{
	const RealVec3_t &n = plane.m_Normal;
	const real_t flStart = Dot( n, origin ) + plane.m_flDistance;
	const real_t flEnd = ( ( n.x * ( direction.x + origin.x ) + n.y * ( direction.y + origin.y ) ) + n.z * ( direction.z + origin.z ) ) + plane.m_flDistance;
	const real_t flDenominator = flEnd - flStart;
	if ( std::fabs( flDenominator ) < REAL_RESOLUTION )
	{
		if ( !( flStart < REAL_RESOLUTION ) )
			return false;

		out = origin;
		return true;
	}

	out = -( flStart / flDenominator ) * direction + origin;
	return true;
}

static bool IntersectPlanes( const Plane_t &first, const Plane_t &second, b3Vec3 &direction, b3Vec3 &point )
{
	const RealPlane_t p = { ToReal( first.m_Normal ), first.m_flDistance };
	const RealPlane_t w = { ToReal( second.m_Normal ), second.m_flDistance };
	const RealVec3_t p0 = ProjectOntoPlane( p, { 0, 0, 0 } );
	if ( IsParallel( p.m_Normal, w.m_Normal ) )
		return false;

	RealVec3_t lineDirection = Cross( p.m_Normal, w.m_Normal );
	Normalize( lineDirection );

	RealVec3_t toLine = ProjectOntoPlane( p, ProjectOntoPlane( w, p0 ) ) - p0;
	Normalize( toLine );

	RealVec3_t linePoint;
	if ( !IntersectRayPlane( w, toLine, p0, linePoint ) )
		return false;

	direction = ToFloat( lineDirection );
	point = ToFloat( linePoint );
	return true;
}

static bool FindQuadraticRoots( real_t a, real_t b, real_t c, real_t &t1, real_t &t2 )
{
	if ( std::fabs( a ) < REAL_RESOLUTION )
	{
		if ( std::fabs( b ) < REAL_RESOLUTION )
			return false;

		t1 = t2 = -( c / b );
		return true;
	}

	const real_t flDiscriminant = b * b - ( a * c ) * 4;
	if ( !( flDiscriminant >= 0 ) )
		return false;

	const real_t q = real_t( 0.5 ) / a;
	const real_t s = std::sqrt( flDiscriminant );
	t1 = ( -b - s ) * q;
	t2 = ( s - b ) * q;
	return true;
}

static bool IntersectLineSphere( const Plane_t &first, const Plane_t &second, b3Vec3 center, float flRadius, b3Vec3 &hit1, b3Vec3 &hit2 )
{
	b3Vec3 direction, point;
	if ( !IntersectPlanes( first, second, direction, point ) )
		return false;

	const b3Vec3 offset = b3Sub( point, center );
	const float a = LengthSquared3( direction );
	const float b = ( ( offset.y + offset.y ) * direction.y + ( offset.x + offset.x ) * direction.x ) + ( offset.z + offset.z ) * direction.z;
	const float c = LengthSquared3( offset ) - flRadius * flRadius;
	real_t t1, t2;
	if ( !FindQuadraticRoots( a, b, c, t1, t2 ) )
	{
		hit1 = hit2 = b3Vec3_zero;
		return false;
	}

	hit1 = ScaleAdd( direction, t1, point );
	hit2 = ScaleAdd( direction, t2, point );
	return true;
}

static real_t GetSilhouetteSegment( float flRadius, real_t flChord, real_t flMidDistance )
{
#if defined( VPHYSICS_GMOD )
	const float x = static_cast<float>( flChord / real_t( flRadius + flRadius ) );
#else
	const float x = ( 0.5f / flRadius ) * flChord;
#endif
	const float x2 = x * x;
	const float x3 = x2 * x;
	const float x5 = x2 * x3;
	const float flPolynomial = ( x5 * 0.58f + x ) + ( x3 * 0.24f + x );
#if defined( VPHYSICS_GMOD )
	return std::fabs( ( real_t( flPolynomial * flRadius ) * real_t( flRadius ) - ( real_t( flRadius ) - ( real_t( flRadius ) - flMidDistance ) ) * flChord ) * 0.5 );
#else
	return std::fabs( ( ( flPolynomial * flRadius ) * flRadius - flMidDistance * flChord ) * 0.5f );
#endif
}

enum SegmentForm_t
{
	SEGMENT_UNCLAMPED,
	SEGMENT_ABSOLUTE,
	SEGMENT_REST,
};

struct SmallSegment_t
{
	real_t m_flArea;
	real_t m_flOffset;
};

static SmallSegment_t GetSmallSegment( real_t flRadius, real_t flSagitta, real_t flDistance, SegmentForm_t nForm )
{
	real_t flChord = std::sqrt( std::fabs( ( flSagitta + flSagitta ) * flRadius - flSagitta * flSagitta ) );
	flChord = flChord + flChord;
	const float y = static_cast<float>( ( real_t( 0.5 ) / flRadius ) * flChord );
	const float y2 = y * y;
	const float y3 = y2 * y;
	const float y5 = y3 * y2;
	const float flPolynomial = ( ( y + y3 * 0.12f ) + y3 * 0.12f ) + ( ( y5 * 0.29f + y5 * 0.29f ) + y );

	real_t flArea;
	if ( nForm == SEGMENT_REST )
		flArea = std::fabs( ( std::fabs( real_t( flPolynomial ) * flRadius ) * flRadius - flDistance * flChord ) * real_t( 0.5 ) );
	else
		flArea = ( ( real_t( flPolynomial ) * flRadius ) * flRadius - ( flRadius - flSagitta ) * flChord ) * real_t( 0.5 );
	if ( nForm == SEGMENT_ABSOLUTE )
		flArea = std::fabs( flArea );

	return { flArea, ( ( TWELFTH / flArea ) * flChord ) * ( flChord * flChord ) };
}

static real_t GetEllipseSegment( real_t flWaterRadius, real_t flAxis, real_t flDistance, real_t flChord, bool bChordFirst )
{
	const float flRatio = static_cast<float>( flDistance / flAxis );
	const real_t flAngle = flRatio < 1.0f ? real_t( static_cast<float>( std::acos( real_t( flRatio ) ) ) ) : real_t( 0 );
	const real_t flChordTerm = bChordFirst ? ( flChord * real_t( 0.5 ) ) * flDistance : ( flDistance * flChord ) * real_t( 0.5 );
	return ( flWaterRadius * flAxis ) * flAngle - flChordTerm;
}

static void ComputeSphereSubmersion( float flRadius, const Plane_t &water, b3Vec3 velocity, float flPressureDrag, float flSkinDrag, Submersion_t &out )
{
	const b3Vec3 center = b3Vec3_zero;
	const real_t eps = BUOYANCY_EPSILON;

	Plane_t flow;
	flow.m_Normal = real_t( LengthSquared3( velocity ) ) < REAL_RESOLUTION ? b3Vec3{ 0.0f, 1.0f, 0.0f } : velocity;
	flow.m_flDistance = -Dot3( center, flow.m_Normal );
	NormalizePlane( flow );
	const b3Vec3 f = flow.m_Normal;
	const b3Vec3 n = water.m_Normal;

	const float h0 = Dot3( center, n ) + water.m_flDistance;
	b3Vec3 chordStart = b3Vec3_zero, chordEnd = b3Vec3_zero;
	int nCase;
	if ( h0 > 0.0f )
	{
		if ( h0 >= flRadius )
			nCase = 0;
		else
			nCase = IntersectLineSphere( flow, water, center, flRadius, chordStart, chordEnd ) ? 3 : 4;
	}
	else
	{
		if ( std::fabs( h0 ) >= flRadius )
			return;

		nCase = IntersectLineSphere( flow, water, center, flRadius, chordStart, chordEnd ) ? 1 : 2;
	}

	if ( nCase == 0 )
	{
#if defined( VPHYSICS_GMOD )
		out.m_flVolume = static_cast<float>( real_t( ( flRadius * flRadius ) * flRadius ) * ( real_t( 4 ) * PI / real_t( 3 ) ) );
#else
		out.m_flVolume = ( flRadius * flRadius ) * ( flRadius * static_cast<float>( 4.0 * std::numbers::pi / 3.0 ) );
#endif
		out.m_Center = center;
	}
	else
	{
		const float h = h0 + flRadius;
		const float t3 = flRadius * 3.0f - h;
		out.m_flVolume = static_cast<float>( ( real_t( t3 ) * real_t( h ) ) * ( real_t( h ) * ( PI / real_t( 3 ) ) ) );
		const float s = ( ( ( ( flRadius * 4.0f ) * flRadius - ( flRadius * 4.0f ) * h ) + h * h ) * 0.75f ) / t3;
		out.m_Center = ScaleAdd( n, s, center );
	}

	const b3Vec3 waterCenter = ProjectOntoPlane( water, center );
	const b3Vec3 flowCenter = ProjectOntoPlane( flow, waterCenter );
	const float flFacing = Dot3( f, n );
	const b3Vec3 projectedNormal = ProjectOntoPlane( flow, n );
	const real_t flWaterRadius = std::sqrt( real_t( std::fabs( flRadius * flRadius - h0 * h0 ) ) );
	const real_t flCircle = ( real_t( flRadius ) * PI ) * real_t( flRadius );
	const real_t flWetDisc = ( real_t( flRadius * flRadius - h0 * h0 ) * real_t( flFacing ) ) * PI;

	real_t a = 0;
	b3Vec3 g = b3Vec3_zero;
	const auto applyCenterWet = [&]() {
		if ( !( real_t( flFacing ) > 0 ) )
		{
			a = flCircle;
			g = center;
			return;
		}

		a = std::fabs( flCircle - flWetDisc );
		if ( a < eps )
		{
			a = 0;
			g = b3Vec3_zero;
			return;
		}

		g = Combine( flowCenter, -flWetDisc, center, flCircle, real_t( 1 ) / a );
	};
	const auto applyCenterDry = [&]() {
		if ( real_t( flFacing ) >= 0 )
		{
			a = 0;
			g = b3Vec3_zero;
			return;
		}

		g = flowCenter;
		a = std::fabs( flWetDisc );
	};

	b3Vec3 chordDirection = nCase == 3 ? b3Neg( projectedNormal ) : projectedNormal;
	const b3Vec3 chord = b3Sub( chordEnd, chordStart );
	const real_t flChord = Length3( chord );
	if ( nCase == 0 )
	{
		a = flCircle;
		g = center;
	}
	else if ( nCase == 4 )
	{
		applyCenterWet();
	}
	else if ( nCase == 2 )
	{
		applyCenterDry();
	}
	else if ( real_t( LengthSquared3( chordDirection ) ) < eps )
	{
		if ( nCase == 1 )
			applyCenterDry();
		else
			applyCenterWet();
	}
	else if ( flChord < eps )
	{
		a = nCase == 1 ? 0 : flCircle;
		g = nCase == 1 ? b3Vec3_zero : center;
	}
	else
	{
		Normalize( chordDirection );

		const b3Vec3 mid = b3MulAdd( chordStart, 0.5f, chord );
		Plane_t bisector = { chord, -Dot3( mid, chord ) };
		NormalizePlane( bisector );
		b3Vec3 hit1 = b3Vec3_zero, hit2 = b3Vec3_zero;
		IntersectLineSphere( bisector, water, center, flRadius, hit1, hit2 );
		const b3Vec3 upstream = ( ( hit2.y - hit1.y ) * f.y + ( hit2.x - hit1.x ) * f.x ) + ( hit2.z - hit1.z ) * f.z >= 0.0f ? hit1 : hit2;
		const b3Vec3 q = ProjectOntoPlane( flow, upstream );

		const real_t flAxis = Length3( b3Sub( flowCenter, q ) );
		const real_t flMidOffset = Length3( b3Sub( flowCenter, mid ) );
		const real_t flMidDistance = Length3( b3Sub( mid, center ) );
		const real_t flEllipse = ( flWaterRadius * PI ) * flAxis;
		const real_t flCube = ( flChord * flChord ) * flChord;

		if ( nCase == 1 && real_t( flFacing ) > eps )
		{
			const real_t s = GetSilhouetteSegment( flRadius, flChord, flMidDistance );
			const real_t e = flAxis >= eps ? GetEllipseSegment( flWaterRadius, flAxis, flMidOffset, flChord, false ) : real_t( 0 );
			a = s - e;
			if ( s < eps || a < eps )
			{
				a = 0;
				g = b3Vec3_zero;
			}
			else
			{
				const b3Vec3 silhouetteCenter = ScaleAdd( chordDirection, ( ( TWELFTH / s ) * flChord ) * ( flChord * flChord ), center );
				b3Vec3 ellipseCenter = b3Vec3_zero;
				if ( flAxis >= eps )
				{
					const SmallSegment_t segment = GetSmallSegment( flAxis, std::fabs( flAxis - flMidOffset ), flMidOffset, SEGMENT_UNCLAMPED );
					if ( segment.m_flArea >= eps )
						ellipseCenter = ScaleAdd( chordDirection, segment.m_flOffset, flowCenter );
				}
				g = Combine( ellipseCenter, -e, silhouetteCenter, s, real_t( 1 ) / a );
			}
		}
		else if ( nCase == 1 )
		{
			const real_t s = GetSilhouetteSegment( flRadius, flChord, flMidDistance );
			b3Vec3 silhouetteCenter = b3Vec3_zero;
			bool bDone = false;
			if ( s >= eps )
			{
				const real_t k = ( TWELFTH / s ) * flCube;
				if ( real_t( flFacing ) > -eps )
				{
					g = ScaleAdd( n, k, center );
					a = s;
					bDone = true;
				}
				else
				{
					silhouetteCenter = ScaleAdd( chordDirection, k, center );
				}
			}

			if ( !bDone )
			{
				real_t flRest = 0;
				b3Vec3 restCenter = b3Vec3_zero;
				if ( flAxis >= eps )
				{
					const real_t e = std::fabs( GetEllipseSegment( flWaterRadius, flAxis, flMidOffset, flChord, true ) );
					flRest = std::fabs( flEllipse - e );
					const SmallSegment_t segment = GetSmallSegment( flAxis, flAxis - flMidOffset, flMidOffset, SEGMENT_REST );
					real_t flSegmentWeight = e;
					b3Vec3 segmentCenter = b3Vec3_zero;
					if ( segment.m_flArea < eps )
					{
						flSegmentWeight = 0;
					}
					else
					{
						b3Vec3 toAxis = b3Sub( flowCenter, q );
						Normalize( toAxis );
						segmentCenter = ScaleAdd( toAxis, segment.m_flOffset, flowCenter );
					}

					if ( flRest < eps )
						flRest = 0;
					else
						restCenter = Combine( segmentCenter, -flSegmentWeight, flowCenter, flEllipse, real_t( 1 ) / flRest );
				}

				a = flRest + s;
				if ( a < eps )
				{
					a = 0;
					g = b3Vec3_zero;
				}
				else
				{
					g = Combine( silhouetteCenter, s, restCenter, flRest, real_t( 1 ) / a );
				}
			}
		}
		else if ( real_t( flFacing ) <= eps )
		{
			const real_t s = GetSilhouetteSegment( flRadius, flChord, flMidDistance );
			const real_t flRemainder = std::fabs( flCircle - s );
			b3Vec3 remainderCenter = chord;
			bool bDone = false;
			if ( s >= eps )
			{
				if ( real_t( flFacing ) > -eps )
				{
					g = Combine( ScaleAdd( n, ( -TWELFTH / s ) * flCube, center ), -s, center, flCircle, real_t( 1 ) / flRemainder );
					a = flRemainder;
					bDone = true;
				}
				else
				{
					remainderCenter = Combine( ScaleAdd( chordDirection, ( TWELFTH / s ) * flCube, center ), -s, center, flCircle, real_t( 1 ) / flRemainder );
				}
			}

			if ( !bDone )
			{
				real_t e = 0;
				b3Vec3 ellipseCenter = b3Vec3_zero;
				if ( flAxis >= eps )
				{
					e = GetEllipseSegment( flWaterRadius, flAxis, flMidOffset, flChord, false );
					const SmallSegment_t segment = GetSmallSegment( flAxis, std::fabs( flAxis - flMidOffset ), flMidOffset, SEGMENT_ABSOLUTE );
					if ( segment.m_flArea < eps )
						e = 0;
					else
						ellipseCenter = ScaleAdd( chordDirection, segment.m_flOffset, flowCenter );
				}

				a = e + flRemainder;
				if ( a < eps )
				{
					a = 0;
					g = b3Vec3_zero;
				}
				else
				{
					g = Combine( remainderCenter, flRemainder, ellipseCenter, e, real_t( 1 ) / a );
				}
			}
		}
		else
		{
			const real_t s = GetSilhouetteSegment( flRadius, flChord, flMidDistance );
			if ( s < eps )
			{
				applyCenterWet();
			}
			else
			{
				const real_t e = flAxis >= eps ? GetEllipseSegment( flWaterRadius, flAxis, flMidOffset, flChord, false ) : real_t( 0 );
				a = std::fabs( ( real_t( flRadius * flRadius ) - flWaterRadius * flAxis ) * PI - std::fabs( s - e ) );
				if ( a < eps )
				{
					a = 0;
					g = b3Vec3_zero;
				}
				else
				{
					real_t flRemainder = std::fabs( flCircle - s );
					b3Vec3 remainderCenter = b3Vec3_zero;
					if ( flRemainder < eps )
						flRemainder = 0;
					else
						remainderCenter = Combine( ScaleAdd( chordDirection, ( ( TWELFTH / s ) * flChord ) * ( flChord * flChord ), center ), -s, center, flCircle, real_t( 1 ) / flRemainder );

					b3Vec3 ellipseCenter = b3Vec3_zero;
					if ( flAxis >= eps )
					{
						const SmallSegment_t segment = GetSmallSegment( flAxis, std::fabs( flAxis - flMidOffset ), flMidOffset, SEGMENT_UNCLAMPED );
						if ( segment.m_flArea >= eps )
							ellipseCenter = ScaleAdd( chordDirection, segment.m_flOffset, flowCenter );
					}

					real_t flEllipseRest = std::fabs( flEllipse - e );
					b3Vec3 ellipseRestCenter = b3Vec3_zero;
					if ( flEllipseRest < eps )
						flEllipseRest = 0;
					else
						ellipseRestCenter = Combine( ellipseCenter, -e, flowCenter, flEllipse, real_t( 1 ) / flEllipseRest );

					g = Combine( ellipseRestCenter, -flEllipseRest, remainderCenter, flRemainder, real_t( 1 ) / a );
				}
			}
		}
	}

	real_t flSpeedSquared = LengthSquared3( velocity );
	if ( flSpeedSquared < eps )
	{
		velocity = { 0.0f, 1.0f, 0.0f };
		flSpeedSquared = 0;
	}

	const real_t flMagnitude = ( ( a * 3 ) * flSpeedSquared ) * real_t( flSkinDrag ) + ( flSpeedSquared * a ) * real_t( flPressureDrag );
	Normalize( velocity );
	out.m_flArea = static_cast<float>( real_t( out.m_flArea ) + a );
	const b3Vec3 force = Scale( velocity, flMagnitude );
	out.m_Torque = b3Add( b3Cross( g, force ), out.m_Torque );
	out.m_Force = b3Add( force, out.m_Force );
}

static b3Vec3 GetWeightedPoint( const b3Vec3 points[3], int k0, real_t w0, real_t w1, real_t w2 )
{
	const int k1 = ( k0 + 1 ) % 3, k2 = ( k0 + 2 ) % 3;
	const auto cut = [&]( float p0, float p1, float p2 ) {
		const float flFirst = static_cast<float>( w0 * real_t( p0 ) );
		const float flSecond = static_cast<float>( real_t( flFirst ) + w1 * real_t( p1 ) );
		return static_cast<float>( real_t( flSecond ) + w2 * real_t( p2 ) );
	};
	return {
		cut( points[k0].x, points[k1].x, points[k2].x ),
		cut( points[k0].y, points[k1].y, points[k2].y ),
		cut( points[k0].z, points[k1].z, points[k2].z ),
	};
}

static void ComputePolygonSubmersion( const CPhysCollideCompactSurface &surface, const std::vector<int> &ledges, const CPhysicsObject *pObject, const RealMatrix3_t &rotation, RealVec3_t origin, const Plane_t &water, float flPressureDrag, float flSkinDrag, b3Vec3 fluidVelocity, Submersion_t &out )
{
	const float eps = BUOYANCY_EPSILON;
	const b3Vec3 n = water.m_Normal;
	const b3Vec3 surfacePoint = Scale( n, -real_t( water.m_flDistance ) );
	const RealMatrix3_t &coreRotation = pObject->GetCoreRotation();
	const RealVec3_t coreOrigin = pObject->GetCorePosition();
	const b3Vec3 coreAngular = pObject->GetCoreAngularVelocity();
	const b3Vec3 coreVelocity = pObject->GetCoreVelocity();

	float flVolume = 0.0f;
	b3Vec3 volumeCenter = b3Vec3_zero;
	for ( auto ledge = ledges.rbegin(); ledge != ledges.rend(); ++ledge )
	{
		const unsigned char *pLedge = surface.GetLedge( *ledge );
		const int nTriangles = GetLedgeTriangleCount( pLedge );
		for ( int t = 0; t < nTriangles; ++t )
		{
			const unsigned char *pTriangle = GetLedgeTriangle( pLedge, t );
			b3Vec3 points[3];
			float distances[3];
			int nWet = -1, nDry = -1, nDryCount = 0;
			for ( int i = 2; i >= 0; --i )
			{
				points[i] = LoadPoint( GetLedgePoint( pLedge, GetEdgeStartPoint( pTriangle, i ) ) );
				const float d = Dot3( n, points[i] ) + water.m_flDistance;
				if ( d >= 0.0f )
				{
					distances[i] = eps + d;
					nWet = i;
				}
				else
				{
					distances[i] = d - eps;
					nDry = i;
					++nDryCount;
				}
			}

			if ( nDryCount == 3 )
				continue;

			const b3Vec3 sum = b3Add( b3Add( points[0], points[1] ), points[2] );
			const RealVec3_t e1 = ToReal( points[1] ) - ToReal( points[0] );
			const RealVec3_t e2 = ToReal( points[2] ) - ToReal( points[0] );
			const RealVec3_t lever = ToReal( b3Sub( points[0], surfacePoint ) );
			const RealVec3_t c = Cross( e2, e1 );
			const float flTetraVolume = static_cast<float>( ( ( c.y * lever.y + c.x * lever.x ) + c.z * lever.z ) * NEGATIVE_SIXTH );

			real_t flFraction = 1;
			b3Vec3 wetCenter;
			if ( nDryCount == 0 )
			{
				flVolume += flTetraVolume;
				volumeCenter = b3Add( b3MulSV( flTetraVolume, b3Add( sum, surfacePoint ) ), volumeCenter );
				wetCenter = Scale( sum, THIRD );
			}
			else
			{
				const int k0 = nDryCount == 1 ? nDry : nWet;
				const float tau1 = distances[k0] / ( distances[k0] - distances[( k0 + 1 ) % 3] );
				const float tau2 = distances[k0] / ( distances[k0] - distances[( k0 + 2 ) % 3] );
				const real_t w0 = ( real_t( 3 ) - tau1 ) - tau2;
				const real_t p = real_t( tau2 ) * real_t( tau1 );
				const b3Vec3 cut = GetWeightedPoint( points, k0, w0, tau1, tau2 );
				const b3Vec3 thirdWeighted = GetWeightedPoint( points, k0, w0 * THIRD, tau1 * THIRD, tau2 * THIRD );
				if ( nDryCount == 1 )
				{
					const real_t flPartVolume = real_t( flTetraVolume ) - ( real_t( flTetraVolume ) * tau1 ) * tau2;
					flVolume = static_cast<float>( real_t( flVolume ) + flPartVolume );
					flFraction = real_t( 1 ) - p;
					if ( flFraction < real_t( eps ) )
					{
						flVolume = 0.0f;
						volumeCenter = b3Vec3_zero;
						continue;
					}

					const b3Vec3 partCenter = Scale( b3Sub( sum, Scale( cut, p ) ), real_t( 1 ) / flFraction );
					volumeCenter = b3Add( Scale( b3Add( partCenter, surfacePoint ), flPartVolume ), volumeCenter );
					wetCenter = Scale( b3Sub( Scale( sum, THIRD ), Scale( thirdWeighted, p ) ), real_t( 1 ) / flFraction );
				}
				else
				{
					const real_t flPartVolume = real_t( flTetraVolume ) * p;
					flVolume = static_cast<float>( real_t( flVolume ) + flPartVolume );
					volumeCenter = b3Add( Scale( b3Add( cut, surfacePoint ), flPartVolume ), volumeCenter );
					flFraction = static_cast<float>( p );
					wetCenter = thirdWeighted;
				}
			}

			b3Vec3 faceNormal = ToFloat( Cross( e1, e2 ) );
			if ( LengthSquared3( faceNormal ) < eps )
				continue;

			const RealVec3_t worldPoint = Rotate( rotation, wetCenter ) + origin;
			const b3Vec3 corePoint = ToFloat( InverseRotate( coreRotation, worldPoint - coreOrigin ) );
			const b3Vec3 worldVelocity = b3Add( ToFloat( Rotate( coreRotation, b3Cross( coreAngular, corePoint ) ) ), coreVelocity );
			b3Vec3 flowVelocity = b3Sub( fluidVelocity, ToFloat( InverseRotate( rotation, worldVelocity ) ) );
			float flSpeedSquared = LengthSquared3( flowVelocity );
			if ( flSpeedSquared < eps )
			{
				flowVelocity = { 0.0f, 1.0f, 0.0f };
				flSpeedSquared = 0.0f;
			}

			if ( Dot3( flowVelocity, faceNormal ) > eps )
				continue;

			Normalize( flowVelocity );
			const float flArea = static_cast<float>( NormalizeLength( faceNormal ) * real_t( 0.5 ) );
			const float flFacing = Dot3( faceNormal, flowVelocity );
			const float flProjectedArea = -( ( flFacing * static_cast<float>( flFraction ) ) * flArea );
			const real_t flMagnitude = real_t( ( flProjectedArea * flSpeedSquared ) * flPressureDrag ) + real_t( ( flArea * flSpeedSquared ) * flSkinDrag );
			const b3Vec3 force = Scale( flowVelocity, flMagnitude );
			out.m_flArea = std::fabs( flProjectedArea ) + out.m_flArea;
			out.m_Torque = b3Add( b3Cross( wetCenter, force ), out.m_Torque );
			out.m_TangentTorque = b3Add( b3Cross( ScaleAdd( faceNormal, -flFacing, flowVelocity ), force ), out.m_TangentTorque );
			out.m_Force = b3Add( force, out.m_Force );
		}
	}

	out.m_flVolume = flVolume;
	out.m_Center = flVolume > eps ? Scale( volumeCenter, 0.25f / flVolume ) : volumeCenter;
}

CBuoyancyController::CBuoyancyController( CPhysicsFluidController *pFluid, CPhysicsObject *pObject )
: m_pFluid( pFluid )
, m_pObject( pObject )
{
	if ( const CPhysCollideCompactSurface *pSurface = pObject->GetCollide() ? pObject->GetCollide()->GetCompactSurface() : nullptr )
		pSurface->VisitLeafLedges( [this]( int nLedge ) { m_Ledges.push_back( nLedge ); } );
	pObject->AttachController( this );
}

CBuoyancyController::~CBuoyancyController()
{
	m_pObject->DetachController( this );
}

void CBuoyancyController::Simulate( CPhysicsObject *pObject, float flTimestep )
{
	const bool bSimulate = !pObject->GetShadowController() && ( pObject->GetCallbackFlags() & CALLBACK_DO_FLUID_SIMULATION );
	const float flDensity = bSimulate ? pObject->GetBuoyancyRatio() * m_pFluid->GetDensity() : 0.0f;
	if ( flDensity == 0.0f || std::isnan( flDensity ) )
		return;

	b3Vec3 surfaceNormal, current;
	float flSurfaceOffset;
	m_pFluid->GetSurface( surfaceNormal, flSurfaceOffset, current );

	if ( m_flArea > m_flPreviousArea )
		m_FluidVelocity = b3MulSV( m_flPreviousArea / m_flArea, m_FluidVelocity );
	m_flPreviousArea = m_flArea;
	m_flArea = 0.0f;

	const b3Vec3 speed = pObject->GetCoreVelocity();
	const b3Vec3 fluidVelocity = b3Add( current, m_FluidVelocity );
	const b3Vec3 relativeVelocity = b3Sub( fluidVelocity, speed );

	RealMatrix3_t rotation;
	RealVec3_t origin;
	pObject->GetPose( pObject->GetEnvironment()->GetCurrentTime(), rotation, origin );
	const RealVec3_t n = ToReal( surfaceNormal );
#if defined( VPHYSICS_GMOD )
	const float flLocalOffset = static_cast<float>( ( n.x * origin.x + n.y * origin.y ) + ( n.z * origin.z + real_t( flSurfaceOffset ) ) );
#else
	const float flLocalOffset = ( ( n.x * origin.x + n.y * origin.y ) + n.z * origin.z ) + flSurfaceOffset;
#endif
	const Plane_t water = { ToFloat( InverseRotate( rotation, surfaceNormal ) ), flLocalOffset };
	const float flPressureDrag = ( m_pFluid->GetDamping() * 0.5f ) * flDensity;
	const float flSkinDrag = ( SKIN_FRICTION_COEFFICIENT * 0.5f ) * flDensity;

	Submersion_t submersion;
	const CPhysCollideCompactSurface *pSurface = pObject->GetCollide() ? pObject->GetCollide()->GetCompactSurface() : nullptr;
	if ( flDensity > 0.0f )
	{
		const b3Vec3 localFluidVelocity = ToFloat( InverseRotate( rotation, fluidVelocity ) );
		if ( pSurface )
			ComputePolygonSubmersion( *pSurface, m_Ledges, pObject, rotation, origin, water, flPressureDrag, flSkinDrag, localFluidVelocity, submersion );
		else if ( !pObject->GetCollide() )
			ComputeSphereSubmersion( pObject->GetIVPSphereRadius(), water, ToFloat( InverseRotate( rotation, relativeVelocity ) ), flPressureDrag, flSkinDrag, submersion );

		if ( !( submersion.m_flVolume > BUOYANCY_EPSILON ) && !( LengthSquared3( submersion.m_Force ) > BUOYANCY_EPSILON ) )
			submersion = {};
	}

	if ( submersion.m_flVolume > BUOYANCY_EPSILON )
	{
		const RealVec3_t gravity = pObject->GetEnvironment()->GetIVPGravity();
		const real_t flScale = real_t( -( submersion.m_flVolume * flDensity ) ) * real_t( flTimestep );
		const b3Vec3 impulse = Scale( { static_cast<float>( gravity.x ), static_cast<float>( gravity.y ), static_cast<float>( gravity.z ) }, flScale );
		pObject->ApplyImpulse( Rotate( rotation, submersion.m_Center ) + origin, impulse );
	}

	const float flArea = std::fabs( submersion.m_flArea );
#if defined( VPHYSICS_GMOD )
	const float flTangentScale = static_cast<float>( ( std::sqrt( real_t( flArea ) ) * real_t( -TANGENTIAL_TORQUE_COEFFICIENT ) ) * real_t( TANGENTIAL_TORQUE_SCALE ) );
	const b3Vec3 tangentTorque = Scale( submersion.m_TangentTorque, real_t( flTangentScale ) * real_t( flTimestep ) );
#else
	const float flTangentScale = ( ( std::sqrt( flArea ) * TANGENTIAL_TORQUE_COEFFICIENT ) * -TANGENTIAL_TORQUE_SCALE ) * flTimestep;
	const b3Vec3 tangentTorque = b3MulSV( flTangentScale, submersion.m_TangentTorque );
#endif
	const b3Vec3 torque = b3MulSV( flTimestep, submersion.m_Torque );
	pObject->ApplyImpulse( origin, ToFloat( Rotate( rotation, b3MulSV( flTimestep, submersion.m_Force ) ) ) );

	const b3Vec3 inverseInertia = pObject->GetCoreInverseInertia();
#if defined( VPHYSICS_GMOD )
	const b3Vec3 angularChange = b3Add( b3Mul( tangentTorque, inverseInertia ), b3Mul( torque, inverseInertia ) );
#else
	const b3Vec3 angularChange = {
		( tangentTorque.x + torque.x ) * inverseInertia.x,
		torque.y * inverseInertia.y + tangentTorque.y * inverseInertia.y,
		torque.z * inverseInertia.z + tangentTorque.z * inverseInertia.z,
	};
#endif
	pObject->AddPendingAngularVelocity( angularChange );

	if ( !pObject->GetCollide() )
	{
		const float flRadius = pObject->GetIVPSphereRadius();
		const real_t flSpin = real_t( flArea * SPHERE_SPIN_DAMPING ) / ( ( real_t( flRadius ) * ( PI + PI ) ) * real_t( flRadius ) );
		pObject->AddPendingAngularVelocity( b3Neg( Scale( pObject->GetCoreAngularVelocity(), flSpin ) ) );
	}

	m_flArea = static_cast<float>( real_t( flArea ) + real_t( m_flArea ) );

	m_FluidVelocity = ScaleAdd( b3Sub( speed, fluidVelocity ), FLUID_VELOCITY_DRAG * flTimestep, m_FluidVelocity );
}

void CBuoyancyController::ObjectDestroyed( CPhysicsObject *pObject )
{
	m_pFluid->ObjectLeft( pObject );
}

CPhysicsFluidController::CPhysicsFluidController( CPhysicsObject *pFluidObject, const fluidparams_t &params )
: m_pFluidObject( pFluidObject )
, m_pGameData( params.pGameData )
, m_flDamping( params.damping )
, m_nContents( params.contents )
{
	matrix3x4_t matrix;
	pFluidObject->GetPositionMatrix( &matrix );
	const Vector origin( matrix[0][3], matrix[1][3], matrix[2][3] );
	const Vector normal( params.surfacePlane.x, params.surfacePlane.y, params.surfacePlane.z );
	m_LocalNormal = MatrixInverseRotateVector( matrix, normal );
	m_LocalCurrent = MatrixInverseRotateVector( matrix, params.currentVelocity );
	m_flLocalDistance = m_LocalNormal.LengthSqr() * params.surfacePlane.w - DotProduct( m_LocalNormal, MatrixInverseRotateVector( matrix, origin ) );
	m_flDensity = g_PhysicsSurfaceProps.GetSurfaceData( pFluidObject->GetMaterialIndex() )->physics.density;
}

CPhysicsFluidController::~CPhysicsFluidController()
{
	RemoveAllObjects();
	if ( m_pFluidObject )
		m_pFluidObject->SetFluidController( nullptr );
}

static float GetWorldDistance( const matrix3x4_t &matrix, const Vector &normal, float flLocalDistance )
{
	return normal.LengthSqr() * flLocalDistance + DotProduct( normal, Vector( matrix[0][3], matrix[1][3], matrix[2][3] ) );
}

void CPhysicsFluidController::GetSurfacePlane( Vector *pNormal, float *pDist ) const
{
	matrix3x4_t matrix;
	m_pFluidObject->GetPositionMatrix( &matrix );
	const Vector normal = MatrixRotateVector( matrix, m_LocalNormal );
	if ( pNormal )
		*pNormal = normal;
	if ( pDist )
		*pDist = GetWorldDistance( matrix, normal, m_flLocalDistance );
}

void CPhysicsFluidController::GetSurface( b3Vec3 &normal, float &flOffset, b3Vec3 &current )
{
	const uint32_t nStep = m_pFluidObject->GetEnvironment()->GetStepCount();
	if ( m_nSurfaceStep == nStep )
	{
		normal = m_SurfaceNormal;
		flOffset = m_flSurfaceOffset;
		current = m_SurfaceCurrent;
		return;
	}

	matrix3x4_t matrix;
	m_pFluidObject->GetPositionMatrix( &matrix );
	const Vector worldNormal = MatrixRotateVector( matrix, m_LocalNormal );
	normal = { -worldNormal.x, worldNormal.z, -worldNormal.y };
	flOffset = GetWorldDistance( matrix, worldNormal, m_flLocalDistance ) * SOURCE_TO_IVP;

	const Vector worldCurrent = MatrixRotateVector( matrix, m_LocalCurrent );
	current = { worldCurrent.x, -worldCurrent.z, worldCurrent.y };

	m_nSurfaceStep = nStep;
	m_SurfaceNormal = normal;
	m_flSurfaceOffset = flOffset;
	m_SurfaceCurrent = current;
}

void CPhysicsFluidController::WakeAllSleepingObjects()
{
	for ( const std::unique_ptr<CBuoyancyController> &pController : m_Controllers )
		pController->GetObject()->Wake();
}

void CPhysicsFluidController::ObjectEntered( CPhysicsObject *pObject )
{
	if ( pObject->IsStatic() )
		return;

	m_Controllers.push_back( std::make_unique<CBuoyancyController>( this, pObject ) );
}

void CPhysicsFluidController::ObjectLeft( CPhysicsObject *pObject )
{
	auto it = std::find_if( m_Controllers.begin(), m_Controllers.end(), [pObject]( const std::unique_ptr<CBuoyancyController> &pController ) { return pController->GetObject() == pObject; } );
	if ( it != m_Controllers.end() )
		m_Controllers.erase( it );
}

void CPhysicsFluidController::RemoveAllObjects()
{
	m_Controllers.clear();
}

void CPhysicsFluidController::FluidObjectDestroyed()
{
	RemoveAllObjects();
	m_pFluidObject = nullptr;
}
