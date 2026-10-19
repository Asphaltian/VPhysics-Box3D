#include "physics_fluid_controller.h"

#include "compact_builder/compact_surface.h"
#include "compact_builder/math_util.h"
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
static constexpr double PI = std::numbers::pi_v<double>;
static constexpr double THIRD = 1.0 / 3.0;
static constexpr double TWELFTH = 1.0 / 12.0;
static constexpr double NEGATIVE_SIXTH = -1.0 / 6.0;

using Submersion_t = CBuoyancyController::Submersion_t;
using Plane_t = CBuoyancyController::Plane_t;

struct DoublePlane_t
{
	DoubleVec3_t m_Normal;
	double m_flDistance;
};

static float Dot3( b3Vec3 a, b3Vec3 b )
{
	return ( a.x * b.x + a.y * b.y ) + a.z * b.z;
}

static float LengthSquared3( b3Vec3 v )
{
	return Dot3( v, v );
}

static double Length3( b3Vec3 v )
{
	return std::sqrt( static_cast<double>( LengthSquared3( v ) ) );
}

static b3Vec3 Combine( b3Vec3 u, double flWeightU, b3Vec3 v, double flWeightV, double flInverse )
{
	const auto combine = [&]( float a, float b ) {
		const float flSum = static_cast<float>( static_cast<double>( static_cast<float>( static_cast<double>( a ) * flWeightU ) ) + static_cast<double>( b ) * flWeightV );
		return static_cast<float>( static_cast<double>( flSum ) * flInverse );
	};
	return { combine( u.x, v.x ), combine( u.y, v.y ), combine( u.z, v.z ) };
}

static bool Normalize( b3Vec3 &v )
{
	const float flLengthSquared = LengthSquared3( v );
	if ( static_cast<double>( flLengthSquared ) < DOUBLE_RESOLUTION )
		return false;

	v = Scale( v, ApproximateInverseSquareRoot( flLengthSquared, 5 ) );
	return true;
}

static double NormalizeLength( b3Vec3 &v )
{
	const float flLengthSquared = LengthSquared3( v );
	if ( static_cast<double>( flLengthSquared ) < DOUBLE_RESOLUTION )
		return 0;

	const double flInverse = ApproximateInverseSquareRoot( flLengthSquared, 5 );
	v = Scale( v, flInverse );
	return flInverse * static_cast<double>( flLengthSquared );
}

static void NormalizePlane( Plane_t &plane )
{
	const double flInverse = 1.0 / std::sqrt( static_cast<double>( LengthSquared3( plane.m_Normal ) ) );
	plane.m_Normal = Scale( plane.m_Normal, flInverse );
	plane.m_flDistance = static_cast<float>( static_cast<double>( plane.m_flDistance ) * flInverse );
}

static b3Vec3 ProjectOntoPlane( const Plane_t &plane, b3Vec3 point )
{
	const double t = -( ( static_cast<double>( Dot3( plane.m_Normal, point ) ) + static_cast<double>( plane.m_flDistance ) ) / static_cast<double>( LengthSquared3( plane.m_Normal ) ) );
	return ScaleAdd( plane.m_Normal, t, point );
}

static DoubleVec3_t ProjectOntoPlane( const DoublePlane_t &plane, DoubleVec3_t point )
{
	const double t = -( ( Dot( plane.m_Normal, point ) + plane.m_flDistance ) / LengthSquared( plane.m_Normal ) );
	return t * plane.m_Normal + point;
}

static void Normalize( DoubleVec3_t &v )
{
	const double flLengthSquared = LengthSquared( v );
	if ( flLengthSquared >= DOUBLE_RESOLUTION )
		v = ApproximateInverseSquareRoot( flLengthSquared, 5 ) * v;
}

static bool IsParallel( DoubleVec3_t a, DoubleVec3_t b )
{
	const DoubleVec3_t c = Cross( a, b );
	const double flTolerance = PARALLEL_TOLERANCE;
	return ( LengthSquared( b ) * flTolerance ) * ( LengthSquared( a ) * flTolerance ) >= ( c.x * c.x + c.y * c.y ) + c.z * c.z;
}

static bool IntersectRayPlane( const DoublePlane_t &plane, DoubleVec3_t direction, DoubleVec3_t origin, DoubleVec3_t &out )
{
	const DoubleVec3_t &n = plane.m_Normal;
	const double flStart = Dot( n, origin ) + plane.m_flDistance;
	const double flEnd = ( ( n.x * ( direction.x + origin.x ) + n.y * ( direction.y + origin.y ) ) + n.z * ( direction.z + origin.z ) ) + plane.m_flDistance;
	const double flDenominator = flEnd - flStart;
	if ( std::fabs( flDenominator ) < DOUBLE_RESOLUTION )
	{
		if ( !( flStart < DOUBLE_RESOLUTION ) )
			return false;

		out = origin;
		return true;
	}

	out = -( flStart / flDenominator ) * direction + origin;
	return true;
}

static bool IntersectPlanes( const Plane_t &first, const Plane_t &second, b3Vec3 &direction, b3Vec3 &point )
{
	const DoublePlane_t p = { ToDouble( first.m_Normal ), first.m_flDistance };
	const DoublePlane_t w = { ToDouble( second.m_Normal ), second.m_flDistance };
	const DoubleVec3_t p0 = ProjectOntoPlane( p, { 0, 0, 0 } );
	if ( IsParallel( p.m_Normal, w.m_Normal ) )
		return false;

	DoubleVec3_t lineDirection = Cross( p.m_Normal, w.m_Normal );
	Normalize( lineDirection );

	DoubleVec3_t toLine = ProjectOntoPlane( p, ProjectOntoPlane( w, p0 ) ) - p0;
	Normalize( toLine );

	DoubleVec3_t linePoint;
	if ( !IntersectRayPlane( w, toLine, p0, linePoint ) )
		return false;

	direction = ToFloat( lineDirection );
	point = ToFloat( linePoint );
	return true;
}

static bool FindQuadraticRoots( double a, double b, double c, double &t1, double &t2 )
{
	if ( std::fabs( a ) < DOUBLE_RESOLUTION )
	{
		if ( std::fabs( b ) < DOUBLE_RESOLUTION )
			return false;

		t1 = t2 = -( c / b );
		return true;
	}

	const double flDiscriminant = b * b - ( a * c ) * 4;
	if ( !( flDiscriminant >= 0 ) )
		return false;

	const double q = 0.5 / a;
	const double s = std::sqrt( flDiscriminant );
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
	double t1, t2;
	if ( !FindQuadraticRoots( a, b, c, t1, t2 ) )
	{
		hit1 = hit2 = b3Vec3_zero;
		return false;
	}

	hit1 = ScaleAdd( direction, t1, point );
	hit2 = ScaleAdd( direction, t2, point );
	return true;
}

static double GetSilhouetteSegment( float flRadius, double flChord, double flMidDistance )
{
	const float x = static_cast<float>( flChord / static_cast<double>( flRadius + flRadius ) );
	const float x2 = x * x;
	const float x3 = x2 * x;
	const float x5 = x2 * x3;
	const float flPolynomial = ( x5 * 0.58f + x ) + ( x3 * 0.24f + x );
	return std::fabs( ( static_cast<double>( flPolynomial * flRadius ) * static_cast<double>( flRadius ) - ( static_cast<double>( flRadius ) - ( static_cast<double>( flRadius ) - flMidDistance ) ) * flChord ) * 0.5 );
}

enum SegmentForm_t
{
	SEGMENT_UNCLAMPED,
	SEGMENT_ABSOLUTE,
	SEGMENT_REST,
};

struct SmallSegment_t
{
	double m_flArea;
	double m_flOffset;
};

static SmallSegment_t GetSmallSegment( double flRadius, double flSagitta, double flDistance, SegmentForm_t nForm )
{
	double flChord = std::sqrt( std::fabs( ( flSagitta + flSagitta ) * flRadius - flSagitta * flSagitta ) );
	flChord = flChord + flChord;
	const float y = static_cast<float>( ( 0.5 / flRadius ) * flChord );
	const float y2 = y * y;
	const float y3 = y2 * y;
	const float y5 = y3 * y2;
	const float flPolynomial = ( ( y + y3 * 0.12f ) + y3 * 0.12f ) + ( ( y5 * 0.29f + y5 * 0.29f ) + y );

	double flArea;
	if ( nForm == SEGMENT_REST )
		flArea = std::fabs( ( std::fabs( static_cast<double>( flPolynomial ) * flRadius ) * flRadius - flDistance * flChord ) * 0.5 );
	else
		flArea = ( ( static_cast<double>( flPolynomial ) * flRadius ) * flRadius - ( flRadius - flSagitta ) * flChord ) * 0.5;
	if ( nForm == SEGMENT_ABSOLUTE )
		flArea = std::fabs( flArea );

	return { flArea, ( ( TWELFTH / flArea ) * flChord ) * ( flChord * flChord ) };
}

static double GetEllipseSegment( double flWaterRadius, double flAxis, double flDistance, double flChord, bool bChordFirst )
{
	const float flRatio = static_cast<float>( flDistance / flAxis );
	const double flAngle = flRatio < 1.0f ? static_cast<double>( static_cast<float>( std::acos( static_cast<double>( flRatio ) ) ) ) : 0.0;
	const double flChordTerm = bChordFirst ? ( flChord * 0.5 ) * flDistance : ( flDistance * flChord ) * 0.5;
	return ( flWaterRadius * flAxis ) * flAngle - flChordTerm;
}

static void ComputeSphereSubmersion( float flRadius, const Plane_t &water, b3Vec3 velocity, float flPressureDrag, float flSkinDrag, Submersion_t &out )
{
	const b3Vec3 center = b3Vec3_zero;
	const double eps = BUOYANCY_EPSILON;

	Plane_t flow;
	flow.m_Normal = static_cast<double>( LengthSquared3( velocity ) ) < DOUBLE_RESOLUTION ? b3Vec3{ 0.0f, 1.0f, 0.0f } : velocity;
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
		out.m_flVolume = static_cast<float>( static_cast<double>( ( flRadius * flRadius ) * flRadius ) * ( 4.0 * PI / 3.0 ) );
		out.m_Center = center;
	}
	else
	{
		const float h = h0 + flRadius;
		const float t3 = flRadius * 3.0f - h;
		out.m_flVolume = static_cast<float>( ( static_cast<double>( t3 ) * static_cast<double>( h ) ) * ( static_cast<double>( h ) * ( PI / 3.0 ) ) );
		const float s = ( ( ( ( flRadius * 4.0f ) * flRadius - ( flRadius * 4.0f ) * h ) + h * h ) * 0.75f ) / t3;
		out.m_Center = ScaleAdd( n, s, center );
	}

	const b3Vec3 waterCenter = ProjectOntoPlane( water, center );
	const b3Vec3 flowCenter = ProjectOntoPlane( flow, waterCenter );
	const float flFacing = Dot3( f, n );
	const b3Vec3 projectedNormal = ProjectOntoPlane( flow, n );
	const double flWaterRadius = std::sqrt( static_cast<double>( std::fabs( flRadius * flRadius - h0 * h0 ) ) );
	const double flCircle = ( static_cast<double>( flRadius ) * PI ) * static_cast<double>( flRadius );
	const double flWetDisc = ( static_cast<double>( flRadius * flRadius - h0 * h0 ) * static_cast<double>( flFacing ) ) * PI;

	double a = 0;
	b3Vec3 g = b3Vec3_zero;
	const auto applyCenterWet = [&]() {
		if ( !( static_cast<double>( flFacing ) > 0 ) )
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

		g = Combine( flowCenter, -flWetDisc, center, flCircle, 1.0 / a );
	};
	const auto applyCenterDry = [&]() {
		if ( static_cast<double>( flFacing ) >= 0 )
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
	const double flChord = Length3( chord );
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
	else if ( static_cast<double>( LengthSquared3( chordDirection ) ) < eps )
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

		const double flAxis = Length3( b3Sub( flowCenter, q ) );
		const double flMidOffset = Length3( b3Sub( flowCenter, mid ) );
		const double flMidDistance = Length3( b3Sub( mid, center ) );
		const double flEllipse = ( flWaterRadius * PI ) * flAxis;
		const double flCube = ( flChord * flChord ) * flChord;

		if ( nCase == 1 && static_cast<double>( flFacing ) > eps )
		{
			const double s = GetSilhouetteSegment( flRadius, flChord, flMidDistance );
			const double e = flAxis >= eps ? GetEllipseSegment( flWaterRadius, flAxis, flMidOffset, flChord, false ) : 0.0;
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
				g = Combine( ellipseCenter, -e, silhouetteCenter, s, 1.0 / a );
			}
		}
		else if ( nCase == 1 )
		{
			const double s = GetSilhouetteSegment( flRadius, flChord, flMidDistance );
			b3Vec3 silhouetteCenter = b3Vec3_zero;
			bool bDone = false;
			if ( s >= eps )
			{
				const double k = ( TWELFTH / s ) * flCube;
				if ( static_cast<double>( flFacing ) > -eps )
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
				double flRest = 0;
				b3Vec3 restCenter = b3Vec3_zero;
				if ( flAxis >= eps )
				{
					const double e = std::fabs( GetEllipseSegment( flWaterRadius, flAxis, flMidOffset, flChord, true ) );
					flRest = std::fabs( flEllipse - e );
					const SmallSegment_t segment = GetSmallSegment( flAxis, flAxis - flMidOffset, flMidOffset, SEGMENT_REST );
					double flSegmentWeight = e;
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
						restCenter = Combine( segmentCenter, -flSegmentWeight, flowCenter, flEllipse, 1.0 / flRest );
				}

				a = flRest + s;
				if ( a < eps )
				{
					a = 0;
					g = b3Vec3_zero;
				}
				else
				{
					g = Combine( silhouetteCenter, s, restCenter, flRest, 1.0 / a );
				}
			}
		}
		else if ( static_cast<double>( flFacing ) <= eps )
		{
			const double s = GetSilhouetteSegment( flRadius, flChord, flMidDistance );
			const double flRemainder = std::fabs( flCircle - s );
			b3Vec3 remainderCenter = chord;
			bool bDone = false;
			if ( s >= eps )
			{
				if ( static_cast<double>( flFacing ) > -eps )
				{
					g = Combine( ScaleAdd( n, ( -TWELFTH / s ) * flCube, center ), -s, center, flCircle, 1.0 / flRemainder );
					a = flRemainder;
					bDone = true;
				}
				else
				{
					remainderCenter = Combine( ScaleAdd( chordDirection, ( TWELFTH / s ) * flCube, center ), -s, center, flCircle, 1.0 / flRemainder );
				}
			}

			if ( !bDone )
			{
				double e = 0;
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
					g = Combine( remainderCenter, flRemainder, ellipseCenter, e, 1.0 / a );
				}
			}
		}
		else
		{
			const double s = GetSilhouetteSegment( flRadius, flChord, flMidDistance );
			if ( s < eps )
			{
				applyCenterWet();
			}
			else
			{
				const double e = flAxis >= eps ? GetEllipseSegment( flWaterRadius, flAxis, flMidOffset, flChord, false ) : 0.0;
				a = std::fabs( ( static_cast<double>( flRadius * flRadius ) - flWaterRadius * flAxis ) * PI - std::fabs( s - e ) );
				if ( a < eps )
				{
					a = 0;
					g = b3Vec3_zero;
				}
				else
				{
					double flRemainder = std::fabs( flCircle - s );
					b3Vec3 remainderCenter = b3Vec3_zero;
					if ( flRemainder < eps )
						flRemainder = 0;
					else
						remainderCenter = Combine( ScaleAdd( chordDirection, ( ( TWELFTH / s ) * flChord ) * ( flChord * flChord ), center ), -s, center, flCircle, 1.0 / flRemainder );

					b3Vec3 ellipseCenter = b3Vec3_zero;
					if ( flAxis >= eps )
					{
						const SmallSegment_t segment = GetSmallSegment( flAxis, std::fabs( flAxis - flMidOffset ), flMidOffset, SEGMENT_UNCLAMPED );
						if ( segment.m_flArea >= eps )
							ellipseCenter = ScaleAdd( chordDirection, segment.m_flOffset, flowCenter );
					}

					double flEllipseRest = std::fabs( flEllipse - e );
					b3Vec3 ellipseRestCenter = b3Vec3_zero;
					if ( flEllipseRest < eps )
						flEllipseRest = 0;
					else
						ellipseRestCenter = Combine( ellipseCenter, -e, flowCenter, flEllipse, 1.0 / flEllipseRest );

					g = Combine( ellipseRestCenter, -flEllipseRest, remainderCenter, flRemainder, 1.0 / a );
				}
			}
		}
	}

	double flSpeedSquared = LengthSquared3( velocity );
	if ( flSpeedSquared < eps )
	{
		velocity = { 0.0f, 1.0f, 0.0f };
		flSpeedSquared = 0;
	}

	const double flMagnitude = ( ( a * 3 ) * flSpeedSquared ) * static_cast<double>( flSkinDrag ) + ( flSpeedSquared * a ) * static_cast<double>( flPressureDrag );
	Normalize( velocity );
	out.m_flArea = static_cast<float>( static_cast<double>( out.m_flArea ) + a );
	const b3Vec3 force = Scale( velocity, flMagnitude );
	out.m_Torque = b3Add( b3Cross( g, force ), out.m_Torque );
	out.m_Force = b3Add( force, out.m_Force );
}

static b3Vec3 GetWeightedPoint( const b3Vec3 points[3], int k0, double w0, double w1, double w2 )
{
	const int k1 = ( k0 + 1 ) % 3, k2 = ( k0 + 2 ) % 3;
	const auto cut = [&]( float p0, float p1, float p2 ) {
		const float flFirst = static_cast<float>( w0 * static_cast<double>( p0 ) );
		const float flSecond = static_cast<float>( static_cast<double>( flFirst ) + w1 * static_cast<double>( p1 ) );
		return static_cast<float>( static_cast<double>( flSecond ) + w2 * static_cast<double>( p2 ) );
	};
	return {
		cut( points[k0].x, points[k1].x, points[k2].x ),
		cut( points[k0].y, points[k1].y, points[k2].y ),
		cut( points[k0].z, points[k1].z, points[k2].z ),
	};
}

static void ComputePolygonSubmersion( const CPhysCollideCompactSurface &surface, const std::vector<int> &ledges, const DoubleMatrix3_t &coreRotation, DoubleVec3_t coreOrigin, b3Vec3 coreAngular, b3Vec3 coreVelocity, const DoubleMatrix3_t &rotation, DoubleVec3_t origin, const Plane_t &water, float flPressureDrag, float flSkinDrag, b3Vec3 fluidVelocity, Submersion_t &out )
{
	const float eps = BUOYANCY_EPSILON;
	const b3Vec3 n = water.m_Normal;
	const b3Vec3 surfacePoint = Scale( n, -static_cast<double>( water.m_flDistance ) );

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
			const DoubleVec3_t e1 = ToDouble( points[1] ) - ToDouble( points[0] );
			const DoubleVec3_t e2 = ToDouble( points[2] ) - ToDouble( points[0] );
			const DoubleVec3_t lever = ToDouble( b3Sub( points[0], surfacePoint ) );
			const DoubleVec3_t c = Cross( e2, e1 );
			const float flTetraVolume = static_cast<float>( ( ( c.y * lever.y + c.x * lever.x ) + c.z * lever.z ) * NEGATIVE_SIXTH );

			double flFraction = 1;
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
				const double w0 = ( 3.0 - tau1 ) - tau2;
				const double p = static_cast<double>( tau2 ) * static_cast<double>( tau1 );
				const b3Vec3 cut = GetWeightedPoint( points, k0, w0, tau1, tau2 );
				const b3Vec3 thirdWeighted = GetWeightedPoint( points, k0, w0 * THIRD, tau1 * THIRD, tau2 * THIRD );
				if ( nDryCount == 1 )
				{
					const double flPartVolume = static_cast<double>( flTetraVolume ) - ( static_cast<double>( flTetraVolume ) * tau1 ) * tau2;
					flVolume = static_cast<float>( static_cast<double>( flVolume ) + flPartVolume );
					flFraction = 1.0 - p;
					if ( flFraction < static_cast<double>( eps ) )
					{
						flVolume = 0.0f;
						volumeCenter = b3Vec3_zero;
						continue;
					}

					const b3Vec3 partCenter = Scale( b3Sub( sum, Scale( cut, p ) ), 1.0 / flFraction );
					volumeCenter = b3Add( Scale( b3Add( partCenter, surfacePoint ), flPartVolume ), volumeCenter );
					wetCenter = Scale( b3Sub( Scale( sum, THIRD ), Scale( thirdWeighted, p ) ), 1.0 / flFraction );
				}
				else
				{
					const double flPartVolume = static_cast<double>( flTetraVolume ) * p;
					flVolume = static_cast<float>( static_cast<double>( flVolume ) + flPartVolume );
					volumeCenter = b3Add( Scale( b3Add( cut, surfacePoint ), flPartVolume ), volumeCenter );
					flFraction = static_cast<float>( p );
					wetCenter = thirdWeighted;
				}
			}

			b3Vec3 faceNormal = ToFloat( Cross( e1, e2 ) );
			if ( LengthSquared3( faceNormal ) < eps )
				continue;

			const DoubleVec3_t worldPoint = Rotate( rotation, wetCenter ) + origin;
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
			const float flArea = static_cast<float>( NormalizeLength( faceNormal ) * 0.5 );
			const float flFacing = Dot3( faceNormal, flowVelocity );
			const float flProjectedArea = -( ( flFacing * static_cast<float>( flFraction ) ) * flArea );
			const double flMagnitude = static_cast<double>( ( flProjectedArea * flSpeedSquared ) * flPressureDrag ) + static_cast<double>( ( flArea * flSpeedSquared ) * flSkinDrag );
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

void CBuoyancyController::Prepare()
{
	CPhysicsObject *pObject = m_pObject;
	m_nPreparedStep = pObject->GetEnvironment()->GetStepCount();
	const bool bSimulate = !pObject->GetShadowController() && ( pObject->GetCallbackFlags() & CALLBACK_DO_FLUID_SIMULATION );
	m_flDensity = bSimulate ? pObject->GetBuoyancyRatio() * m_pFluid->GetDensity() : 0.0f;
	if ( m_flDensity == 0.0f || std::isnan( m_flDensity ) )
		return;

	b3Vec3 surfaceNormal, current;
	float flSurfaceOffset;
	m_pFluid->GetSurface( surfaceNormal, flSurfaceOffset, current );

	m_PreparedFluidVelocity = m_flArea > m_flPreviousArea ? b3MulSV( m_flPreviousArea / m_flArea, m_FluidVelocity ) : m_FluidVelocity;
	m_Speed = pObject->GetCoreVelocity();
	m_WorldFluidVelocity = b3Add( current, m_PreparedFluidVelocity );
	m_CoreAngularVelocity = pObject->GetCoreAngularVelocity();
	m_CoreRotation = pObject->GetCoreRotation();
	m_CoreOrigin = pObject->GetCorePosition();

	pObject->GetPose( pObject->GetEnvironment()->GetCurrentTime(), m_Rotation, m_Origin );
	const DoubleVec3_t n = ToDouble( surfaceNormal );
	const float flLocalOffset = static_cast<float>( ( n.x * m_Origin.x + n.y * m_Origin.y ) + ( n.z * m_Origin.z + static_cast<double>( flSurfaceOffset ) ) );
	m_Water = { ToFloat( InverseRotate( m_Rotation, surfaceNormal ) ), flLocalOffset };
	m_flPressureDrag = ( m_pFluid->GetDamping() * 0.5f ) * m_flDensity;
	m_flSkinDrag = ( SKIN_FRICTION_COEFFICIENT * 0.5f ) * m_flDensity;

	m_pSurface = pObject->GetCollide() ? pObject->GetCollide()->GetCompactSurface() : nullptr;
	m_bSphere = !pObject->GetCollide();
	m_flSphereRadius = pObject->GetIVPSphereRadius();
}

void CBuoyancyController::ComputeSubmersion()
{
	m_Submersion = {};
	if ( !( m_flDensity > 0.0f ) )
		return;

	const b3Vec3 localFluidVelocity = ToFloat( InverseRotate( m_Rotation, m_WorldFluidVelocity ) );
	if ( m_pSurface )
		ComputePolygonSubmersion( *m_pSurface, m_Ledges, m_CoreRotation, m_CoreOrigin, m_CoreAngularVelocity, m_Speed, m_Rotation, m_Origin, m_Water, m_flPressureDrag, m_flSkinDrag, localFluidVelocity, m_Submersion );
	else if ( m_bSphere )
		ComputeSphereSubmersion( m_flSphereRadius, m_Water, ToFloat( InverseRotate( m_Rotation, b3Sub( m_WorldFluidVelocity, m_Speed ) ) ), m_flPressureDrag, m_flSkinDrag, m_Submersion );

	if ( !( m_Submersion.m_flVolume > BUOYANCY_EPSILON ) && !( LengthSquared3( m_Submersion.m_Force ) > BUOYANCY_EPSILON ) )
		m_Submersion = {};
}

void CBuoyancyController::Simulate( CPhysicsObject *pObject, float flTimestep )
{
	if ( m_nPreparedStep != pObject->GetEnvironment()->GetStepCount() )
	{
		Prepare();
		ComputeSubmersion();
	}
	m_nPreparedStep = UINT32_MAX;
	if ( m_flDensity == 0.0f || std::isnan( m_flDensity ) )
		return;

	m_FluidVelocity = m_PreparedFluidVelocity;
	m_flPreviousArea = m_flArea;
	m_flArea = 0.0f;

	const Submersion_t &submersion = m_Submersion;
	if ( submersion.m_flVolume > BUOYANCY_EPSILON )
	{
		const DoubleVec3_t gravity = pObject->GetEnvironment()->GetIVPGravity();
		const double flScale = static_cast<double>( -( submersion.m_flVolume * m_flDensity ) ) * static_cast<double>( flTimestep );
		const b3Vec3 impulse = Scale( { static_cast<float>( gravity.x ), static_cast<float>( gravity.y ), static_cast<float>( gravity.z ) }, flScale );
		pObject->ApplyImpulse( Rotate( m_Rotation, submersion.m_Center ) + m_Origin, impulse );
	}

	const float flArea = std::fabs( submersion.m_flArea );
	const float flTangentScale = static_cast<float>( ( std::sqrt( static_cast<double>( flArea ) ) * static_cast<double>( -TANGENTIAL_TORQUE_COEFFICIENT ) ) * static_cast<double>( TANGENTIAL_TORQUE_SCALE ) );
	const b3Vec3 tangentTorque = Scale( submersion.m_TangentTorque, static_cast<double>( flTangentScale ) * static_cast<double>( flTimestep ) );
	const b3Vec3 torque = b3MulSV( flTimestep, submersion.m_Torque );
	pObject->ApplyImpulse( m_Origin, ToFloat( Rotate( m_Rotation, b3MulSV( flTimestep, submersion.m_Force ) ) ) );

	const b3Vec3 inverseInertia = pObject->GetCoreInverseInertia();
	const b3Vec3 angularChange = b3Add( b3Mul( tangentTorque, inverseInertia ), b3Mul( torque, inverseInertia ) );
	pObject->AddPendingAngularVelocity( angularChange );

	if ( m_bSphere )
	{
		const double flSpin = static_cast<double>( flArea * SPHERE_SPIN_DAMPING ) / ( ( static_cast<double>( m_flSphereRadius ) * ( PI + PI ) ) * static_cast<double>( m_flSphereRadius ) );
		pObject->AddPendingAngularVelocity( b3Neg( Scale( pObject->GetCoreAngularVelocity(), flSpin ) ) );
	}

	m_flArea = static_cast<float>( static_cast<double>( flArea ) + static_cast<double>( m_flArea ) );

	m_FluidVelocity = ScaleAdd( b3Sub( m_Speed, m_WorldFluidVelocity ), FLUID_VELOCITY_DRAG * flTimestep, m_FluidVelocity );
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

void CPhysicsFluidController::PrepareControllers( uint32_t nStep, std::vector<CBuoyancyController *> &prepared )
{
	for ( const std::unique_ptr<CBuoyancyController> &pController : m_Controllers )
	{
		if ( pController->GetObject()->GetControllerStep() != nStep )
			continue;

		pController->Prepare();
		prepared.push_back( pController.get() );
	}
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
