//=================================================================================================
//
// Source / Box3D utilities
//
// All unit and type conversion between Source and Box3D lives here. Source works in inches,
// Box3D in metres. Both are Z-up right-handed, so live conversion is pure scale + type; the
// only genuine axis remap in the codebase is reading legacy IVP collision data (see vbox_collide).
//
//=================================================================================================

#pragma once

inline constexpr float InchesToMetres = 0.0254f;
inline constexpr float MetresToInches = 1.0f / 0.0254f;

#define vox_expr inline

vox_expr Vector VectorHalfExtent( Vector mins, Vector maxs )
{
	return 0.5f * ( maxs - mins );
}

vox_expr Quaternion ToQuaternion( const QAngle& angles )
{
	Quaternion result;
	AngleQuaternion( angles, result );
	return result;
}

vox_expr QAngle ToQAngle( const Quaternion &q )
{
	QAngle result;
	QuaternionAngles( q, result );
	return result;
}

vox_expr QAngle ToQAngle( const matrix3x4_t& m )
{
	QAngle result;
	MatrixAngles( m, result );
	return result;
}

vox_expr Vector Abs( const Vector &v )
{
	Vector result;
	VectorAbs( v, result );
	return result;
}

vox_expr Vector Rotate( const Vector &vector, const Quaternion &angle )
{
	Vector out;
	VectorRotate( vector, angle, out );
	return out;
}

vox_expr Vector Rotate( const Vector& vector, const matrix3x4_t &matrix )
{
	Vector out;
	VectorRotate( vector, matrix, out );
	return out;
}

template < typename T >
constexpr T Cube( T x )
{
	return x * x * x;
}

namespace MatrixAxis
{
	enum SourceMatrixAxes
	{
		Forward = 0,
		Left = 1,
		Up = 2,

		X = 0,
		Y = 1,
		Z = 2,

		Origin = 3,
		Projective = 3,
	};
}
using SourceMatrixAxes = MatrixAxis::SourceMatrixAxes;

vox_expr Vector GetColumn( const matrix3x4_t& m, SourceMatrixAxes axis )
{
	Vector value;
	MatrixGetColumn( m, (int)axis, value );
	return value;
}

//
// BoxToSource:
//
// Type conversions from Box3D -> Source types, and unit conversions from
// metres -> inches (distance, area, volume, energy) and radians -> degrees.
namespace BoxToSource
{
	inline constexpr float Factor	 = MetresToInches;
	inline constexpr float InvFactor = InchesToMetres;

	// Direct type conversions: normals, directions, coefficients, dimensionless quantities.
	vox_expr float		Unitless( float value )			{ return value; }
	vox_expr Vector		Unitless( b3Vec3 value )		{ return Vector( value.x, value.y, value.z ); }

	// Any unit with a singular metre factor: distance (m), velocity (m/s), acceleration, force.
	vox_expr float		Distance( float value )			{ return value * Factor; }
	vox_expr Vector		Distance( b3Vec3 value )		{ return Vector( Distance( value.x ), Distance( value.y ), Distance( value.z ) ); }

	vox_expr float		Area( float value )				{ return value * Factor * Factor; }
	vox_expr float		Volume( float value )			{ return value * Factor * Factor * Factor; }

	// b3Quat -> Quaternion is a direct passthrough (both are x,y,z,w). Angle also does rad -> deg.
	vox_expr Quaternion	Quat( b3Quat value )			{ return Quaternion( value.v.x, value.v.y, value.v.z, value.s ); }
	vox_expr float		Angle( float value )			{ return RAD2DEG( value ); }
	vox_expr QAngle		Angle( b3Quat value )			{ return ToQAngle( Quat( value ) ); }

	vox_expr float		Energy( float value )			{ return value / ( InvFactor * InvFactor ); }

	vox_expr float		AngularImpulse( float value )	{ return Angle( value ); }
	vox_expr Vector		AngularImpulse( b3Vec3 value )	{ return Vector( AngularImpulse( value.x ), AngularImpulse( value.y ), AngularImpulse( value.z ) ); }

	// Box3D AABBs are min/max, matching Source's mins/maxs.
	vox_expr void		AABBBounds( const b3AABB &box, Vector &outMins, Vector &outMaxs )
	{
		outMins = Distance( box.lowerBound );
		outMaxs = Distance( box.upperBound );
	}

	vox_expr matrix3x4_t Matrix( const b3Transform &t )
	{
		matrix3x4_t m;
		QuaternionMatrix( Quat( t.q ), Distance( t.p ), m );
		return m;
	}
}

//
// SourceToBox:
//
// Type conversions from Source -> Box3D types, and unit conversions from
// inches -> metres (distance, area, volume, energy) and degrees -> radians.
namespace SourceToBox
{
	inline constexpr float Factor	 = InchesToMetres;
	inline constexpr float InvFactor = MetresToInches;

	vox_expr float		Unitless( float value )			{ return value; }
	vox_expr b3Vec3		Unitless( Vector value )		{ return b3Vec3{ value.x, value.y, value.z }; }

	constexpr float		Distance( float value )			{ return value * Factor; }
	vox_expr b3Vec3		Distance( Vector value )		{ return b3Vec3{ Distance( value.x ), Distance( value.y ), Distance( value.z ) }; }

	vox_expr float		Area( float value )				{ return value * Factor * Factor; }
	vox_expr float		Volume( float value )			{ return value * Factor * Factor * Factor; }

	vox_expr b3Quat		Quat( Quaternion value )		{ return b3Quat{ b3Vec3{ value.x, value.y, value.z }, value.w }; }
	vox_expr float		Angle( float value )			{ return DEG2RAD( value ); }
	vox_expr b3Quat		Angle( QAngle value )			{ return Quat( ToQuaternion( value ) ); }

	vox_expr float		Energy( float value )			{ return value / ( InvFactor * InvFactor ); }

	vox_expr float		AngularImpulse( float value )	{ return Angle( value ); }
	vox_expr b3Vec3		AngularImpulse( Vector value )	{ return b3Vec3{ AngularImpulse( value.x ), AngularImpulse( value.y ), AngularImpulse( value.z ) }; }

	vox_expr b3AABB		AABBBounds( Vector mins, Vector maxs )
	{
		return b3AABB{ Distance( mins ), Distance( maxs ) };
	}

	vox_expr b3Transform Transform( const matrix3x4_t &m )
	{
		Quaternion q;
		MatrixQuaternion( m, q );
		return b3Transform{ Distance( GetColumn( m, MatrixAxis::Origin ) ), Quat( q ) };
	}
}

// Same as CM_ClearTrace
inline void ClearTrace( trace_t *trace )
{
	memset( trace, 0, sizeof( *trace ) );
	trace->fraction = 1.0f;
	trace->fractionleftsolid = 0.0f;
	trace->surface.name = "**empty**";
}

template < typename T >
bool VectorContains( const std::vector< T >& vector, const T &object )
{
	return std::find( vector.begin(), vector.end(), object ) != vector.end();
}

template< typename T, typename Value >
constexpr void Erase( T &c, const Value &value )
{
	auto it = std::remove( c.begin(), c.end(), value );
	c.erase( it, c.end() );
}

template< typename T, typename Pred >
constexpr void EraseIf( T &c, Pred pred )
{
	auto it = std::remove_if( c.begin(), c.end(), pred );
	c.erase( it, c.end() );
}

template< typename T, typename Value >
constexpr bool Contains( const T &c, const Value &value )
{
	return c.find( value ) != c.end();
}
