#pragma once

#include "mathlib/mathlib.h"
#include "mathlib/vector.h"
#include "tier0/valve_minmax_off.h"

#include "compact_builder/precision.h"

#include "box3d/math_functions.h"

#include <cmath>
#include <numbers>

inline constexpr float SOURCE_TO_IVP = 0.0254f;
inline constexpr float IVP_TO_SOURCE = 39.37008f;
inline constexpr float DEGREES_TO_RADIANS = 0.017453292f;
inline constexpr float RADIANS_TO_DEGREES = 57.29578f;

inline b3Vec3 ToBox3D( const Vector &vector )
{
	return { vector.x, vector.y, vector.z };
}

inline Vector ToSource( b3Vec3 vector )
{
	return Vector( vector.x, vector.y, vector.z );
}

inline b3Vec3 SourceToIVP( b3Vec3 vector )
{
	return { vector.x * SOURCE_TO_IVP, -( vector.z * SOURCE_TO_IVP ), vector.y * SOURCE_TO_IVP };
}

inline b3Vec3 SourceToIVP( const Vector &vector )
{
	return SourceToIVP( ToBox3D( vector ) );
}

inline b3Vec3 IVPToSource( b3Vec3 vector )
{
	return { IVP_TO_SOURCE * vector.x, IVP_TO_SOURCE * vector.z, -( IVP_TO_SOURCE * vector.y ) };
}

inline b3Vec3 SourceToIVPAngular( const Vector &angular )
{
	return { angular.x * DEGREES_TO_RADIANS, angular.z * -DEGREES_TO_RADIANS, angular.y * DEGREES_TO_RADIANS };
}

inline b3Vec3 IVPToSourceAngular( b3Vec3 angular )
{
	return { angular.x * RADIANS_TO_DEGREES, angular.z * RADIANS_TO_DEGREES, angular.y * -RADIANS_TO_DEGREES };
}

inline b3Quat ToBox3D( const QAngle &angles )
{
	constexpr float flHalfDegreesToRadians = std::numbers::pi_v<float> / 360.0f;

	const float sp = std::sin( angles.x * flHalfDegreesToRadians );
	const float cp = std::cos( angles.x * flHalfDegreesToRadians );
	const float sy = std::sin( angles.y * flHalfDegreesToRadians );
	const float cy = std::cos( angles.y * flHalfDegreesToRadians );
	const float sr = std::sin( angles.z * flHalfDegreesToRadians );
	const float cr = std::cos( angles.z * flHalfDegreesToRadians );

	b3Quat q;
	q.v.x = sr * cp * cy - cr * sp * sy;
	q.v.y = cr * sp * cy + sr * cp * sy;
	q.v.z = cr * cp * sy - sr * sp * cy;
	q.s = cr * cp * cy + sr * sp * sy;
	return q;
}

inline b3Transform ToBox3D( const Vector &origin, const QAngle &angles )
{
	return { ToBox3D( origin ), ToBox3D( angles ) };
}

inline b3Quat IVPToBox3D( RealQuat_t q )
{
	return { { static_cast<float>( q.x ), static_cast<float>( q.z ), -static_cast<float>( q.y ) }, static_cast<float>( q.w ) };
}

inline RealQuat_t Box3DToIVP( b3Quat q )
{
	return { q.v.x, -q.v.z, q.v.y, q.s };
}

inline Vector MatrixRotateVector( const matrix3x4_t &m, const Vector &v )
{
	return Vector( ( v.x * m[0][0] + v.y * m[0][1] ) + v.z * m[0][2], ( v.x * m[1][0] + v.y * m[1][1] ) + v.z * m[1][2], ( v.x * m[2][0] + v.y * m[2][1] ) + v.z * m[2][2] );
}

inline Vector MatrixInverseRotateVector( const matrix3x4_t &m, const Vector &v )
{
	return Vector( ( v.x * m[0][0] + v.y * m[1][0] ) + v.z * m[2][0], ( v.x * m[0][1] + v.y * m[1][1] ) + v.z * m[2][1], ( v.x * m[0][2] + v.y * m[1][2] ) + v.z * m[2][2] );
}

void AngleVectors( const QAngle &angles, b3Vec3 &forward, b3Vec3 &right, b3Vec3 &up );

inline Vector IVPToSourcePosition( RealVec3_t position )
{
	return Vector( static_cast<float>( real_t( IVP_TO_SOURCE ) * position.x ), static_cast<float>( real_t( IVP_TO_SOURCE ) * position.z ), -static_cast<float>( real_t( IVP_TO_SOURCE ) * position.y ) );
}

RealQuat_t AnglesToIVP( const QAngle &angles );
RealQuat_t MatrixToIVP( const matrix3x4_t &matrix, RealVec3_t &origin );
QAngle IVPToAngles( const RealMatrix3_t &rotation );
void IVPToMatrix( const RealMatrix3_t &rotation, RealVec3_t origin, matrix3x4_t &matrix );
QAngle MatrixAngles( const matrix3x4_t &matrix );
