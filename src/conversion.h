#pragma once

#include "mathlib/vector.h"
#include "tier0/valve_minmax_off.h"

#include "box3d/math_functions.h"

#include <cmath>
#include <numbers>

inline b3Vec3 ToBox3D( const Vector &vector )
{
	return { vector.x, vector.y, vector.z };
}

inline Vector ToSource( b3Vec3 vector )
{
	return Vector( vector.x, vector.y, vector.z );
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
