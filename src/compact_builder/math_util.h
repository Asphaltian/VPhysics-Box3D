#pragma once

#include "box3d/math_functions.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#include <xmmintrin.h>

inline constexpr double DOUBLE_EPSILON = 1e-12;
inline constexpr double DOUBLE_RESOLUTION = 1e-19;

template<typename To, typename From>
inline To BitCast( const From &value )
{
	static_assert( sizeof( To ) == sizeof( From ) );
	To result;
	std::memcpy( &result, &value, sizeof( To ) );
	return result;
}

struct DoubleVec3_t
{
	double x, y, z;
};

inline DoubleVec3_t ToDouble( b3Vec3 v )
{
	return { v.x, v.y, v.z };
}

inline b3Vec3 ToFloat( DoubleVec3_t v )
{
	return { static_cast<float>( v.x ), static_cast<float>( v.y ), static_cast<float>( v.z ) };
}

inline DoubleVec3_t operator+( DoubleVec3_t a, DoubleVec3_t b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

inline DoubleVec3_t operator-( DoubleVec3_t a, DoubleVec3_t b )
{
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}

inline DoubleVec3_t operator-( DoubleVec3_t a )
{
	return { -a.x, -a.y, -a.z };
}

inline DoubleVec3_t operator*( double s, DoubleVec3_t a )
{
	return { s * a.x, s * a.y, s * a.z };
}

inline double Dot( DoubleVec3_t a, DoubleVec3_t b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline DoubleVec3_t Cross( DoubleVec3_t a, DoubleVec3_t b )
{
	return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

inline double LengthSquared( DoubleVec3_t a )
{
	return Dot( a, a );
}

inline double Length( DoubleVec3_t a )
{
	return std::sqrt( LengthSquared( a ) );
}

inline double Component( DoubleVec3_t v, int nAxis )
{
	return nAxis == 0 ? v.x : ( nAxis == 1 ? v.y : v.z );
}

inline float FastInverseSquareRoot( float x )
{
	const float r = _mm_cvtss_f32( _mm_rsqrt_ss( _mm_set_ss( x ) ) );
	return ( r * ( 3.0f - ( r * r ) * x ) ) * 0.5f;
}

inline float FastReciprocal( float x )
{
	const float r = _mm_cvtss_f32( _mm_rcp_ss( _mm_set_ss( x ) ) );
	return ( r + r ) - ( r * r ) * x;
}

inline double ApproximateInverseSquareRoot( double x, int nRefinements )
{
	const double flHalf = x * 0.5;
	const int32_t nSeed = ( ( 0x7FF00000 - static_cast<int32_t>( BitCast<uint64_t>( x ) >> 32 ) ) >> 1 ) + 0x1FF00000;
	double flInverse = BitCast<double>( static_cast<uint64_t>( static_cast<uint32_t>( nSeed ) ) << 32 );
	for ( int i = 0; i < nRefinements; ++i )
		flInverse = ( ( 0.5 - flInverse * flInverse * flHalf ) + 1.0 ) * flInverse;
	return flInverse;
}

struct DoubleQuat_t
{
	double x, y, z, w;
};

struct DoubleMatrix3_t
{
	double m[3][3];
};

inline DoubleQuat_t ConjugateQuat( DoubleQuat_t q )
{
	return { -q.x, -q.y, -q.z, q.w };
}

inline DoubleQuat_t MultiplyQuat( DoubleQuat_t a, DoubleQuat_t b )
{
	return {
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

inline DoubleQuat_t NormalizeQuat( DoubleQuat_t q )
{
	const double n = ( q.w * q.w + q.z * q.z ) + ( q.x * q.x + q.y * q.y );
	if ( !( n > DOUBLE_RESOLUTION ) )
		return q;

	const double r = 1.0 / std::sqrt( n );
	return { r * q.x, r * q.y, r * q.z, r * q.w };
}

inline DoubleQuat_t MatrixToQuat( const DoubleMatrix3_t &matrix )
{
	const auto &m = matrix.m;
	DoubleQuat_t q;
	const double trace = ( m[0][0] + m[1][1] ) + m[2][2];
	if ( trace > 0 )
	{
		const double s = std::sqrt( trace + 1.0 );
		const double f = 0.5 / s;
		q.w = s * 0.5;
		q.x = ( m[2][1] - m[1][2] ) * f;
		q.y = ( m[0][2] - m[2][0] ) * f;
		q.z = ( m[1][0] - m[0][1] ) * f;
	}
	else
	{
		static constexpr int s_Next[3] = { 1, 2, 0 };
		int i = m[1][1] > m[0][0] ? 1 : 0;
		if ( m[2][2] > m[i][i] )
			i = 2;
		const int j = s_Next[i];
		const int k = s_Next[j];

		double *const pComponents[3] = { &q.x, &q.y, &q.z };
		const double s = std::sqrt( ( m[i][i] - ( m[j][j] + m[k][k] ) ) + 1.0 );
		const double f = s != 0 ? 0.5 / s : s;
		*pComponents[i] = s * 0.5;
		q.w = ( m[k][j] - m[j][k] ) * f;
		*pComponents[j] = ( m[j][i] + m[i][j] ) * f;
		*pComponents[k] = ( m[k][i] + m[i][k] ) * f;
	}

	return NormalizeQuat( q );
}

inline DoubleMatrix3_t QuatToMatrix( DoubleQuat_t q )
{
	const double x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
	const double xx = q.x * x2, yy = q.y * y2, zz = q.z * z2;
	const double xy = q.x * y2, xz = q.x * z2, yz = q.y * z2;
	const double wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;

	DoubleMatrix3_t matrix;
	auto &m = matrix.m;
	m[0][0] = 1.0 - ( zz + yy );
	m[1][1] = 1.0 - ( zz + xx );
	m[2][2] = 1.0 - ( yy + xx );
	m[0][1] = xy - wz;
	m[1][0] = wz + xy;
	m[0][2] = wy + xz;
	m[2][0] = xz - wy;
	m[1][2] = yz - wx;
	m[2][1] = wx + yz;
	return matrix;
}

inline DoubleVec3_t Rotate( const DoubleMatrix3_t &rotation, b3Vec3 v )
{
	const auto &m = rotation.m;
	return {
		( static_cast<double>( v.x ) * m[0][0] + static_cast<double>( v.y ) * m[0][1] ) + static_cast<double>( v.z ) * m[0][2],
		( static_cast<double>( v.x ) * m[1][0] + static_cast<double>( v.y ) * m[1][1] ) + static_cast<double>( v.z ) * m[1][2],
		( static_cast<double>( v.x ) * m[2][0] + static_cast<double>( v.y ) * m[2][1] ) + static_cast<double>( v.z ) * m[2][2],
	};
}

inline DoubleVec3_t InverseRotate( const DoubleMatrix3_t &rotation, b3Vec3 v )
{
	const auto &m = rotation.m;
	return {
		( static_cast<double>( v.x ) * m[0][0] + static_cast<double>( v.y ) * m[1][0] ) + static_cast<double>( v.z ) * m[2][0],
		( static_cast<double>( v.x ) * m[0][1] + static_cast<double>( v.y ) * m[1][1] ) + static_cast<double>( v.z ) * m[2][1],
		( static_cast<double>( v.x ) * m[0][2] + static_cast<double>( v.y ) * m[1][2] ) + static_cast<double>( v.z ) * m[2][2],
	};
}

inline DoubleVec3_t InverseRotate( const DoubleMatrix3_t &rotation, DoubleVec3_t v )
{
	const auto &m = rotation.m;
	return {
		( v.x * m[0][0] + v.y * m[1][0] ) + v.z * m[2][0],
		( v.x * m[0][1] + v.y * m[1][1] ) + v.z * m[2][1],
		( v.x * m[0][2] + v.y * m[1][2] ) + v.z * m[2][2],
	};
}

inline b3Vec3 Scale( b3Vec3 v, double s )
{
	return { static_cast<float>( static_cast<double>( v.x ) * s ), static_cast<float>( static_cast<double>( v.y ) * s ), static_cast<float>( static_cast<double>( v.z ) * s ) };
}

inline b3Vec3 ScaleAdd( b3Vec3 v, double s, b3Vec3 base )
{
	return {
		static_cast<float>( static_cast<double>( v.x ) * s + static_cast<double>( base.x ) ),
		static_cast<float>( static_cast<double>( v.y ) * s + static_cast<double>( base.y ) ),
		static_cast<float>( static_cast<double>( v.z ) * s + static_cast<double>( base.z ) ),
	};
}

inline DoubleQuat_t InterpolateQuat( DoubleQuat_t a, DoubleQuat_t b, double t )
{
	double dot = ( a.w * b.w + a.z * b.z ) + ( a.y * b.y + a.x * b.x );
	double sign = 1;
	if ( !( dot > 0 ) )
	{
		dot = -dot;
		sign = -1;
	}

	if ( !( dot < static_cast<double>( 0.999f ) ) )
	{
		const DoubleQuat_t r = {
			( ( sign * b.x - a.x ) * t ) + a.x,
			( ( sign * b.y - a.y ) * t ) + a.y,
			( ( sign * b.z - a.z ) * t ) + a.z,
			( ( sign * b.w - a.w ) * t ) + a.w,
		};
		const double h = ( ( r.w * r.w + r.z * r.z ) + ( r.x * r.x + r.y * r.y ) ) * 0.5;
		double g = 1.5 - h;
		g = g + ( 0.5 - ( g * g ) * h );
		g = g + ( 0.5 - ( g * g ) * h );
		return { g * r.x, g * r.y, g * r.z, g * r.w };
	}

	const double angle = std::acos( dot );
	const double inverse = 1.0 / std::sqrt( 1.0 - dot * dot );
	const double c0 = std::sin( ( 1.0 - t ) * angle ) * inverse;
	const double c1 = ( std::sin( angle * t ) * inverse ) * sign;
	return { c1 * b.x + c0 * a.x, c1 * b.y + c0 * a.y, c1 * b.z + c0 * a.z, c1 * b.w + c0 * a.w };
}
