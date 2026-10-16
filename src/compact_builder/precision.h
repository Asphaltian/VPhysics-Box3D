#pragma once

#include "box3d/math_functions.h"

#include <bit>
#include <cmath>
#include <cstdint>

#include <xmmintrin.h>

#if defined( VPHYSICS_GMOD )
using real_t = double;
inline constexpr real_t REAL_EPSILON = 1e-12;
inline constexpr real_t REAL_RESOLUTION = 1e-19;
#else
using real_t = float;
inline constexpr real_t REAL_EPSILON = 1e-6f;
inline constexpr real_t REAL_RESOLUTION = 1e-10f;
#endif

struct RealVec3_t
{
	real_t x, y, z;
};

inline RealVec3_t ToReal( b3Vec3 v )
{
	return { v.x, v.y, v.z };
}

inline b3Vec3 ToFloat( RealVec3_t v )
{
	return { static_cast<float>( v.x ), static_cast<float>( v.y ), static_cast<float>( v.z ) };
}

inline RealVec3_t operator+( RealVec3_t a, RealVec3_t b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

inline RealVec3_t operator-( RealVec3_t a, RealVec3_t b )
{
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}

inline RealVec3_t operator-( RealVec3_t a )
{
	return { -a.x, -a.y, -a.z };
}

inline RealVec3_t operator*( real_t s, RealVec3_t a )
{
	return { s * a.x, s * a.y, s * a.z };
}

inline real_t Dot( RealVec3_t a, RealVec3_t b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline RealVec3_t Cross( RealVec3_t a, RealVec3_t b )
{
	return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

inline real_t LengthSquared( RealVec3_t a )
{
	return Dot( a, a );
}

inline real_t Length( RealVec3_t a )
{
	return std::sqrt( LengthSquared( a ) );
}

inline real_t Component( RealVec3_t v, int nAxis )
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
	const int32_t nSeed = ( ( 0x7FF00000 - static_cast<int32_t>( std::bit_cast<uint64_t>( x ) >> 32 ) ) >> 1 ) + 0x1FF00000;
	double flInverse = std::bit_cast<double>( static_cast<uint64_t>( static_cast<uint32_t>( nSeed ) ) << 32 );
	for ( int i = 0; i < nRefinements; ++i )
		flInverse = ( ( 0.5 - flInverse * flInverse * flHalf ) + 1.0 ) * flInverse;
	return flInverse;
}

struct RealQuat_t
{
	real_t x, y, z, w;
};

struct RealMatrix3_t
{
	real_t m[3][3];
};

inline RealQuat_t ConjugateQuat( RealQuat_t q )
{
	return { -q.x, -q.y, -q.z, q.w };
}

inline RealQuat_t MultiplyQuat( RealQuat_t a, RealQuat_t b )
{
	return {
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

inline RealQuat_t NormalizeQuat( RealQuat_t q )
{
	const real_t n = ( q.w * q.w + q.z * q.z ) + ( q.x * q.x + q.y * q.y );
	if ( !( n > REAL_RESOLUTION ) )
		return q;

	const real_t r = real_t( 1 ) / std::sqrt( n );
	return { r * q.x, r * q.y, r * q.z, r * q.w };
}

inline RealQuat_t MatrixToQuat( const RealMatrix3_t &matrix )
{
	const auto &m = matrix.m;
	RealQuat_t q;
	const real_t trace = ( m[0][0] + m[1][1] ) + m[2][2];
	if ( trace > 0 )
	{
		const real_t s = std::sqrt( trace + real_t( 1 ) );
		const real_t f = real_t( 0.5 ) / s;
		q.w = s * real_t( 0.5 );
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

		real_t *const pComponents[3] = { &q.x, &q.y, &q.z };
		const real_t s = std::sqrt( ( m[i][i] - ( m[j][j] + m[k][k] ) ) + real_t( 1 ) );
		const real_t f = s != 0 ? real_t( 0.5 ) / s : s;
		*pComponents[i] = s * real_t( 0.5 );
		q.w = ( m[k][j] - m[j][k] ) * f;
		*pComponents[j] = ( m[j][i] + m[i][j] ) * f;
		*pComponents[k] = ( m[k][i] + m[i][k] ) * f;
	}

	return NormalizeQuat( q );
}

inline RealMatrix3_t QuatToMatrix( RealQuat_t q )
{
	const real_t x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
	const real_t xx = q.x * x2, yy = q.y * y2, zz = q.z * z2;
	const real_t xy = q.x * y2, xz = q.x * z2, yz = q.y * z2;
	const real_t wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;

	RealMatrix3_t matrix;
	auto &m = matrix.m;
	m[0][0] = real_t( 1 ) - ( zz + yy );
	m[1][1] = real_t( 1 ) - ( zz + xx );
	m[2][2] = real_t( 1 ) - ( yy + xx );
	m[0][1] = xy - wz;
	m[1][0] = wz + xy;
	m[0][2] = wy + xz;
	m[2][0] = xz - wy;
	m[1][2] = yz - wx;
	m[2][1] = wx + yz;
	return matrix;
}

inline RealVec3_t Rotate( const RealMatrix3_t &rotation, b3Vec3 v )
{
	const auto &m = rotation.m;
	return {
		( real_t( v.x ) * m[0][0] + real_t( v.y ) * m[0][1] ) + real_t( v.z ) * m[0][2],
		( real_t( v.x ) * m[1][0] + real_t( v.y ) * m[1][1] ) + real_t( v.z ) * m[1][2],
		( real_t( v.x ) * m[2][0] + real_t( v.y ) * m[2][1] ) + real_t( v.z ) * m[2][2],
	};
}

inline RealVec3_t InverseRotate( const RealMatrix3_t &rotation, b3Vec3 v )
{
	const auto &m = rotation.m;
	return {
		( real_t( v.x ) * m[0][0] + real_t( v.y ) * m[1][0] ) + real_t( v.z ) * m[2][0],
		( real_t( v.x ) * m[0][1] + real_t( v.y ) * m[1][1] ) + real_t( v.z ) * m[2][1],
		( real_t( v.x ) * m[0][2] + real_t( v.y ) * m[1][2] ) + real_t( v.z ) * m[2][2],
	};
}

inline RealVec3_t InverseRotate( const RealMatrix3_t &rotation, RealVec3_t v )
{
	const auto &m = rotation.m;
	return {
		( v.x * m[0][0] + v.y * m[1][0] ) + v.z * m[2][0],
		( v.x * m[0][1] + v.y * m[1][1] ) + v.z * m[2][1],
		( v.x * m[0][2] + v.y * m[1][2] ) + v.z * m[2][2],
	};
}

inline b3Vec3 Scale( b3Vec3 v, real_t s )
{
	return { static_cast<float>( real_t( v.x ) * s ), static_cast<float>( real_t( v.y ) * s ), static_cast<float>( real_t( v.z ) * s ) };
}

inline b3Vec3 ScaleAdd( b3Vec3 v, real_t s, b3Vec3 base )
{
	return {
		static_cast<float>( real_t( v.x ) * s + real_t( base.x ) ),
		static_cast<float>( real_t( v.y ) * s + real_t( base.y ) ),
		static_cast<float>( real_t( v.z ) * s + real_t( base.z ) ),
	};
}

inline RealQuat_t InterpolateQuat( RealQuat_t a, RealQuat_t b, real_t t )
{
	real_t dot = ( a.w * b.w + a.z * b.z ) + ( a.y * b.y + a.x * b.x );
	real_t sign = 1;
	if ( !( dot > 0 ) )
	{
		dot = -dot;
		sign = -1;
	}

	if ( !( dot < real_t( 0.999f ) ) )
	{
		const RealQuat_t r = {
			( ( sign * b.x - a.x ) * t ) + a.x,
			( ( sign * b.y - a.y ) * t ) + a.y,
			( ( sign * b.z - a.z ) * t ) + a.z,
			( ( sign * b.w - a.w ) * t ) + a.w,
		};
		const real_t h = ( ( r.w * r.w + r.z * r.z ) + ( r.x * r.x + r.y * r.y ) ) * real_t( 0.5 );
		real_t g = real_t( 1.5 ) - h;
		g = g + ( real_t( 0.5 ) - ( g * g ) * h );
		g = g + ( real_t( 0.5 ) - ( g * g ) * h );
		return { g * r.x, g * r.y, g * r.z, g * r.w };
	}

	const real_t angle = std::acos( dot );
	const real_t inverse = real_t( 1 ) / std::sqrt( real_t( 1 ) - dot * dot );
	const real_t c0 = std::sin( ( real_t( 1 ) - t ) * angle ) * inverse;
	const real_t c1 = ( std::sin( angle * t ) * inverse ) * sign;
	return { c1 * b.x + c0 * a.x, c1 * b.y + c0 * a.y, c1 * b.z + c0 * a.z, c1 * b.w + c0 * a.w };
}
