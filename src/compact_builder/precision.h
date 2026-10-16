#pragma once

#include "box3d/math_functions.h"

#include <bit>
#include <cmath>
#include <cstdint>

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

inline double ApproximateInverseSquareRoot( double x, int nRefinements )
{
	const double flHalf = x * 0.5;
	const int32_t nSeed = ( ( 0x7FF00000 - static_cast<int32_t>( std::bit_cast<uint64_t>( x ) >> 32 ) ) >> 1 ) + 0x1FF00000;
	double flInverse = std::bit_cast<double>( static_cast<uint64_t>( static_cast<uint32_t>( nSeed ) ) << 32 );
	for ( int i = 0; i < nRefinements; ++i )
		flInverse = ( ( 0.5 - flInverse * flInverse * flHalf ) + 1.0 ) * flInverse;
	return flInverse;
}
