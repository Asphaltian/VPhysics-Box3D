#include "conversion.h"

#include <cmath>

#include <emmintrin.h>

static constexpr float HALF_PI = 1.5707964f;
static constexpr float SMALL_ROLL_ANGLE = 1.2207031e-4f;
static constexpr double GIMBAL_DISTANCE = 0.001;

static __m128 BroadcastBits( uint32_t nBits )
{
	return _mm_castsi128_ps( _mm_set1_epi32( static_cast<int>( nBits ) ) );
}

static __m128 Sine( __m128 x )
{
	const __m128 absoluteMask = BroadcastBits( 0x7FFFFFFF );
	const __m128 roundingShift = BroadcastBits( 0x4B400000 );
	const __m128 absolute = _mm_and_ps( x, absoluteMask );
	const __m128 sign = _mm_andnot_ps( absoluteMask, x );

	const __m128 rounded = _mm_add_ps( _mm_mul_ps( absolute, BroadcastBits( 0x3EA2F983 ) ), roundingShift );
	const __m128 n = _mm_sub_ps( rounded, roundingShift );
	__m128 r = _mm_sub_ps( absolute, _mm_mul_ps( n, BroadcastBits( 0x40490000 ) ) );
	r = _mm_sub_ps( r, _mm_mul_ps( n, BroadcastBits( 0x3A7DA000 ) ) );
	r = _mm_sub_ps( r, _mm_mul_ps( n, BroadcastBits( 0x34222000 ) ) );
	r = _mm_sub_ps( r, _mm_mul_ps( n, BroadcastBits( 0x2CB4611A ) ) );

	const __m128 r2 = _mm_mul_ps( r, r );
	r = _mm_xor_ps( r, _mm_castsi128_ps( _mm_slli_epi32( _mm_castps_si128( rounded ), 31 ) ) );

	__m128 polynomial = _mm_add_ps( _mm_mul_ps( BroadcastBits( 0x362EDEF8 ), r2 ), BroadcastBits( 0xB94FB7FF ) );
	polynomial = _mm_add_ps( _mm_mul_ps( polynomial, r2 ), BroadcastBits( 0x3C088766 ) );
	polynomial = _mm_add_ps( _mm_mul_ps( polynomial, r2 ), BroadcastBits( 0xBE2AAAA6 ) );
	const __m128 result = _mm_xor_ps( _mm_add_ps( r, _mm_mul_ps( _mm_mul_ps( r2, polynomial ), r ) ), sign );

	const int nLargeMask = _mm_movemask_ps( _mm_cmpnle_ps( absolute, BroadcastBits( 0x461C4000 ) ) );
	if ( nLargeMask == 0 )
		return result;

	alignas( 16 ) float inputs[4];
	alignas( 16 ) float outputs[4];
	_mm_store_ps( inputs, x );
	_mm_store_ps( outputs, result );
	for ( int nLane = 0; nLane < 4; ++nLane )
	{
		if ( nLargeMask & ( 1 << nLane ) )
			outputs[nLane] = std::sin( inputs[nLane] );
	}
	return _mm_load_ps( outputs );
}

static void GetAngleSinCos( const QAngle &angles, float &sy, float &cy, float &sp, float &cp, float &sr, float &cr )
{
	const float flYaw = angles.y * DEGREES_TO_RADIANS;
	const float flPitch = angles.x * DEGREES_TO_RADIANS;
	const float flRoll = angles.z * DEGREES_TO_RADIANS;

	alignas( 16 ) float yawPitch[4];
	_mm_store_ps( yawPitch, Sine( _mm_add_ps( _mm_setr_ps( flYaw, flYaw, flPitch, flPitch ), _mm_setr_ps( 0.0f, HALF_PI, 0.0f, HALF_PI ) ) ) );
	sy = yawPitch[0];
	cy = yawPitch[1];
	sp = yawPitch[2];
	cp = yawPitch[3];

	const float flAbsoluteRoll = std::fabs( flRoll );
	sr = flRoll;
	cr = 1.0f - ( flAbsoluteRoll * flAbsoluteRoll ) * 0.5f;
	if ( !( flAbsoluteRoll < SMALL_ROLL_ANGLE ) )
	{
		alignas( 16 ) float roll[4];
		_mm_store_ps( roll, Sine( _mm_setr_ps( flRoll, flAbsoluteRoll + HALF_PI, 0.0f, 0.0f ) ) );
		sr = roll[0];
		cr = roll[1];
	}
}

void AngleVectors( const QAngle &angles, b3Vec3 &forward, b3Vec3 &right, b3Vec3 &up )
{
	float sy, cy, sp, cp, sr, cr;
	GetAngleSinCos( angles, sy, cy, sp, cp, sr, cr );
	forward = { cp * cy, cp * sy, -sp };
	right = { cr * sy - ( sr * sp ) * cy, -( cr * cy ) - ( sr * sp ) * sy, -( sr * cp ) };
	up = { ( cr * sp ) * cy + sr * sy, ( cr * sp ) * sy - sr * cy, cr * cp };
}

void AngleMatrix( const QAngle &angles, const Vector &position, matrix3x4_t &matrix )
{
	float sy, cy, sp, cp, sr, cr;
	GetAngleSinCos( angles, sy, cy, sp, cp, sr, cr );

	matrix[0][0] = cp * cy;
	matrix[1][0] = cp * sy;
	matrix[2][0] = -sp;

	const float crcy = cr * cy;
	const float crsy = cr * sy;
	const float srcy = sr * cy;
	const float srsy = sr * sy;
	matrix[0][1] = sp * srcy - crsy;
	matrix[1][1] = sp * srsy + crcy;
	matrix[2][1] = sr * cp;

	matrix[0][2] = sp * crcy + srsy;
	matrix[1][2] = sp * crsy - srcy;
	matrix[2][2] = cr * cp;

	matrix[0][3] = position.x;
	matrix[1][3] = position.y;
	matrix[2][3] = position.z;
}

void VectorTransform( const float *in1, const matrix3x4_t &in2, float *out )
{
	for ( int i = 0; i < 3; ++i )
		out[i] = ( ( in1[0] * in2[i][0] + in1[1] * in2[i][1] ) + in1[2] * in2[i][2] ) + in2[i][3];
}

DoubleQuat_t AnglesToIVP( const QAngle &angles )
{
	b3Vec3 forward, right, up;
	AngleVectors( angles, forward, right, up );

	const DoubleMatrix3_t rotation = { {
		{ forward.x, -up.x, -right.x },
		{ -forward.z, up.z, right.z },
		{ forward.y, -up.y, -right.y },
	} };
	return MatrixToQuat( rotation );
}

DoubleQuat_t MatrixToIVP( const matrix3x4_t &matrix, DoubleVec3_t &origin )
{
	const DoubleMatrix3_t rotation = { {
		{ matrix[0][0], -matrix[0][2], matrix[0][1] },
		{ -matrix[2][0], matrix[2][2], -matrix[2][1] },
		{ matrix[1][0], -matrix[1][2], matrix[1][1] },
	} };
	origin = ToDouble( SourceToIVP( b3Vec3{ matrix[0][3], matrix[1][3], matrix[2][3] } ) );
	return MatrixToQuat( rotation );
}

QAngle IVPToAngles( const DoubleMatrix3_t &rotation )
{
	float m[3][3];
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
			m[i][j] = static_cast<float>( rotation.m[i][j] );
	}

	const float flXYDistance = std::sqrt( m[2][0] * m[2][0] + m[0][0] * m[0][0] );
	QAngle angles;
	angles.x = static_cast<float>( std::atan2( static_cast<double>( m[1][0] ), static_cast<double>( flXYDistance ) ) ) * RADIANS_TO_DEGREES;
	if ( static_cast<double>( flXYDistance ) > GIMBAL_DISTANCE )
	{
		angles.z = static_cast<float>( std::atan2( static_cast<double>( m[1][2] ), static_cast<double>( -m[1][1] ) ) ) * RADIANS_TO_DEGREES + 180.0f;
		angles.y = static_cast<float>( std::atan2( static_cast<double>( m[2][0] ), static_cast<double>( m[0][0] ) ) ) * RADIANS_TO_DEGREES;
	}
	else
	{
		angles.z = 180.0f;
		angles.y = static_cast<float>( std::atan2( static_cast<double>( m[0][2] ), static_cast<double>( -m[2][2] ) ) ) * RADIANS_TO_DEGREES;
	}
	return angles;
}

void IVPToMatrix( const DoubleMatrix3_t &rotation, DoubleVec3_t origin, matrix3x4_t &matrix )
{
	const auto m = [&]( int i, int j ) { return static_cast<float>( rotation.m[i][j] ); };
	matrix[0][0] = m( 0, 0 );
	matrix[0][1] = m( 0, 2 );
	matrix[0][2] = -m( 0, 1 );
	matrix[1][0] = m( 2, 0 );
	matrix[1][1] = m( 2, 2 );
	matrix[1][2] = -m( 2, 1 );
	matrix[2][0] = -m( 1, 0 );
	matrix[2][1] = -m( 1, 2 );
	matrix[2][2] = m( 1, 1 );
	matrix[0][3] = static_cast<float>( static_cast<double>( IVP_TO_SOURCE ) * origin.x );
	matrix[1][3] = static_cast<float>( static_cast<double>( IVP_TO_SOURCE ) * origin.z );
	matrix[2][3] = -static_cast<float>( static_cast<double>( IVP_TO_SOURCE ) * origin.y );
}

QAngle MatrixAngles( const matrix3x4_t &matrix )
{
	const float flXYDistance = std::sqrt( matrix[0][0] * matrix[0][0] + matrix[1][0] * matrix[1][0] );
	if ( flXYDistance > 0.001f )
		return QAngle( RADIANS_TO_DEGREES * std::atan2( -matrix[2][0], flXYDistance ), RADIANS_TO_DEGREES * std::atan2( matrix[1][0], matrix[0][0] ), RADIANS_TO_DEGREES * std::atan2( matrix[2][1], matrix[2][2] ) );

	return QAngle( RADIANS_TO_DEGREES * std::atan2( -matrix[2][0], flXYDistance ), RADIANS_TO_DEGREES * std::atan2( -matrix[0][1], matrix[1][1] ), 0.0f );
}
