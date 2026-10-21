#include "physics_constraint_solver.h"

#include "conversion.h"
#include "physics_object.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <numbers>

static constexpr float MIN_SWING_JACOBIAN = 0.1f;
static constexpr float SLACK_LOOK_AHEAD = 0.01f;
static constexpr float LENGTH_BIAS = 1e-18f;
static constexpr float LENGTH_VELOCITY_GAIN = 0.5f;
static constexpr float MAX_ROTATION_STEP = 0.997f;
static constexpr float HALF_PI = 0.5f * std::numbers::pi_v<float>;
static constexpr float QUARTER_PI = 0.25f * std::numbers::pi_v<float>;
static constexpr float ARC_TANGENT_C1 = 0.16591105f;
static constexpr float ARC_TANGENT_C2 = -0.33080792f;
static constexpr float ARC_TANGENT_C3 = 0.99995309f;

static FloatMatrix_t ToFloat( const DoubleMatrix3_t &rotation )
{
	FloatMatrix_t matrix;
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
			matrix.m[i][j] = static_cast<float>( rotation.m[i][j] );
	}
	return matrix;
}

static float Sum3( b3Vec3 v )
{
	return ( v.y + v.x ) + v.z;
}

static b3Vec3 Multiply( b3Vec3 a, b3Vec3 b )
{
	return { a.x * b.x, a.y * b.y, a.z * b.z };
}

static b3Vec3 Negate( b3Vec3 v )
{
	return { 0.0f - v.x, 0.0f - v.y, 0.0f - v.z };
}

static b3Vec3 RotateColumns( const FloatMatrix_t &rotation, b3Vec3 u )
{
	const auto &m = rotation.m;
	return {
		( u.z * m[0][2] + u.y * m[0][1] ) + u.x * m[0][0],
		( u.z * m[1][2] + u.y * m[1][1] ) + u.x * m[1][0],
		( u.z * m[2][2] + u.y * m[2][1] ) + u.x * m[2][0],
	};
}

static b3Vec3 InverseRotateRows( const FloatMatrix_t &rotation, b3Vec3 v )
{
	const auto &m = rotation.m;
	return {
		( v.z * m[2][0] + v.y * m[1][0] ) + v.x * m[0][0],
		( v.z * m[2][1] + v.y * m[1][1] ) + v.x * m[0][1],
		( v.z * m[2][2] + v.y * m[1][2] ) + v.x * m[0][2],
	};
}

static float Positive( float x )
{
	return x > 0.0f ? x : 0.0f;
}

static float Negative( float x )
{
	return x < 0.0f ? x : 0.0f;
}

static float FastArcTangent( float x, float y )
{
	const float flAbsoluteX = std::fabs( x ), flAbsoluteY = std::fabs( y );
	const bool bSteep = flAbsoluteY < flAbsoluteX;
	const float flHigh = bSteep ? flAbsoluteX : flAbsoluteY;
	const float flLow = bSteep ? flAbsoluteY : flAbsoluteX;
	const bool bValid = flHigh > FLT_EPSILON;
	const float flRatio = bValid ? FastReciprocal( flHigh ) * flLow : 0.0f;
	const float flDiagonal = FastReciprocal( flAbsoluteX + flAbsoluteY ) * ( flAbsoluteX - flAbsoluteY );
	const bool bNearDiagonal = std::fabs( flDiagonal ) < flRatio && bValid;
	const float v = bNearDiagonal ? flDiagonal : flRatio;
	const float z = v * v;
	const float p = ( ( ( z * ARC_TANGENT_C1 + ARC_TANGENT_C2 ) * z ) + ARC_TANGENT_C3 ) * v;
	float flAngle = bNearDiagonal ? p + QUARTER_PI : ( bSteep ? HALF_PI - p : p );
	if ( std::signbit( x ) != std::signbit( y ) )
		flAngle = -flAngle;
	return flAngle + ( y < 0.0f ? std::copysign( PI, x ) : 0.0f );
}

static b3Vec3 GetAnchorPoint( const DoubleMatrix3_t &rotation, DoubleVec3_t position, b3Vec3 shift, b3Vec3 anchor )
{
	return ToFloat( Rotate( rotation, anchor ) + ( Rotate( rotation, shift ) + position ) );
}

static b3Vec3 PredictAnchor( const CPhysicsObject *pObject, const ConstraintStep_t &step, b3Vec3 anchor )
{
	const b3Vec3 w = pObject->GetCoreAngularVelocity();
	const float flHalfStep = step.m_flTimestep * 0.5f;
	b3Vec3 d = { std::sin( flHalfStep * w.x ), std::sin( flHalfStep * w.y ), std::sin( flHalfStep * w.z ) };
	float flLengthSquared = ( d.x * d.x + d.y * d.y ) + d.z * d.z;
	if ( flLengthSquared > 1.0f )
	{
		d = b3MulSV( MAX_ROTATION_STEP / std::sqrt( flLengthSquared ), d );
		flLengthSquared = ( d.x * d.x + d.y * d.y ) + d.z * d.z;
	}

	const DoubleQuat_t delta = { d.x, d.y, d.z, std::sqrt( 1.0f - flLengthSquared ) };
	const DoubleMatrix3_t rotation = QuatToMatrix( NormalizeQuat( MultiplyQuat( pObject->GetNextRotation(), delta ) ) );
	const b3Vec3 s = pObject->GetCoreVelocity();
	const DoubleVec3_t position = pObject->GetCorePosition();
	const double dt = step.m_flTimestep;
	return GetAnchorPoint( rotation, { dt * s.x + position.x, dt * s.y + position.y, dt * s.z + position.z }, pObject->GetShift(), anchor );
}

static b3Vec3 ToCoreAxis( const DoubleMatrix3_t &rotation, b3Vec3 x )
{
	const auto &m = rotation.m;
	const double flX = x.x, flY = x.y, flZ = x.z;
	return {
		static_cast<float>( ( flY * m[1][0] + flX * m[0][0] ) + flZ * m[2][0] ),
		static_cast<float>( ( flY * m[1][1] + flX * m[0][1] ) + flZ * m[2][1] ),
		static_cast<float>( ( flY * m[1][2] + flX * m[0][2] ) + flZ * m[2][2] ),
	};
}

static float MeasureLength( b3Vec3 delta )
{
	return std::sqrt( ( delta.y * delta.y + delta.x * delta.x ) + delta.z * delta.z );
}

static float GetRowVelocity( const ConstraintBody_t bodies[2], const b3Vec3 linear[2], const b3Vec3 angular[2] )
{
	float flVelocity = 0.0f;
	for ( int i = 0; i < 2; ++i )
	{
		const b3Vec3 s = bodies[i].m_pObject->GetCoreVelocity();
		const b3Vec3 w = bodies[i].m_pObject->GetCoreAngularVelocity();
		flVelocity = ( Sum3( Multiply( linear[i], s ) ) + flVelocity ) + Sum3( Multiply( angular[i], w ) );
	}
	return flVelocity;
}

void CPulleySolver::Initialize( const b3Vec3 anchors[2], const b3Vec3 pulleys[2], float flLength, float flGearRatio, float flStrength, bool bRigid )
{
	for ( int i = 0; i < 2; ++i )
	{
		m_Anchors[i] = anchors[i];
		m_Pulleys[i] = pulleys[i];
	}
	m_flLength = flLength;
	m_flGearRatio = flGearRatio;
	m_flStrength = flStrength;
	m_bRigid = bRigid;
}

float CPulleySolver::MeasureError( const b3Vec3 points[2], b3Vec3 directions[2] ) const
{
	float flLengths[2];
	for ( int i = 0; i < 2; ++i )
	{
		b3Vec3 delta = b3Sub( m_Pulleys[i], points[i] );
		if ( i == 1 )
			delta = b3MulSV( m_flGearRatio, delta );
		flLengths[i] = std::sqrt( ( delta.y * delta.y + delta.x * delta.x ) + delta.z * delta.z );
		if ( directions )
			directions[i] = b3MulSV( 1.0f / ( flLengths[i] + LENGTH_BIAS ), delta );
	}
	return ( flLengths[0] + flLengths[1] ) - m_flLength;
}

bool CPulleySolver::IsSlackAhead( ConstraintBody_t bodies[2], const ConstraintStep_t &step ) const
{
	const b3Vec3 points[2] = { PredictAnchor( bodies[0].m_pObject, step, m_Anchors[0] ), PredictAnchor( bodies[1].m_pObject, step, m_Anchors[1] ) };
	return MeasureError( points, nullptr ) < m_flLength * SLACK_LOOK_AHEAD;
}

void CPulleySolver::Apply( ConstraintBody_t bodies[2], float flImpulse ) const
{
	for ( int i = 0; i < 2; ++i )
	{
		const ConstraintBody_t &body = bodies[i];
		if ( !body.m_bDynamic )
			continue;

		const float flLinear = flImpulse * body.m_flInverseMass;
		const b3Vec3 s = body.m_pObject->GetCoreVelocity();
		const b3Vec3 w = body.m_pObject->GetCoreAngularVelocity();
		body.m_pObject->SetCoreVelocity( { flLinear * m_Linear[i].x + s.x, flLinear * m_Linear[i].y + s.y, flLinear * m_Linear[i].z + s.z } );
		body.m_pObject->SetCoreAngularVelocity( { flImpulse * m_Response[i].x + w.x, flImpulse * m_Response[i].y + w.y, flImpulse * m_Response[i].z + w.z } );
	}
}

void CPulleySolver::Setup( ConstraintBody_t bodies[2], const ConstraintStep_t &step )
{
	DoubleMatrix3_t rotations[2];
	DoubleVec3_t positions[2];
	b3Vec3 points[2];
	for ( int i = 0; i < 2; ++i )
	{
		const CPhysicsObject *pObject = bodies[i].m_pObject;
		rotations[i] = pObject->GetCoreRotation();
		positions[i] = pObject->GetCorePosition();
		points[i] = GetAnchorPoint( rotations[i], positions[i], pObject->GetShift(), m_Anchors[i] );
	}

	m_flError = MeasureError( points, m_Linear );
	m_bActive = m_bRigid || m_flError >= 0.0f || !IsSlackAhead( bodies, step );
	if ( !m_bActive )
		return;

	float flVelocity = 0.0f, flInverseEffectiveMass = 0.0f;
	for ( int i = 0; i < 2; ++i )
	{
		const FloatMatrix_t rotation = ToFloat( rotations[i] );
		const b3Vec3 r = ToFloat( ToDouble( points[i] ) - positions[i] );
		const b3Vec3 n = m_Linear[i];
		const b3Vec3 x = { n.z * r.y - n.y * r.z, n.x * r.z - n.z * r.x, n.y * r.x - n.x * r.y };
		const auto &m = rotation.m;
		m_Angular[i] = {
			( x.y * m[1][0] + x.x * m[0][0] ) + x.z * m[2][0],
			( x.y * m[1][1] + x.x * m[0][1] ) + x.z * m[2][1],
			( x.y * m[1][2] + x.x * m[0][2] ) + x.z * m[2][2],
		};

		const ConstraintBody_t &body = bodies[i];
		if ( !body.m_bDynamic )
		{
			m_Response[i] = b3Vec3_zero;
			continue;
		}

		const b3Vec3 &a = m_Angular[i];
		const b3Vec3 response = Multiply( a, body.m_InverseInertia );
		const b3Vec3 s = body.m_pObject->GetCoreVelocity();
		const b3Vec3 w = body.m_pObject->GetCoreAngularVelocity();
		m_Response[i] = response;
		flVelocity = ( ( n.z * s.z + a.y * w.y ) + ( a.x * w.x + a.z * w.z ) ) + ( ( n.y * s.y + n.x * s.x ) + flVelocity );
		flInverseEffectiveMass = ( ( ( n.y * n.y + n.x * n.x ) + n.z * n.z ) * body.m_flInverseMass + a.y * response.y ) + ( ( a.z * response.z + flInverseEffectiveMass ) + a.x * response.x );
	}

	m_flEffectiveMass = 1.0f / flInverseEffectiveMass;
	Apply( bodies, ( ( 1.0f * m_flStrength ) * ( step.m_flInverseTimestep * m_flError ) - ( 1.0f * 1.0f ) * flVelocity ) * m_flEffectiveMass );
}

void CPulleySolver::Solve( ConstraintBody_t bodies[2], float flAlpha, float flBeta, const ConstraintStep_t &step )
{
	if ( !m_bActive )
		return;

	float flVelocity = 0.0f;
	for ( int i = 0; i < 2; ++i )
	{
		const b3Vec3 s = bodies[i].m_pObject->GetCoreVelocity();
		const b3Vec3 w = bodies[i].m_pObject->GetCoreAngularVelocity();
		const b3Vec3 &n = m_Linear[i];
		const b3Vec3 &a = m_Angular[i];
		flVelocity = ( ( n.y * s.y + n.x * s.x ) + ( n.z * s.z + flVelocity ) ) + ( ( a.x * w.x + a.z * w.z ) + a.y * w.y );
	}
	Apply( bodies, ( ( m_flError * step.m_flInverseTimestep ) * ( flAlpha * m_flStrength ) - ( flBeta * 1.0f ) * flVelocity ) * m_flEffectiveMass );
}

void CLengthSolver::Initialize( const b3Vec3 anchors[2], float flMinLength, float flMaxLength, float flStrength )
{
	m_Anchors[0] = anchors[0];
	m_Anchors[1] = anchors[1];
	m_flMinLength = flMinLength;
	m_flMaxLength = flMaxLength;
	m_flStrength = flStrength;
}

bool CLengthSolver::IsOutsideAhead( ConstraintBody_t bodies[2], const ConstraintStep_t &step ) const
{
	const float flLength = MeasureLength( b3Sub( PredictAnchor( bodies[1].m_pObject, step, m_Anchors[1] ), PredictAnchor( bodies[0].m_pObject, step, m_Anchors[0] ) ) );
	return flLength < m_flMinLength || flLength > m_flMaxLength;
}

void CLengthSolver::Apply( ConstraintBody_t bodies[2], float flImpulse ) const
{
	for ( int i = 0; i < 2; ++i )
	{
		const ConstraintBody_t &body = bodies[i];
		if ( !body.m_bDynamic )
			continue;

		const float flLinear = flImpulse * body.m_flInverseMass;
		const b3Vec3 s = body.m_pObject->GetCoreVelocity();
		const b3Vec3 w = body.m_pObject->GetCoreAngularVelocity();
		body.m_pObject->SetCoreVelocity( { flLinear * m_Linear[i].x + s.x, flLinear * m_Linear[i].y + s.y, flLinear * m_Linear[i].z + s.z } );
		body.m_pObject->SetCoreAngularVelocity( { flImpulse * m_Response[i].x + w.x, flImpulse * m_Response[i].y + w.y, flImpulse * m_Response[i].z + w.z } );
	}
}

void CLengthSolver::Setup( ConstraintBody_t bodies[2], const ConstraintStep_t &step )
{
	DoubleMatrix3_t rotations[2];
	b3Vec3 centers[2], points[2];
	for ( int i = 0; i < 2; ++i )
	{
		const CPhysicsObject *pObject = bodies[i].m_pObject;
		rotations[i] = pObject->GetCoreRotation();
		const DoubleVec3_t position = pObject->GetCorePosition();
		centers[i] = ToFloat( position );
		points[i] = GetAnchorPoint( rotations[i], position, pObject->GetShift(), m_Anchors[i] );
	}

	const b3Vec3 delta = b3Sub( points[1], points[0] );
	const float flLength = MeasureLength( delta );
	m_flError = flLength < m_flMinLength ? flLength - m_flMinLength : ( flLength > m_flMaxLength ? flLength - m_flMaxLength : 0.0f );
	m_bActive = m_flMinLength == m_flMaxLength || m_flError != 0.0f || IsOutsideAhead( bodies, step );
	if ( !m_bActive )
		return;

	const float flInverseLength = 1.0f / ( flLength + LENGTH_BIAS );
	m_Linear[0] = { delta.x * flInverseLength, delta.y * flInverseLength, delta.z * flInverseLength };
	m_Linear[1] = Negate( m_Linear[0] );
	const b3Vec3 levers[2] = { b3Sub( points[0], centers[0] ), b3Sub( centers[1], points[0] ) };
	const b3Vec3 n = m_Linear[0];
	float flInverseEffectiveMass = 0.0f;
	for ( int i = 0; i < 2; ++i )
	{
		const b3Vec3 &r = levers[i];
		m_Angular[i] = ToCoreAxis( rotations[i], { n.z * r.y - r.z * n.y, n.x * r.z - r.x * n.z, n.y * r.x - r.y * n.x } );

		const ConstraintBody_t &body = bodies[i];
		if ( !body.m_bDynamic )
		{
			m_Response[i] = b3Vec3_zero;
			continue;
		}

		m_Response[i] = Multiply( m_Angular[i], body.m_InverseInertia );
		flInverseEffectiveMass = ( Sum3( Multiply( m_Linear[i], m_Linear[i] ) ) * body.m_flInverseMass + flInverseEffectiveMass ) + Sum3( Multiply( m_Angular[i], m_Response[i] ) );
	}

	m_flEffectiveMass = 1.0f / flInverseEffectiveMass;
	Solve( bodies, 1.0f, 1.0f, step );
}

void CLengthSolver::Solve( ConstraintBody_t bodies[2], float flAlpha, float flBeta, const ConstraintStep_t &step )
{
	if ( !m_bActive )
		return;

	const float flVelocity = GetRowVelocity( bodies, m_Linear, m_Angular );
	Apply( bodies, ( ( ( m_flStrength * m_flError ) * flAlpha ) * step.m_flInverseTimestep - ( flBeta * LENGTH_VELOCITY_GAIN ) * flVelocity ) * m_flEffectiveMass );
}

void CSlidingSolver::Initialize( b3Vec3 axis, float flWeight, float flLower, float flUpper, bool bLimited, float flReference )
{
	m_Axis = axis;
	m_flWeight = flWeight;
	m_flLower = flLower;
	m_flUpper = flUpper;
	m_bLimited = bLimited;
	m_flReference = flReference;
}

void CSlidingSolver::SetMotor( float flTargetSpeed, float flMaxForce, bool bActive )
{
	m_flTargetSpeed = flTargetSpeed;
	m_flMaxForce = flMaxForce;
	m_bMotor = bActive;
}

void CSlidingSolver::Solve( ConstraintBody_t bodies[2], float flAlpha, float flBeta, const ConstraintStep_t &step )
{
	float flVelocity = GetRowVelocity( bodies, m_Linear, m_Angular );
	float flImpulse = 0.0f;
	if ( m_bMotor )
	{
		const float flError = m_flPosition - m_flReference;
		const float flReference = m_flTargetSpeed * step.m_flTimestep + m_flReference;
		m_flReference = flReference;
		flImpulse = ( ( ( flError * step.m_flInverseTimestep ) * flAlpha ) * SERVO_GAIN - flBeta * flVelocity ) * m_flEffectiveMass;
		if ( m_flMaxForce < step.m_flInverseTimestep * std::fabs( flImpulse ) )
		{
			const float flScale = ( m_flMaxForce / std::fabs( flImpulse ) ) * step.m_flTimestep;
			flImpulse *= flScale;
			m_flReference = flError * ( 1.0f - flScale ) + flReference;
		}
		flVelocity += m_flInverseEffectiveMass * flImpulse;
	}

	if ( m_bLimited )
	{
		const float flPredicted = m_flPosition - flVelocity * step.m_flTimestep;
		if ( flPredicted > m_flUpper )
			flImpulse += ( ( flPredicted - m_flUpper ) * flAlpha ) * m_flEffectiveMass * step.m_flInverseTimestep;
		else if ( flPredicted < m_flLower )
			flImpulse += ( ( flPredicted - m_flLower ) * flAlpha ) * m_flEffectiveMass * step.m_flInverseTimestep;
	}

	if ( flImpulse == 0.0f )
		return;

	for ( int i = 0; i < 2; ++i )
	{
		const ConstraintBody_t &body = bodies[i];
		if ( !body.m_bDynamic )
			continue;

		const float flLinear = flImpulse * body.m_flInverseMass;
		const b3Vec3 s = body.m_pObject->GetCoreVelocity();
		body.m_pObject->SetCoreVelocity( { flLinear * m_Linear[i].x + s.x, flLinear * m_Linear[i].y + s.y, flLinear * m_Linear[i].z + s.z } );
	}
}

void CSlidingSolver::Setup( ConstraintBody_t bodies[2], const ConstraintStep_t &step, float flPosition )
{
	DoubleMatrix3_t rotations[2];
	b3Vec3 centers[2];
	for ( int i = 0; i < 2; ++i )
	{
		rotations[i] = bodies[i].m_pObject->GetCoreRotation();
		centers[i] = ToFloat( bodies[i].m_pObject->GetCorePosition() );
	}

	const b3Vec3 n = ToFloat( Rotate( rotations[0], m_Axis ) );
	const float flComplement = 1.0f - m_flWeight;
	const b3Vec3 anchor = { flComplement * centers[0].x + m_flWeight * centers[1].x, flComplement * centers[0].y + m_flWeight * centers[1].y, flComplement * centers[0].z + m_flWeight * centers[1].z };
	const b3Vec3 levers[2] = { b3Sub( anchor, centers[0] ), b3Sub( centers[1], anchor ) };
	m_Linear[0] = n;
	m_Linear[1] = Negate( n );
	m_flPosition = flPosition;
	m_flInverseEffectiveMass = 0.0f;
	for ( int i = 0; i < 2; ++i )
	{
		const b3Vec3 &r = levers[i];
		m_Angular[i] = ToCoreAxis( rotations[i], { n.z * r.y - r.z * n.y, n.x * r.z - r.x * n.z, n.y * r.x - r.y * n.x } );

		const ConstraintBody_t &body = bodies[i];
		if ( !body.m_bDynamic )
			continue;

		const b3Vec3 response = Multiply( m_Angular[i], body.m_InverseInertia );
		m_flInverseEffectiveMass = ( Sum3( Multiply( m_Linear[i], m_Linear[i] ) ) * body.m_flInverseMass + m_flInverseEffectiveMass ) + Sum3( Multiply( m_Angular[i], response ) );
	}

	m_flEffectiveMass = 1.0f / m_flInverseEffectiveMass;
	Solve( bodies, 1.0f, 1.0f, step );
}

void CRagdollSolver::SetAxes( int nBody, b3Vec3 twist, b3Vec3 cone )
{
	m_Twist[nBody] = twist;
	m_Cone[nBody] = cone;
}

float CRagdollSolver::GetRowVelocity( const ConstraintBody_t bodies[2], const Row_t &row )
{
	return Sum3( Multiply( row.m_Jacobian[0], bodies[0].m_pObject->GetCoreAngularVelocity() ) ) + Sum3( Multiply( row.m_Jacobian[1], bodies[1].m_pObject->GetCoreAngularVelocity() ) );
}

void CRagdollSolver::BuildRow( ConstraintBody_t bodies[2], const FloatMatrix_t rotations[2], Row_t &row, b3Vec3 axis ) const
{
	row.m_Jacobian[0] = InverseRotateRows( rotations[0], axis );
	row.m_Jacobian[1] = InverseRotateRows( rotations[1], Negate( axis ) );

	float flInverseEffectiveMass[2] = { 0.0f, 0.0f };
	for ( int i = 0; i < 2; ++i )
	{
		row.m_Response[i] = b3Vec3_zero;
		if ( !bodies[i].m_bDynamic )
			continue;

		row.m_Response[i] = Multiply( row.m_Jacobian[i], bodies[i].m_InverseInertia );
		flInverseEffectiveMass[i] = Sum3( Multiply( row.m_Response[i], row.m_Jacobian[i] ) );
	}

	row.m_flInverseEffectiveMass = flInverseEffectiveMass[0] + flInverseEffectiveMass[1];
	row.m_flEffectiveMass = row.m_flInverseEffectiveMass > FLT_EPSILON ? FastReciprocal( row.m_flInverseEffectiveMass ) : 0.0f;
	row.m_bEnabled = true;
}

void CRagdollSolver::ApplyRow( ConstraintBody_t bodies[2], const Row_t &row, float flImpulse )
{
	for ( int i = 0; i < 2; ++i )
	{
		if ( !bodies[i].m_bDynamic )
			continue;

		const b3Vec3 w = bodies[i].m_pObject->GetCoreAngularVelocity();
		const b3Vec3 &response = row.m_Response[i];
		bodies[i].m_pObject->SetCoreAngularVelocity( { flImpulse * response.x + w.x, flImpulse * response.y + w.y, flImpulse * response.z + w.z } );
	}
}

void CRagdollSolver::SolveTwist( ConstraintBody_t bodies[2], float flAlpha, float flBeta, const ConstraintStep_t &step )
{
	Row_t &row = m_Rows[SLOT_TWIST];
	Limit_t &limit = m_Limits[SLOT_TWIST];
	const float flVelocity = GetRowVelocity( bodies, row );
	const float flGainA = m_flHalfSwingCosine * flAlpha;
	const float flGainB = m_flHalfSwingCosine * flBeta;

	float flImpulse = 0.0f;
	if ( limit.m_bFriction )
	{
		float flDelta = limit.m_flReference - row.m_flMeasure;
		const float flWrap = flDelta > PI ? TWO_PI : ( flDelta < -PI ? -TWO_PI : 0.0f );
		flDelta -= flWrap;
		row.m_flMeasure += flWrap;
		flImpulse = ( ( ( flGainA * SERVO_GAIN ) * flDelta ) * step.m_flInverseTimestep - flGainB * flVelocity ) * row.m_flEffectiveMass;
		const float flRatio = std::fabs( FastReciprocal( flImpulse ) ) * ( limit.m_flMaxTorque * step.m_flTimestep );
		const float flScale = flRatio < 1.0f ? flRatio : 1.0f;
		flImpulse *= flScale;
		limit.m_flReference = ( limit.m_flTargetSpeed * step.m_flTimestep + limit.m_flReference ) - ( flDelta - flScale * flDelta );
	}

	if ( limit.m_bLimited )
	{
		const float flPredicted = ( ( flImpulse * row.m_flInverseEffectiveMass + flVelocity ) * step.m_flTimestep ) + row.m_flMeasure;
		const float flGain = ( 1.0f * flGainA ) * ( step.m_flInverseTimestep * row.m_flEffectiveMass );
		flImpulse = flImpulse - ( Positive( ( flPredicted - limit.m_flUpper ) * flGain ) + Negative( ( flPredicted - limit.m_flLower ) * flGain ) );
	}

	ApplyRow( bodies, row, flImpulse );
}

void CRagdollSolver::SolveSwing( ConstraintBody_t bodies[2], Slot_t nSlot, float flAlpha, float flBeta, const ConstraintStep_t &step )
{
	Row_t &row = m_Rows[nSlot];
	if ( !row.m_bEnabled )
		return;

	Limit_t &limit = m_Limits[nSlot];
	const float flVelocity = GetRowVelocity( bodies, row );

	float flImpulse = 0.0f;
	if ( limit.m_bFriction && !m_bPlaneOnly )
	{
		const float flDelta = row.m_flMeasure - limit.m_flReference;
		float flDesired = ( ( ( flAlpha * SERVO_GAIN ) * flDelta ) * step.m_flInverseTimestep - flBeta * flVelocity ) * row.m_flEffectiveMass;
		flDesired = flDesired > FLT_EPSILON ? flDesired : FLT_EPSILON;
		const float flRatio = FastReciprocal( std::fabs( flDesired ) ) * ( limit.m_flMaxTorque * step.m_flTimestep );
		const float flScale = flRatio < 1.0f ? flRatio : 1.0f;
		flImpulse = flDesired * flScale;
		limit.m_flReference = ( limit.m_flTargetSpeed * step.m_flTimestep + limit.m_flReference ) + ( flDelta - flScale * flDelta );
	}

	if ( limit.m_bLimited )
	{
		const float flPredicted = row.m_flMeasure - ( ( flImpulse * row.m_flInverseEffectiveMass + flVelocity ) * step.m_flTimestep );
		const float flGain = ( 1.0f * flAlpha ) * ( step.m_flInverseTimestep * row.m_flEffectiveMass );
		flImpulse = ( Negative( ( flPredicted - limit.m_flLower ) * flGain ) + Positive( ( flPredicted - limit.m_flUpper ) * flGain ) ) + flImpulse;
	}

	ApplyRow( bodies, row, flImpulse );
}

void CRagdollSolver::Setup( ConstraintBody_t bodies[2], const ConstraintStep_t &step )
{
	const FloatMatrix_t rotations[2] = { ToFloat( bodies[0].m_pObject->GetCoreRotation() ), ToFloat( bodies[1].m_pObject->GetCoreRotation() ) };
	const b3Vec3 coneReference = RotateColumns( rotations[0], m_Cone[0] );
	const b3Vec3 twistAttached = RotateColumns( rotations[1], m_Twist[1] );
	const b3Vec3 twistReference = RotateColumns( rotations[0], m_Twist[0] );
	m_flSwingCosine = Sum3( Multiply( twistAttached, twistReference ) );
	const b3Vec3 coneAttached = RotateColumns( rotations[1], m_Cone[1] );

	const b3Vec3 sum = b3Add( twistAttached, twistReference );
	const float flSumSquared = Sum3( Multiply( sum, sum ) );
	const float flInverse = FastInverseSquareRoot( flSumSquared );
	const b3Vec3 bisector = b3MulSV( flInverse * 1.0f, sum );
	m_flHalfSwingCosine = ( flInverse * 0.5f ) * flSumSquared;

	const b3Vec3 normal = { coneReference.z * bisector.y - coneReference.y * bisector.z, coneReference.x * bisector.z - coneReference.z * bisector.x, coneReference.y * bisector.x - coneReference.x * bisector.y };
	const float flNormalSquared = Sum3( Multiply( normal, normal ) );
	const b3Vec3 n = flNormalSquared > FLT_EPSILON ? b3MulSV( FastInverseSquareRoot( flNormalSquared ), normal ) : b3Vec3_zero;
	const b3Vec3 tangent = { n.y * bisector.z - n.z * bisector.y, n.z * bisector.x - n.x * bisector.z, n.x * bisector.y - n.y * bisector.x };
	const float flTwist = 0.0f - FastArcTangent( Sum3( Multiply( n, coneAttached ) ), Sum3( Multiply( tangent, coneAttached ) ) );
	if ( m_bPlaneOnly )
	{
		m_Rows[SLOT_TWIST].m_flMeasure = flTwist;
		if ( !m_Limits[SLOT_PLANE].m_bLimited )
			return;

		BuildSwingRow( bodies, rotations, m_Rows[SLOT_PLANE], twistAttached, coneReference );
		SolveSwing( bodies, SLOT_PLANE, 1.0f, 1.0f, step );
		return;
	}

	BuildRow( bodies, rotations, m_Rows[SLOT_TWIST], bisector );
	m_Rows[SLOT_TWIST].m_flMeasure = flTwist;

	BuildSwingRow( bodies, rotations, m_Rows[SLOT_PLANE], twistAttached, coneReference );
	BuildSwingRow( bodies, rotations, m_Rows[SLOT_CONE], twistAttached, twistReference );
	Solve( bodies, 1.0f, 1.0f, step );
}

void CRagdollSolver::BuildSwingRow( ConstraintBody_t bodies[2], const FloatMatrix_t rotations[2], Row_t &row, b3Vec3 a, b3Vec3 b ) const
{
	const b3Vec3 cross = { b.z * a.y - b.y * a.z, b.x * a.z - b.z * a.x, b.y * a.x - b.x * a.y };
	const float flCrossSquared = Sum3( Multiply( cross, cross ) ) + 0.0f;
	row.m_flMeasure = Sum3( Multiply( a, b ) );
	if ( flCrossSquared <= FLT_EPSILON )
	{
		row.m_bEnabled = false;
		return;
	}

	const float flScale = std::max( FastInverseSquareRoot( flCrossSquared ) * MIN_SWING_JACOBIAN, 1.0f );
	BuildRow( bodies, rotations, row, b3MulSV( flScale, cross ) );
}

float CRagdollSolver::GetSwingLimit() const
{
	const float flConeLimit = std::acos( std::clamp( m_Limits[SLOT_CONE].m_flLower, -1.0f, 1.0f ) );
	const Limit_t &plane = m_Limits[SLOT_PLANE];
	const float flSine = std::sqrt( std::max( 1.0f - m_flSwingCosine * m_flSwingCosine, 0.0f ) );
	if ( !plane.m_bLimited || flSine <= FLT_EPSILON )
		return flConeLimit;

	const float flAzimuthCosine = m_Rows[SLOT_PLANE].m_flMeasure / flSine;
	const float flBound = flAzimuthCosine > 0.0f ? plane.m_flUpper / flAzimuthCosine : ( flAzimuthCosine < 0.0f ? plane.m_flLower / flAzimuthCosine : 1.0f );
	const float flPlaneLimit = flBound >= 1.0f ? 0.5f * PI : std::asin( std::max( flBound, 0.0f ) );
	return std::min( flConeLimit, flPlaneLimit );
}

void CRagdollSolver::Solve( ConstraintBody_t bodies[2], float flAlpha, float flBeta, const ConstraintStep_t &step )
{
	if ( m_bPlaneOnly )
	{
		SolveSwing( bodies, SLOT_PLANE, flAlpha, flBeta, step );
		return;
	}

	SolveTwist( bodies, flAlpha, flBeta, step );
	SolveSwing( bodies, SLOT_PLANE, flAlpha, flBeta, step );
	SolveSwing( bodies, SLOT_CONE, flAlpha, flBeta, step );
}
