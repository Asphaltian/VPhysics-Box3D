#include "physics_car_system.h"

#include "conversion.h"
#include "physics_environment.h"
#include "physics_object.h"
#include "physics_surface_props.h"
#include "physics_trace.h"

#include "bspflags.h"
#include "cmodel.h"
#include "gametrace.h"
#include "tier0/dbg.h"

#include "box3d/box3d.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

static constexpr float MIN_LENGTH_SQUARED = 1e-10f;
static constexpr float MIN_LENGTH = 1e-10f;
static constexpr float SUSPENSION_TOP_OFFSET = 500.0f;
static constexpr float AIRBORNE_BODY_FORCE_SCALE = 0.1f;
static constexpr float BODY_MOTOR_MAX_ROTATION_SPEED = 314.15927f;
static constexpr float LATERAL_FORCE_LEVER = 2.0f;
static constexpr real_t FAR_ANCHOR_DISTANCE = 1e8f;
static constexpr float ACKERMANN_MIN_ANGLE = 0.001f;
static constexpr float CENTER_WHEEL_STEERING_SCALE = 0.5f;
static constexpr float FIXED_WHEEL_SPIN_TRANSFER = 0.25f;
static constexpr float IMPACT_NORMAL_FACTOR = 1.1f;
static constexpr float IMPACT_NORMAL_BIAS = 0.1f;
static constexpr float MAX_NORMAL_FACTOR = 3.0f;
static constexpr float MIN_DETERMINANT_SQUARED = 1e-20f;
static constexpr float SIDE_FORCE_SCALE = -1.2f;
static constexpr float STOPPED_SPEED = 0.5f;
static constexpr float STOPPED_FORCE_SCALE = -0.5f;
static constexpr float ROLLING_RESISTANCE = 0.625f;
static constexpr float AIRBOAT_GRAVITY = 9.81f;
static constexpr float PONTOON_RAY_LENGTH = 0.35f;
static constexpr float PONTOON_WATER_RAY_LENGTH = 0.1f;
static constexpr float PONTOON_DEEP_RAY_LENGTH = 3.937f;
static constexpr float PONTOON_MAX_DEPTH = 0.41f;
static constexpr float PONTOON_BUOYANCY = -28.448f;
static constexpr float WATER_DEPTH_PROBE = 1000.0f;
static constexpr float WAVE_SPEED = 15.0f;
static constexpr float WAVE_PHASE_SPEED = 0.3f;
static constexpr float WAVE_RATE = 1.5f;
static constexpr float WAVE_AMPLITUDE = 0.02f;
static constexpr float WAVE_MIN_AMPLITUDE = 0.01f;
static constexpr float LOW_AIR_SPEED = 11.0f;
static constexpr float FRICTION_SPEED = 5.886f;
static constexpr float FRICTION_ENERGY_THRESHOLD = 0.05f;
static constexpr float THRUST_FORWARD = 11.0f;
static constexpr float THRUST_REVERSE = 7.5f;
static constexpr float MIN_THROTTLE = 0.01f;
static constexpr float MIN_STEERING = 0.01f;
static constexpr float ANALOG_REVERSE_THRUST = 2.0f;
static constexpr float RIGHTING_COS = 0.9848077f;
static constexpr float RIGHTING_SIN = 0.17364818f;
static constexpr double ROLL_RIGHTING_ANGLE = 0.17453292;
static constexpr float AXIS_LENGTH_BIAS = 1e-10f;
static constexpr float WATER_DRAG_LATERAL = -0.6f;
static constexpr float WATER_DRAG_VERTICAL = -0.0025f;
static constexpr float WATER_DRAG_FORWARD = -0.005f;
static constexpr float GROUND_DRAG_LATERAL = 2.0f;
static constexpr double GROUND_DRAG_VERTICAL = 0.8;
static constexpr float STEERING_TORQUE_SCALE = 0.0018f;
static constexpr float STEERING_TORQUE_BASE = 0.00045f;
static constexpr float YAW_DAMPING = 0.001f;
static constexpr float YAW_DAMPING_QUADRATIC = 0.00004f;
static constexpr float PITCH_RIGHTING_RATE = 0.04f;
static constexpr float PITCH_RIGHTING_STIFFNESS = 0.1f;
static constexpr float PITCH_RIGHTING_MAX_TORQUE = 0.02617994f;
static constexpr float ROLL_RIGHTING_RATE = 0.3f;
static constexpr float ROLL_RIGHTING_STIFFNESS = 0.2f;
static constexpr float ROLL_RIGHTING_MAX_TORQUE = 0.034906585f;

static float NormalizeLength( b3Vec3 &v )
{
	const float flLengthSquared = ( v.x * v.x + v.y * v.y ) + v.z * v.z;
	if ( flLengthSquared < MIN_LENGTH_SQUARED )
		return 0.0f;

	const float flLength = std::sqrt( flLengthSquared );
	v = b3MulSV( 1.0f / flLength, v );
	return flLength;
}

static b3Vec3 Tangential( b3Vec3 v, b3Vec3 normal )
{
	return b3MulSub( v, b3Dot( v, normal ), normal );
}

static b3Vec3 GetPointVelocity( const CPhysicsObject *pObject, RealVec3_t worldPosition )
{
	return b3Add( pObject->GetCoreVelocity(), b3Cross( pObject->CoreToIVP( pObject->GetCoreAngularVelocity() ), ToFloat( worldPosition - pObject->GetCorePosition() ) ) );
}

static b3Vec3 GetAnchorVelocity( const CPhysicsObject *pObject, b3Vec3 coreAnchor )
{
	return pObject->IsMoveable() ? pObject->GetCoreSurfaceVelocity( coreAnchor ) : b3Vec3_zero;
}

static void AddSpeed( CPhysicsObject *pObject, b3Vec3 impulse )
{
	pObject->SetCoreVelocity( b3MulAdd( pObject->GetCoreVelocity(), pObject->GetInvMass(), impulse ) );
}

static void AddRotationImpulse( CPhysicsObject *pObject, b3Vec3 impulse )
{
	pObject->SetCoreAngularVelocity( b3Add( pObject->GetCoreAngularVelocity(), b3Mul( pObject->GetCoreInverseInertia(), impulse ) ) );
}

static b3Vec3 RotateAxle( float flAngle )
{
	return { std::cos( flAngle ), 0.0f, -std::sin( flAngle ) };
}

static Ray_t MakeRay( const Vector &start, const Vector &delta )
{
	Ray_t ray;
	ray.m_Start = VectorAligned( start );
	ray.m_Delta = VectorAligned( delta );
	ray.m_StartOffset.Init();
	ray.m_Extents.Init();
	ray.m_IsRay = true;
	ray.m_IsSwept = delta.LengthSqr() != 0.0f;
	return ray;
}

static float GetSurfaceFriction( int nSurfaceProps )
{
	float flDensity, flThickness, flFriction, flElasticity;
	g_PhysicsSurfaceProps.GetPhysicsProperties( nSurfaceProps, &flDensity, &flThickness, &flFriction, &flElasticity );
	return flFriction;
}

class CAirboatGravityController final : public IController
{
public:
	static constexpr int PRIORITY = 1000;

	explicit CAirboatGravityController( CPhysicsObject *pObject )
	{
		pObject->AttachController( this );
	}

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "sys:gravity"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override
	{
		pObject->ApplyGravity( flTimestep, { 0.0f, AIRBOAT_GRAVITY, 0.0f }, false );
	}

	void ObjectDestroyed( CPhysicsObject *pObject ) override
	{
		pObject->DetachController( this );
		delete this;
	}
};

CSuspensionActuator::CSuspensionActuator( CPhysicsObject *pBody, CPhysicsObject *pWheel, b3Vec3 bodyAnchor, float flLength, float flConstant, float flReboundDamping, float flCompressionDamping, float flMaxBodyForce )
: m_pBody( pBody )
, m_pWheel( pWheel )
, m_BodyAnchor( bodyAnchor )
, m_WheelAnchor( pWheel->GetShift() )
, m_flLength( flLength )
, m_flConstant( flConstant )
, m_flReboundDamping( flReboundDamping )
, m_flCompressionDamping( flCompressionDamping )
, m_flMaxBodyForce( flMaxBodyForce )
{
	pBody->AttachController( this );
	pWheel->AttachController( this );
}

CSuspensionActuator::~CSuspensionActuator()
{
	m_pBody->DetachController( this );
	m_pWheel->DetachController( this );
}

void CSuspensionActuator::SetLength( float flLength )
{
	if ( flLength == m_flLength )
		return;

	m_flLength = flLength;
	m_pBody->Wake();
}

void CSuspensionActuator::Simulate( CPhysicsObject *, float flTimestep )
{
	CPhysicsEnvironment *pEnvironment = m_pBody->GetEnvironment();
	if ( m_nSimulatedStep == pEnvironment->GetStepCount() )
		return;

	m_nSimulatedStep = pEnvironment->GetStepCount();
	const RealVec3_t top = m_pBody->TransformCorePoint( m_BodyAnchor );
	const RealVec3_t center = m_pWheel->TransformCorePoint( m_WheelAnchor );
	b3Vec3 direction = ToFloat( top - center );
	const float flLength = NormalizeLength( direction );
	if ( flLength < MIN_LENGTH )
		return;

	const b3Vec3 relative = b3Sub( GetAnchorVelocity( m_pWheel, m_WheelAnchor ), GetAnchorVelocity( m_pBody, m_BodyAnchor ) );
	const float flApproach = ( relative.y * direction.y + relative.x * direction.x ) + relative.z * direction.z;
	const float flDamping = flApproach >= 0.0f ? m_flCompressionDamping : m_flReboundDamping;
	const real_t flForce = ( real_t( flLength ) - m_flLength ) * m_flConstant - real_t( flDamping ) * flApproach;
	const real_t flReaction = -flForce;
	const real_t flMaxBodyForce = m_flMaxBodyForce;
	real_t flBodyForce = !( flReaction >= -flMaxBodyForce ) ? -flMaxBodyForce : std::min( flMaxBodyForce, flReaction );

	if ( m_pWheel->IsMoveable() )
		m_pWheel->ApplyImpulse( center, Scale( direction, real_t( flTimestep ) * flForce ) );

	if ( !pEnvironment->HasFrictionContact( m_pWheel ) )
		flBodyForce *= AIRBORNE_BODY_FORCE_SCALE;
	if ( m_pBody->IsMoveable() )
		m_pBody->ApplyImpulse( top, Scale( direction, real_t( flTimestep ) * flBodyForce ) );
}

void CSuspensionActuator::ObjectDestroyed( CPhysicsObject * )
{
	delete this;
}

CTorqueActuator::CTorqueActuator( CPhysicsObject *pObject, float flMaxRotationSpeed )
: m_pObject( pObject )
, m_flMaxRotationSpeed( flMaxRotationSpeed )
{
	pObject->AttachController( this );
}

CTorqueActuator::~CTorqueActuator()
{
	m_pObject->DetachController( this );
}

void CTorqueActuator::SetPower( float flPower )
{
	if ( flPower == m_flPower )
		return;

	m_flPower = flPower;
	m_pObject->Wake();
}

void CTorqueActuator::Simulate( CPhysicsObject *, float flTimestep )
{
	m_flMeasuredSpeed = 0.0f;
	if ( m_flPower == 0.0f || !m_pObject->IsMoveable() )
		return;

	constexpr b3Vec3 axis = { -1.0f, 0.0f, 0.0f };
	const b3Vec3 w = m_pObject->GetCoreAngularVelocity();
	m_flMeasuredSpeed = ( w.y * axis.y + w.x * axis.x ) + w.z * axis.z;
	if ( !( std::fabs( m_flMeasuredSpeed ) <= m_flMaxRotationSpeed ) )
		return;

	const b3Vec3 inverseInertia = m_pObject->GetCoreInverseInertia();
	const float flScale = flTimestep * m_flPower;
	m_pObject->AddPendingAngularVelocity( { ( inverseInertia.x * axis.x ) * flScale, ( axis.y * inverseInertia.y ) * flScale, ( inverseInertia.z * axis.z ) * flScale } );
}

float CTorqueActuator::GetMeasuredSpeed() const
{
	return m_pObject->IsAsleep() || !m_pObject->IsMoveable() ? 0.0f : m_flMeasuredSpeed;
}

void CTorqueActuator::ObjectDestroyed( CPhysicsObject * )
{
	delete this;
}

CForceActuator::CForceActuator( CPhysicsObject *pObject, b3Vec3 anchor, b3Vec3 targetAnchor, float flForce )
: m_pObject( pObject )
, m_Anchor( anchor )
, m_TargetAnchor( targetAnchor )
, m_flForce( flForce )
, m_bWorldTarget( false )
{
	pObject->AttachController( this );
}

CForceActuator::CForceActuator( CPhysicsObject *pObject, b3Vec3 anchor, RealVec3_t worldTarget, float flForce )
: m_pObject( pObject )
, m_Anchor( anchor )
, m_WorldTarget( worldTarget )
, m_flForce( flForce )
, m_bWorldTarget( true )
{
	pObject->AttachController( this );
}

CForceActuator::~CForceActuator()
{
	m_pObject->DetachController( this );
}

void CForceActuator::SetForce( float flForce )
{
	if ( flForce == m_flForce )
		return;

	m_flForce = flForce;
	m_pObject->Wake();
}

RealVec3_t CForceActuator::GetAnchorPosition() const
{
	return m_pObject->TransformCorePoint( m_Anchor );
}

RealVec3_t CForceActuator::GetTargetPosition() const
{
	return m_bWorldTarget ? m_WorldTarget : m_pObject->TransformCorePoint( m_TargetAnchor );
}

void CForceActuator::Simulate( CPhysicsObject *, float flTimestep )
{
	if ( m_flForce == 0.0f )
		return;

	const RealVec3_t anchor = GetAnchorPosition();
	b3Vec3 direction = ToFloat( anchor - GetTargetPosition() );
	NormalizeLength( direction );
	if ( m_pObject->IsMoveable() )
		m_pObject->ApplyImpulse( anchor, b3MulSV( m_flForce * flTimestep, direction ) );
}

void CForceActuator::ObjectDestroyed( CPhysicsObject * )
{
	delete this;
}

CCarWheelsSystem::CCarWheelsSystem( const CarSystemTemplate_t &carTemplate )
: m_pBody( carTemplate.m_pBody )
, m_nWheels( carTemplate.m_nWheels )
, m_nAxles( carTemplate.m_nAxles )
, m_flCounterTorqueFactor( carTemplate.m_flCounterTorqueFactor )
{
	const b3Vec3 bodyShift = m_pBody->GetShift();
	const b3Quat frame = { { 0.0f, std::numbers::sqrt2_v<float> * 0.5f, 0.0f }, std::numbers::sqrt2_v<float> * 0.5f };
	for ( int i = 0; i < m_nWheels; ++i )
	{
		CPhysicsObject *pWheel = carTemplate.m_pWheels[i];
		const b3Vec3 position = carTemplate.m_WheelPositions[i];
		m_pWheels[i] = pWheel;
		m_Attachments[i] = b3Add( position, bodyShift );
		pWheel->SetCarWheel();

		b3WheelJointDef def = b3DefaultWheelJointDef();
		def.base.bodyIdA = m_pBody->GetBody();
		def.base.bodyIdB = pWheel->GetBody();
		def.base.localFrameA = { IVPToSource( position ), frame };
		def.base.localFrameB = { b3Vec3_zero, frame };
		def.base.collideConnected = true;
		def.enableSuspensionSpring = false;
		def.enableSuspensionLimit = false;
		def.enableSpinMotor = false;
		def.maxSpinTorque = std::numeric_limits<float>::max();
		def.spinSpeed = 0.0f;
		def.enableSteering = true;
		def.steeringHertz = 0.0f;
		def.maxSteeringTorque = 0.0f;
		def.enableSteeringLimit = true;
		def.lowerSteeringLimit = 0.0f;
		def.upperSteeringLimit = 0.0f;
		m_Joints[i] = b3CreateWheelJoint( m_pBody->GetEnvironment()->GetWorld(), &def );

		const b3Vec3 top = { position.x, position.y - SUSPENSION_TOP_OFFSET, position.z };
		m_pSuspensions[i] = new CSuspensionActuator( m_pBody, pWheel, b3Add( top, bodyShift ), SUSPENSION_TOP_OFFSET + carTemplate.m_SpringAdditionalLengths[i], carTemplate.m_SpringConstants[i], carTemplate.m_SpringDampings[i], carTemplate.m_SpringDampingCompressions[i], carTemplate.m_MaxBodyForces[i] );
		m_pWheelMotors[i] = new CTorqueActuator( pWheel, carTemplate.m_MaxWheelRotationSpeeds[( i >> 1 ) & 1] );
	}

	m_pBodyMotor = new CTorqueActuator( m_pBody, BODY_MOTOR_MAX_ROTATION_SPEED );

	const RealVec3_t farAnchor = { 0.0f, m_pBody->GetEnvironment()->GetIVPGravity().y > 0 ? -FAR_ANCHOR_DISTANCE : FAR_ANCHOR_DISTANCE, 0.0f };
	m_pDownforce = new CForceActuator( m_pBody, { 0.0f, carTemplate.m_flDownforceHeight, 0.0f }, farAnchor, 0.0f );
	new CForceActuator( m_pBody, b3Vec3_zero, farAnchor, carTemplate.m_flExtraGravityForce );
	m_pFrontLateral = new CForceActuator( m_pBody, { -1.0f, 0.0f, LATERAL_FORCE_LEVER }, b3Vec3{ 1.0f, 0.0f, LATERAL_FORCE_LEVER }, 0.0f );
	m_pRearLateral = new CForceActuator( m_pBody, { -1.0f, 0.0f, -LATERAL_FORCE_LEVER }, b3Vec3{ 1.0f, 0.0f, -LATERAL_FORCE_LEVER }, 0.0f );

	DoSteering( 0.0f, false );
}

CCarWheelsSystem::~CCarWheelsSystem()
{
	for ( int i = 0; i < m_nWheels; ++i )
	{
		if ( b3Joint_IsValid( m_Joints[i] ) )
			b3DestroyJoint( m_Joints[i], false );
	}
}

float CCarWheelsSystem::GetBodySpeed() const
{
	return static_cast<float>( InverseRotate( m_pBody->GetCoreRotation(), m_pBody->GetCoreImplicitVelocity() ).z );
}

float CCarWheelsSystem::GetTrackWidth() const
{
	return m_Attachments[1].x - m_Attachments[0].x;
}

float CCarWheelsSystem::GetWheelbase() const
{
	return std::fabs( m_Attachments[0].z - m_Attachments[2].z );
}

void CCarWheelsSystem::DoSteeringWheel( int nWheel, float flAngle )
{
	b3WheelJoint_SetSteeringLimits( m_Joints[nWheel], flAngle, flAngle );
}

void CCarWheelsSystem::DoSteering( float flAngle, bool )
{
	if ( flAngle == m_flSteeringAngle )
		return;

	float flTrackWidth = 1.0f;
	float flWheelbase = 1.0f;
	if ( m_nWheels > 3 )
	{
		flTrackWidth = GetTrackWidth();
		flWheelbase = GetWheelbase();
	}

	b3Vec3 pending = m_pBody->GetPendingAngularVelocity();
	pending.y = static_cast<float>( real_t( pending.y ) - ( real_t( GetBodySpeed() ) * ( flAngle - m_flSteeringAngle ) ) / flWheelbase );
	m_pBody->SetPendingAngularVelocity( pending );
	m_flSteeringAngle = flAngle;
	m_pBody->Wake();

	const int nSteeredWheels = m_nWheels / m_nAxles;
	for ( int i = 0; i < nSteeredWheels; ++i )
	{
		float flWheelAngle = flAngle;
		if ( ( i & 1 ) != static_cast<int>( flAngle > 0.0f ) && m_nWheels > 3 && std::fabs( flAngle ) >= ACKERMANN_MIN_ANGLE )
		{
			const real_t flTangent = std::tan( real_t( std::fabs( flAngle ) ) );
			flWheelAngle = static_cast<float>( std::atan( ( flTangent * flWheelbase ) / ( flTangent * flTrackWidth + flWheelbase ) ) );
			if ( flAngle < 0.0f )
				flWheelAngle = -flWheelAngle;
		}
		DoSteeringWheel( i, flWheelAngle );
	}

	for ( int i = 4; i < std::min( m_nWheels, 6 ); ++i )
		DoSteeringWheel( i, flAngle * CENTER_WHEEL_STEERING_SCALE );
}

void CCarWheelsSystem::ChangeWheelTorque( int nWheel, float flTorque )
{
	m_pWheelMotors[nWheel]->SetPower( flTorque );
}

void CCarWheelsSystem::UpdateBodyCounterTorque()
{
	float flPower = 0.0f;
	for ( int i = 0; i < m_nWheels; ++i )
		flPower -= m_pWheelMotors[i]->GetPower();
	m_pBodyMotor->SetPower( flPower * m_flCounterTorqueFactor );
}

void CCarWheelsSystem::ChangeBodyDownforce( float flForce )
{
	m_pDownforce->SetForce( flForce );
}

void CCarWheelsSystem::FixWheel( int nWheel, bool bFix )
{
	if ( bFix == m_FixedWheels[nWheel] )
		return;

	m_FixedWheels[nWheel] = bFix;
	if ( bFix )
	{
		CPhysicsObject *pWheel = m_pWheels[nWheel];
		pWheel->SetCoreAngularVelocity( b3MulAdd( pWheel->GetCoreAngularVelocity(), FIXED_WHEEL_SPIN_TRANSFER, m_pBody->GetCoreAngularVelocity() ) );
	}
	b3WheelJoint_EnableSpinMotor( m_Joints[nWheel], bFix );
}

void CCarWheelsSystem::SetPowerslide( float flFront, float flRear )
{
	const float flMass = m_pBody->GetMass();
	m_pFrontLateral->SetForce( flFront * flMass );
	m_pRearLateral->SetForce( flRear * flMass );
}

float CCarWheelsSystem::GetWheelAngularVelocity( int nWheel ) const
{
	return m_pWheelMotors[nWheel]->GetMeasuredSpeed();
}

void CCarWheelsSystem::SetSpringLength( int nWheel, float flLength )
{
	m_pSuspensions[nWheel]->SetLength( flLength );
}

void CCarWheelsSystem::SetBoosterForce( float flForce )
{
	if ( flForce == 0.0f )
	{
		delete m_pBoosterThrust;
		delete m_pBoosterAntiGravity;
		m_pBoosterThrust = nullptr;
		m_pBoosterAntiGravity = nullptr;
		return;
	}

	const float flMass = m_pBody->GetMass();
	if ( m_pBoosterThrust )
	{
		m_pBoosterThrust->SetForce( flForce * flMass );
		return;
	}

	m_pBoosterThrust = new CForceActuator( m_pBody, { 0.0f, 0.0f, 1.0f }, b3Vec3{ 0.0f, 0.0f, -1.0f }, flForce * flMass );
	const float flAntiGravity = static_cast<float>( -real_t( flMass ) * m_pBody->GetEnvironment()->GetIVPGravity().y );
	m_pBoosterAntiGravity = new CForceActuator( m_pBody, b3Vec3_zero, RealVec3_t{ 0.0f, -FAR_ANCHOR_DISTANCE, 0.0f }, flAntiGravity );
}

void CCarWheelsSystem::ActivateBooster( float flForce, float flDuration, float flDelay )
{
	if ( ( m_pBoosterThrust && flForce != 0.0f ) || !( m_flBoosterDelay <= 0.0f ) )
		return;

	if ( flForce != 0.0f )
		SetBoosterForce( static_cast<float>( Length( m_pBody->GetEnvironment()->GetIVPGravity() ) ) * flForce );
	m_flBoosterTimeToGo = flDuration;
	m_flBoosterDelay = flDuration + flDelay;
}

void CCarWheelsSystem::UpdateBooster( float flTimestep )
{
	if ( m_flBoosterDelay > 0.0f )
		m_flBoosterDelay -= flTimestep;

	if ( m_flBoosterTimeToGo > 0.0f )
	{
		m_flBoosterTimeToGo -= flTimestep;
		if ( m_flBoosterTimeToGo <= 0.0f )
			SetBoosterForce( 0.0f );
	}
}

void CCarWheelsSystem::GetDebugData( CarSystemDebugData_t &data ) const
{
	data.m_AxlePositions[0] = m_pFrontLateral->GetAnchorPosition();
	data.m_AxlePositions[1] = m_pFrontLateral->GetTargetPosition();
	data.m_AxlePositions[2] = m_pRearLateral->GetAnchorPosition();
}

CCarRaycastSystem::CCarRaycastSystem( const CarSystemTemplate_t &carTemplate )
: m_pBody( carTemplate.m_pBody )
, m_nWheels( carTemplate.m_nWheels )
, m_nAxles( carTemplate.m_nAxles )
, m_nWheelsPerAxle( carTemplate.m_nWheels / carTemplate.m_nAxles )
, m_flExtraGravityForce( carTemplate.m_flExtraGravityForce )
{
	const RealVec3_t gravity = m_pBody->GetEnvironment()->GetIVPGravity();
	const float flGravitySign = gravity.y > 0 ? 1.0f : -1.0f;
	m_GravityDirection = ToFloat( gravity );
	NormalizeLength( m_GravityDirection );
	m_pBody->AttachController( this );

	const b3Vec3 bodyShift = m_pBody->GetShift();
	for ( int i = 0; i < m_nWheels; ++i )
	{
		CPhysicsObject *pWheel = carTemplate.m_pWheels[i];
		Wheel_t &wheel = m_Wheels[i];
		wheel.m_HardPoint = b3Add( carTemplate.m_WheelPositions[i], bodyShift );
		wheel.m_SpringDirection = { 0.0f, flGravitySign, 0.0f };
		wheel.m_flSpringLength = carTemplate.m_SpringAdditionalLengths[i];
		wheel.m_flSpringConstant = carTemplate.m_SpringConstants[i];
		wheel.m_flSpringDamping = carTemplate.m_SpringDampings[i];
		wheel.m_flSpringDampingCompression = carTemplate.m_SpringDampingCompressions[i];
		wheel.m_flRadius = pWheel->GetIVPSphereRadius();
		wheel.m_flInverseRadius = 1.0f / wheel.m_flRadius;
		wheel.m_Axle = RotateAxle( 0.0f );
		m_pWheels[i] = pWheel;
		pWheel->EnableCollisions( false );
	}

	DoSteering( 0.0f, false );
	for ( int i = 0; i < m_nAxles; ++i )
		m_StabilizerConstants[i] = carTemplate.m_StabilizerConstants[i];
}

CCarRaycastSystem::~CCarRaycastSystem()
{
	m_pBody->DetachController( this );
}

float CCarRaycastSystem::GetBodySpeed() const
{
	return static_cast<float>( InverseRotate( m_pBody->GetCoreRotation(), m_pBody->GetCoreVelocity() ).z );
}

void CCarRaycastSystem::DoSteeringWheel( int nWheel, float flAngle )
{
	m_Wheels[nWheel].m_Axle = RotateAxle( flAngle );
}

void CCarRaycastSystem::DoSteering( float flAngle, bool )
{
	if ( flAngle == m_flSteeringAngle )
		return;

	m_flSteeringAngle = flAngle;
	m_pBody->Wake();
	for ( int i = 0; i < m_nWheelsPerAxle; ++i )
		DoSteeringWheel( i, flAngle );
}

void CCarRaycastSystem::ChangeWheelTorque( int nWheel, float flTorque )
{
	m_Wheels[nWheel].m_flTorque = flTorque;
	m_pBody->Wake();
}

void CCarRaycastSystem::FixWheel( int nWheel, bool bFix )
{
	m_Wheels[nWheel].m_bFixed = bFix;
}

float CCarRaycastSystem::GetWheelAngularVelocity( int nWheel ) const
{
	return m_Wheels[nWheel].m_flAngularVelocity;
}

void CCarRaycastSystem::SetSpringLength( int nWheel, float flLength )
{
	m_Wheels[nWheel].m_flSpringLength = flLength;
}

void CCarRaycastSystem::ActivateBooster( float flForce, float flDuration, float flDelay )
{
	if ( m_flBoosterAcceleration != 0.0f || !( m_flBoosterDelay <= 0.0f ) )
		return;

	m_flBoosterTimeToGo = flDuration;
	m_flBoosterAcceleration = flForce;
	m_flBoosterDelay = flDuration + flDelay;
}

struct RayHit_t
{
	CPhysicsObject *m_pObject;
	b3Vec3 m_Normal;
	float m_flFraction;
};

static bool CollectRayCandidate( b3ShapeId shapeId, void *pContext )
{
	std::vector<CPhysicsObject *> &candidates = *static_cast<std::vector<CPhysicsObject *> *>( pContext );
	CPhysicsObject *pObject = static_cast<CPhysicsObject *>( b3Body_GetUserData( b3Shape_GetBody( shapeId ) ) );
	if ( std::find( candidates.begin(), candidates.end(), pObject ) == candidates.end() )
		candidates.push_back( pObject );
	return true;
}

static void TraceWheelRay( CPhysicsObject *pObject, const Vector &start, const Vector &end, RayHit_t &hit )
{
	if ( const CPhysCollide *pCollide = pObject->GetCollide() )
	{
		Vector origin;
		QAngle angles;
		pObject->GetPosition( &origin, &angles );
		Ray_t ray;
		ray.Init( start, end );
		trace_t trace;
		TraceBoxAgainstCollide( ray, MASK_ALL, nullptr, pCollide, origin, angles, &trace );
		if ( !trace.startsolid && trace.fraction < hit.m_flFraction )
			hit = { pObject, ToBox3D( trace.plane.normal ), trace.fraction };
		return;
	}

	Vector center;
	pObject->GetPosition( &center, nullptr );
	const float flRadius = pObject->GetSphereRadius();
	const Vector delta = end - start;
	const Vector offset = start - center;
	const float a = delta.Dot( delta );
	const float b = offset.Dot( delta );
	const float c = offset.Dot( offset ) - flRadius * flRadius;
	const float flDiscriminant = b * b - a * c;
	if ( c <= 0.0f || flDiscriminant < 0.0f || a <= 0.0f )
		return;

	const float flFraction = ( -b - std::sqrt( flDiscriminant ) ) / a;
	if ( flFraction < 0.0f || !( flFraction < hit.m_flFraction ) )
		return;

	const Vector normal = ( start + delta * flFraction - center ) / flRadius;
	hit = { pObject, ToBox3D( normal ), flFraction };
}

void CCarRaycastSystem::Simulate( CPhysicsObject *, float flTimestep )
{
	CPhysicsEnvironment *pEnvironment = m_pBody->GetEnvironment();
	const float flInverseTimestep = pEnvironment->GetInverseTimestep();
	const RealMatrix3_t &rotation = m_pBody->GetCoreRotation();
	const RealVec3_t position = m_pBody->GetCorePosition();

	RealVec3_t contacts[VEHICLE_MAX_WHEEL_COUNT];
	b3Vec3 directions[VEHICLE_MAX_WHEEL_COUNT];
	b3Vec3 normals[VEHICLE_MAX_WHEEL_COUNT];
	b3Vec3 contactVelocities[VEHICLE_MAX_WHEEL_COUNT];
	b3Vec3 tangentVelocities[VEHICLE_MAX_WHEEL_COUNT];
	b3Vec3 worldAxles[VEHICLE_MAX_WHEEL_COUNT];
	b3Vec3 projectedAxles[VEHICLE_MAX_WHEEL_COUNT];
	float normalFactors[VEHICLE_MAX_WHEEL_COUNT];
	float frictions[VEHICLE_MAX_WHEEL_COUNT];
	bool hits[VEHICLE_MAX_WHEEL_COUNT];

	for ( int i = 0; i < m_nWheels; ++i )
	{
		Wheel_t &wheel = m_Wheels[i];
		const RealVec3_t start = m_pBody->TransformCorePoint( wheel.m_HardPoint );
		const b3Vec3 direction = ToFloat( Rotate( rotation, wheel.m_SpringDirection ) );
		const float flLength = wheel.m_flSpringLength + wheel.m_flRadius;
		directions[i] = direction;

		const Vector rayStart = IVPToSourcePosition( start );
		const Vector rayEnd = IVPToSourcePosition( start + real_t( flLength ) * ToReal( direction ) );
		Vector mins, maxs;
		VectorMin( rayStart, rayEnd, mins );
		VectorMax( rayStart, rayEnd, maxs );
		static thread_local std::vector<CPhysicsObject *> s_Candidates;
		s_Candidates.clear();
		b3World_OverlapAABB( pEnvironment->GetWorld(), { ToBox3D( mins ), ToBox3D( maxs ) }, b3DefaultQueryFilter(), CollectRayCandidate, &s_Candidates );
		RayHit_t hit = { nullptr, b3Vec3_zero, 1.0f };
		for ( CPhysicsObject *pCandidate : s_Candidates )
			TraceWheelRay( pCandidate, rayStart, rayEnd, hit );

		hits[i] = hit.m_pObject != nullptr;
		if ( !hits[i] )
		{
			wheel.m_flSuspensionForce = 0.0f;
			wheel.m_flHitDistance = flLength;
			normals[i] = b3Neg( direction );
			normalFactors[i] = 1.0f;
			frictions[i] = 0.0f;
		}
		else
		{
			b3Vec3 normal = SourceToIVPDirection( hit.m_Normal );
			if ( !hit.m_pObject->GetCollide() )
			{
				RealMatrix3_t hitRotation;
				RealVec3_t hitOrigin;
				hit.m_pObject->GetPose( pEnvironment->GetCurrentTime(), hitRotation, hitOrigin );
				normal = ToFloat( Rotate( hitRotation, normal ) );
			}
			normals[i] = normal;
			wheel.m_flHitDistance = hit.m_flFraction * flLength;
			normalFactors[i] = IMPACT_NORMAL_FACTOR / ( std::fabs( b3Dot( direction, normal ) ) + IMPACT_NORMAL_BIAS );
			frictions[i] = GetSurfaceFriction( hit.m_pObject->GetMaterialIndex() );
		}

		contacts[i] = start + real_t( wheel.m_flHitDistance ) * ToReal( direction );
		contactVelocities[i] = GetPointVelocity( m_pBody, contacts[i] );
		tangentVelocities[i] = Tangential( contactVelocities[i], normals[i] );
		worldAxles[i] = ToFloat( Rotate( rotation, wheel.m_Axle ) );
		projectedAxles[i] = Tangential( worldAxles[i], normals[i] );
		if ( NormalizeLength( projectedAxles[i] ) < MIN_LENGTH )
			return;
	}

	float extraForces[VEHICLE_MAX_WHEEL_COUNT] = {};
	if ( m_nWheelsPerAxle == 2 )
	{
		for ( int i = 0; i < m_nAxles; ++i )
		{
			const Wheel_t &left = m_Wheels[2 * i];
			const Wheel_t &right = m_Wheels[2 * i + 1];
			const float flLeft = ( left.m_flHitDistance - left.m_flSpringLength ) - left.m_flRadius;
			const float flRight = ( right.m_flHitDistance - right.m_flSpringLength ) - right.m_flRadius;
			const float flForce = ( ( flRight - flLeft ) * m_StabilizerConstants[i] ) * 0.5f;
			extraForces[2 * i] = flForce;
			extraForces[2 * i + 1] = -flForce;
		}
	}

	m_pBody->SetCoreVelocity( b3MulAdd( m_pBody->GetCoreVelocity(), ( m_flExtraGravityForce * m_pBody->GetInvMass() ) * flTimestep, m_GravityDirection ) );

	for ( int i = 0; i < m_nWheels; ++i )
	{
		Wheel_t &wheel = m_Wheels[i];
		const float flCompression = ( wheel.m_flHitDistance - wheel.m_flSpringLength ) - wheel.m_flRadius;
		if ( !hits[i] || !( flCompression < 0.0f ) )
			continue;

		const float flNormalFactor = std::clamp( normalFactors[i], 0.0f, MAX_NORMAL_FACTOR );
		const float flRelative = b3Dot( b3Sub( tangentVelocities[i], contactVelocities[i] ), directions[i] );
		const float flDamping = flRelative <= 0.0f ? wheel.m_flSpringDampingCompression : wheel.m_flSpringDamping;
		const float flForce = std::max( ( extraForces[i] - flCompression * wheel.m_flSpringConstant ) * flNormalFactor - flRelative * flDamping, 0.0f );
		wheel.m_flSuspensionForce = flForce;
		m_pBody->ApplyDirectImpulse( contacts[i], b3MulSV( flForce * flTimestep, normals[i] ) );
	}

	if ( m_flBoosterDelay > 0.0f )
		m_flBoosterDelay -= flTimestep;
	if ( m_flBoosterTimeToGo > 0.0f )
	{
		m_flBoosterTimeToGo -= flTimestep;
		if ( m_flBoosterTimeToGo <= 0.0f )
			m_flBoosterAcceleration = 0.0f;
	}
	if ( m_flBoosterAcceleration != 0.0f )
	{
		const b3Vec3 forward = { static_cast<float>( rotation.m[0][2] ), static_cast<float>( rotation.m[1][2] ), static_cast<float>( rotation.m[2][2] ) };
		m_pBody->SetCoreVelocity( b3MulAdd( m_pBody->GetCoreVelocity(), m_flBoosterAcceleration * flTimestep, forward ) );
	}

	const auto rayWheel = [&]( int nWheel ) { return std::min( nWheel, m_nWheels - 1 ); };
	RealVec3_t axlePoints[2];
	b3Vec3 axleDirections[2];
	if ( m_nWheelsPerAxle == 2 )
	{
		axlePoints[0] = real_t( 0.5f ) * ( contacts[rayWheel( 0 )] + contacts[rayWheel( 1 )] );
		axlePoints[1] = real_t( 0.5f ) * ( contacts[rayWheel( 2 )] + contacts[rayWheel( 3 )] );
		axleDirections[0] = worldAxles[rayWheel( 0 )];
		axleDirections[1] = worldAxles[rayWheel( 2 )];
	}
	else
	{
		axlePoints[0] = contacts[rayWheel( 0 )];
		axlePoints[1] = contacts[rayWheel( 1 )];
		axleDirections[0] = worldAxles[rayWheel( 0 )];
		axleDirections[1] = worldAxles[rayWheel( 1 )];
	}

	const float flInverseMass = m_pBody->GetInvMass();
	const b3Vec3 inverseInertia = m_pBody->GetCoreInverseInertia();
	b3Vec3 crosses[2];
	float masses[2];
	float forces[2];
	for ( int i = 0; i < 2; ++i )
	{
		crosses[i] = m_pBody->IVPToCore( b3Cross( ToFloat( axlePoints[i] - position ), axleDirections[i] ) );
		masses[i] = flInverseMass + b3Dot( crosses[i], b3Mul( inverseInertia, crosses[i] ) );
		const float flVelocity = b3Dot( axleDirections[i], m_pBody->GetCoreVelocity() ) + b3Dot( crosses[i], m_pBody->GetCoreAngularVelocity() );
		forces[i] = ( SIDE_FORCE_SCALE * flVelocity ) * flInverseTimestep;
	}

	float sideForces[VEHICLE_MAX_AXLE_COUNT] = {};
	const float flCoupling = b3Dot( axleDirections[1], axleDirections[0] ) * flInverseMass + b3Dot( b3Mul( inverseInertia, crosses[0] ), crosses[1] );
	const float flDeterminant = masses[1] * masses[0] - flCoupling * flCoupling;
	if ( flDeterminant * flDeterminant >= MIN_DETERMINANT_SQUARED )
	{
		sideForces[0] = ( masses[1] * forces[0] - flCoupling * forces[1] ) / flDeterminant;
		sideForces[1] = ( masses[0] * forces[1] - flCoupling * forces[0] ) / flDeterminant;
	}

	bool bAnyTorque = false;
	for ( int i = 0; i < m_nWheels; ++i )
		bAnyTorque = bAnyTorque || m_Wheels[i].m_flTorque != 0.0f;
	const bool bStopped = b3Length( m_pBody->GetCoreVelocity() ) < STOPPED_SPEED && !bAnyTorque;

	const float flMass = m_pBody->GetMass();
	for ( int i = 0; i < m_nWheels; ++i )
	{
		Wheel_t &wheel = m_Wheels[i];
		const float flMaxForce = wheel.m_flSuspensionForce * frictions[i];
		b3Vec3 forward = b3Cross( projectedAxles[i], directions[i] );
		NormalizeLength( forward );
		const float flForwardSpeed = b3Dot( forward, m_pBody->GetCoreVelocity() );

		float flForwardForce;
		if ( wheel.m_bFixed || bStopped )
		{
			wheel.m_flAngularVelocity = 0.0f;
			flForwardForce = ( ( flForwardSpeed * flMass ) * flInverseTimestep ) * STOPPED_FORCE_SCALE;
		}
		else
		{
			wheel.m_flAngularVelocity = flForwardSpeed * wheel.m_flInverseRadius;
			const float flResistance = flForwardSpeed >= 0.0f ? ROLLING_RESISTANCE : -ROLLING_RESISTANCE;
			flForwardForce = wheel.m_flTorque * wheel.m_flInverseRadius - ( flResistance * flMass ) * frictions[i];
		}

		float &flSideForce = sideForces[i / m_nWheelsPerAxle];
		float flUsedSideForce = flSideForce;
		const float flDemand = flSideForce * flSideForce + flForwardForce * flForwardForce;
		if ( flMaxForce * flMaxForce < flDemand )
		{
			const float flScale = std::sqrt( ( flMaxForce * flMaxForce ) / flDemand );
			flForwardForce *= flScale;
			flUsedSideForce = flSideForce * flScale;
		}
		flSideForce -= flUsedSideForce;

		wheel.m_flSpinAngle -= wheel.m_flAngularVelocity * flTimestep;
		m_pBody->ApplyDirectImpulse( contacts[i], b3MulSV( flUsedSideForce * flTimestep, projectedAxles[i] ) );
		m_pBody->ApplyDirectImpulse( contacts[i], b3MulSV( flForwardForce * flTimestep, forward ) );
	}
}

void CCarRaycastSystem::UpdateWheelPositions()
{
	RealMatrix3_t rotation;
	RealVec3_t origin;
	m_pBody->GetPose( m_pBody->GetEnvironment()->GetCurrentTime(), rotation, origin );
	const b3Vec3 bodyShift = m_pBody->GetShift();
	for ( int i = 0; i < m_nWheels; ++i )
	{
		const Wheel_t &wheel = m_Wheels[i];
		const b3Vec3 center = b3Sub( b3MulAdd( wheel.m_HardPoint, wheel.m_flHitDistance - wheel.m_flRadius, wheel.m_SpringDirection ), bodyShift );

		const b3Vec3 up = { 0.0f, -std::cos( wheel.m_flSpinAngle ), -std::sin( wheel.m_flSpinAngle ) };
		b3Vec3 x = wheel.m_Axle;
		NormalizeLength( x );
		b3Vec3 z = b3Cross( up, x );
		NormalizeLength( z );
		const b3Vec3 y = b3Cross( z, x );

		RealMatrix3_t local;
		const b3Vec3 columns[3] = { x, y, z };
		for ( int j = 0; j < 3; ++j )
		{
			const RealVec3_t column = Rotate( rotation, columns[j] );
			local.m[0][j] = column.x;
			local.m[1][j] = column.y;
			local.m[2][j] = column.z;
		}
		m_pWheels[i]->MoveTo( Rotate( rotation, center ) + origin, MatrixToQuat( local ), false );
	}
}

CAirboatRaycastSystem::CAirboatRaycastSystem( const CarSystemTemplate_t &carTemplate, IPhysicsGameTrace *pGameTrace )
: m_pBody( carTemplate.m_pBody )
, m_pGameTrace( pGameTrace )
, m_nWheels( carTemplate.m_nWheels )
, m_nWheelsPerAxle( carTemplate.m_nWheels / carTemplate.m_nAxles )
{
	m_pBody->SetCoreAngularVelocity( b3Vec3_zero );
	m_pBody->SetCoreVelocity( b3Vec3_zero );
	new CAirboatGravityController( m_pBody );
	m_pBody->AttachController( this );

	const float flGravitySign = m_pBody->GetEnvironment()->GetIVPGravity().y > 0 ? 1.0f : -1.0f;
	const b3Vec3 bodyShift = m_pBody->GetShift();
	for ( int i = 0; i < m_nWheels; ++i )
	{
		Wheel_t &wheel = m_Wheels[i];
		wheel.m_TracePoint = b3Add( carTemplate.m_TracePositions[i], bodyShift );
		wheel.m_RayDirection = { 0.0f, flGravitySign, 0.0f };
		wheel.m_flRayLength = PONTOON_RAY_LENGTH;
		wheel.m_flSpringConstant = carTemplate.m_SpringConstants[i];
		wheel.m_flSpringDamping = carTemplate.m_SpringDampings[i];
		wheel.m_flSpringDampingCompression = carTemplate.m_SpringDampingCompressions[i];
		wheel.m_flFrictionScale = 1.0f;
		wheel.m_Axle = RotateAxle( 0.0f );
	}

	DoSteering( 0.0f, false );
	for ( int i = 0; i < m_nWheels; ++i )
		carTemplate.m_pWheels[i]->EnableCollisions( false );

	m_pBody->EnableGravity( false );
	m_pBody->SetCallbackFlags( m_pBody->GetCallbackFlags() & ~CALLBACK_DO_FLUID_SIMULATION );
}

CAirboatRaycastSystem::~CAirboatRaycastSystem()
{
	m_pBody->DetachController( this );
}

float CAirboatRaycastSystem::GetBodySpeed() const
{
	return static_cast<float>( InverseRotate( m_pBody->GetCoreRotation(), m_pBody->GetCoreVelocity() ).z );
}

void CAirboatRaycastSystem::DoSteeringWheel( int nWheel, float flAngle )
{
	m_Wheels[nWheel].m_Axle = RotateAxle( flAngle );
}

void CAirboatRaycastSystem::DoSteering( float flAngle, bool bAnalog )
{
	if ( flAngle == m_flSteeringAngle )
		return;

	m_flSteeringAngle = flAngle;
	m_bAnalogSteering = bAnalog;
	m_pBody->Wake();
	for ( int i = 0; i < m_nWheelsPerAxle; ++i )
		DoSteeringWheel( i, flAngle );
}

void CAirboatRaycastSystem::ChangeWheelTorque( int, float )
{
	m_pBody->Wake();
}

void CAirboatRaycastSystem::UpdateThrottle( float flThrottle )
{
	if ( std::fabs( flThrottle ) < MIN_THROTTLE )
		m_flThrust = 0.0f;
	else if ( flThrottle > 0.0f )
		m_flThrust = THRUST_FORWARD * flThrottle;
	else
		m_flThrust = THRUST_REVERSE * flThrottle;
}

void CAirboatRaycastSystem::SetWheelFriction( int nWheel, float flFriction )
{
	m_Wheels[nWheel].m_flFrictionScale = flFriction;
}

void CAirboatRaycastSystem::GetDebugData( CarSystemDebugData_t &data ) const
{
	data = m_DebugData;
}

float CAirboatRaycastSystem::GetWave( int nWheel, float flSpeedFactor ) const
{
	const float flAmplitude = std::clamp( 1.0f - flSpeedFactor, 0.0f, 1.0f ) * WAVE_AMPLITUDE + WAVE_MIN_AMPLITUDE;
	const float flPhase = flSpeedFactor < WAVE_PHASE_SPEED ? static_cast<float>( nWheel ) * WAVE_RATE : 0.0f;
	const double flTime = m_pBody->GetEnvironment()->GetCurrentTime();
	return static_cast<float>( std::sin( flTime * WAVE_RATE + static_cast<double>( flPhase * WAVE_RATE ) ) * flAmplitude );
}

void CAirboatRaycastSystem::TracePontoons( Impact_t *pImpacts, b3Vec3 localVelocity, float flSpeed )
{
	float flThrustFactor = std::clamp( localVelocity.z * 0.1f, 0.0f, 1.0f );
	if ( m_flThrust == 0.0f )
		flThrustFactor *= 0.5f;
	const float flSpeedFactor = std::clamp( flSpeed / WAVE_SPEED, 0.0f, 1.0f );

	Vector starts[VEHICLE_MAX_WHEEL_COUNT];
	Vector directions[VEHICLE_MAX_WHEEL_COUNT];
	Ray_t rays[VEHICLE_MAX_WHEEL_COUNT];
	int nFrontInWater = 0;
	for ( int i = 0; i < m_nWheels; ++i )
	{
		Impact_t &impact = pImpacts[i];
		Wheel_t &wheel = m_Wheels[i];
		wheel.m_flRayLength = PONTOON_RAY_LENGTH;

		starts[i] = IVPToSourcePosition( impact.m_Start );
		directions[i] = ToSource( IVPToSourceDirection( impact.m_Direction ) );
		impact.m_bSubmerged = m_pGameTrace->VehiclePointInWater( starts[i] );
		if ( impact.m_bSubmerged )
			directions[i] = -directions[i];

		Vector end = starts[i] + directions[i] * ( IVP_TO_SOURCE * PONTOON_RAY_LENGTH );
		if ( m_pGameTrace->VehiclePointInWater( end ) )
		{
			wheel.m_flRayLength = PONTOON_WATER_RAY_LENGTH;
			if ( i < 2 )
			{
				++nFrontInWater;
				wheel.m_flRayLength = GetWave( i, flSpeedFactor ) + PONTOON_WATER_RAY_LENGTH;
			}
			else
			{
				end = starts[i] + directions[i] * PONTOON_DEEP_RAY_LENGTH;
			}
		}
		rays[i] = MakeRay( starts[i], end - starts[i] );
	}

	if ( nFrontInWater == 2 )
	{
		const float flLength = RemapValClamped( flThrustFactor, 0.0f, 1.0f, PONTOON_WATER_RAY_LENGTH, PONTOON_RAY_LENGTH );
		for ( int i = 0; i < 2; ++i )
		{
			m_Wheels[i].m_flRayLength = GetWave( i, flSpeedFactor ) + flLength;
			rays[i] = MakeRay( starts[i], directions[i] * m_Wheels[i].m_flRayLength * IVP_TO_SOURCE );
		}
	}

	void *pVehicle = m_pBody->GetGameData();
	for ( int i = 0; i < m_nWheels; ++i )
	{
		Impact_t &impact = pImpacts[i];
		trace_t trace;
		if ( !impact.m_bSubmerged )
		{
			m_pGameTrace->VehicleTraceRayWithWater( rays[i], pVehicle, &trace );
		}
		else
		{
			m_pGameTrace->VehicleTraceRay( rays[i], pVehicle, &trace );
			trace_t depth;
			m_pGameTrace->VehicleTraceRayWithWater( MakeRay( starts[i], Vector( 0.0f, 0.0f, ( starts[i].z + WATER_DEPTH_PROBE ) - starts[i].z ) ), pVehicle, &depth );
			impact.m_flWaterDepth = depth.fractionleftsolid * WATER_DEPTH_PROBE;
		}

		const Vector end = rays[i].m_Start + rays[i].m_Delta;
		m_DebugData.m_WheelRaycasts[i][0] = ToFloat( impact.m_Start );
		m_DebugData.m_WheelRaycasts[i][1] = SourceToIVP( end );
		m_DebugData.m_WheelRaycastFractions[i] = Vector( rays[i].m_Delta ).Length() * trace.fraction;

		impact.m_bHit = trace.fraction != 1.0f;
		if ( !impact.m_bHit )
			continue;

		impact.m_flWaterDepth = 0.0f;
		impact.m_bInWater = ( trace.contents & MASK_WATER ) != 0;
		impact.m_HitPoint = ToReal( SourceToIVP( trace.endpos ) );
		impact.m_Normal = SourceToIVPDirection( ToBox3D( trace.plane.normal ) );
		impact.m_nSurfaceProps = trace.surface.surfaceProps;
		impact.m_flSurfaceFriction = GetSurfaceFriction( trace.surface.surfaceProps );
	}
}

void CAirboatRaycastSystem::ApplyDrag( const Impact_t *pImpacts, b3Vec3 localVelocity, float flSpeed, float flTimestep )
{
	int nGroundHits = 0;
	int nWaterHits = 0;
	float flFriction = 0.0f;
	Vector point( 0.0f, 0.0f, 0.0f );
	Vector normal( 0.0f, 0.0f, 0.0f );
	int surfaces[VEHICLE_MAX_WHEEL_COUNT];
	int surfaceCounts[VEHICLE_MAX_WHEEL_COUNT];
	int nSurfaces = 0;
	for ( int i = 0; i < m_nWheels; ++i )
	{
		const Impact_t &impact = pImpacts[i];
		if ( !impact.m_bHit )
			continue;

		if ( impact.m_bInWater )
		{
			++nWaterHits;
			continue;
		}

		flFriction += impact.m_flFriction;
		++nGroundHits;
		point += IVPToSourcePosition( impact.m_ContactPoint );
		normal += ToSource( IVPToSourceDirection( impact.m_Normal ) );

		int j = 0;
		while ( j < nSurfaces && surfaces[j] != impact.m_nSurfaceProps )
			++j;
		if ( j == nSurfaces )
		{
			surfaces[nSurfaces] = impact.m_nSurfaceProps;
			surfaceCounts[nSurfaces++] = 0;
		}
		++surfaceCounts[j];
	}

	const float flMass = m_pBody->GetMass();
	if ( nWaterHits > 0 )
	{
		const float flScale = ( flSpeed * flMass ) * flTimestep;
		const b3Vec3 impulse = { ( localVelocity.x * WATER_DRAG_LATERAL ) * flScale, ( localVelocity.y * WATER_DRAG_VERTICAL ) * flScale, ( localVelocity.z * WATER_DRAG_FORWARD ) * flScale };
		AddSpeed( m_pBody, m_pBody->CoreToIVP( impulse ) );
	}

	if ( nGroundHits == 0 || !( flSpeed > 0.0f ) )
		return;

	const float flScale = ( ( ( flFriction / static_cast<float>( nGroundHits ) ) * flMass * FRICTION_SPEED ) / flSpeed ) * flTimestep;
	const float flEnergyBefore = m_pBody->GetEnergy();
	const b3Vec3 impulse = { -( GROUND_DRAG_LATERAL * localVelocity.x ) * flScale, static_cast<float>( static_cast<double>( -localVelocity.y ) * GROUND_DRAG_VERTICAL ) * flScale, -( flScale * localVelocity.z ) };
	AddSpeed( m_pBody, m_pBody->CoreToIVP( impulse ) );
	const float flEnergyAfter = m_pBody->GetEnergy();

	IPhysicsCollisionEvent *pEvents = m_pBody->GetEnvironment()->GetCollisionEventHandler();
	if ( !( m_pBody->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) || !pEvents )
		return;

	const float flEnergy = ( flTimestep / flMass ) * ( flEnergyBefore - flEnergyAfter );
	if ( !( flEnergy > FRICTION_ENERGY_THRESHOLD ) )
		return;

	int nSurface = 0;
	for ( int j = 1; j < nSurfaces; ++j )
	{
		if ( surfaceCounts[j] > surfaceCounts[nSurface] )
			nSurface = j;
	}

	const float flInverseCount = 1.0f / static_cast<float>( nGroundHits );
	normal *= FastInverseSquareRoot( normal.LengthSqr() + AXIS_LENGTH_BIAS );
	const b3Vec3 velocity = m_pBody->GetCoreVelocity();
	CCollisionData data( point * flInverseCount, normal, ToSource( IVPToSourceDirection( velocity ) ) );
	pEvents->Friction( m_pBody, flEnergy, m_pBody->GetMaterialIndex(), surfaces[nSurface], &data );
}

void CAirboatRaycastSystem::ApplySteering( b3Vec3 localVelocity, float flTimestep, float flInverseTimestep )
{
	const float flSteering = m_flSteeringAngle;
	if ( flSteering == 0.0f || m_flThrust != 0.0f )
	{
		const float flReverseThrust = m_bAnalogSteering ? -ANALOG_REVERSE_THRUST : 0.0f;
		const float flForwardThrust = m_bAnalogSteering ? ANALOG_REVERSE_THRUST : 0.0f;
		if ( m_flThrust < flReverseThrust )
			m_bReverseSteering = true;
		else if ( m_flThrust > flForwardThrust || localVelocity.z > 0.0f )
			m_bReverseSteering = false;
	}

	const float flMass = m_pBody->GetMass();
	float flTorque = 0.0f;
	if ( std::fabs( flSteering ) > MIN_STEERING )
	{
		float flSign = flSteering < 0.0f ? -1.0f : 1.0f;
		if ( m_bReverseSteering )
			flSign = -flSign;
		const float flPreviousSign = m_flLastSteering < 0.0f ? -1.0f : 1.0f;
		if ( std::fabs( m_flLastSteering ) < MIN_STEERING || flSign != flPreviousSign )
			m_flSteeringHoldTime = 0.0f;

		const float flFactor = m_bAnalogSteering ? std::clamp( 2.0f * std::fabs( flSteering ), 0.0f, 1.0f ) : std::clamp( 2.0f * m_flSteeringHoldTime, 0.0f, 1.0f );
		m_flSteeringHoldTime += flTimestep;
		flTorque = -( ( flFactor * STEERING_TORQUE_SCALE + STEERING_TORQUE_BASE ) * flMass * flInverseTimestep * flSign );
	}
	m_flLastSteering = static_cast<float>( flSteering * ( m_bReverseSteering ? -1.0 : 1.0 ) );

	const float flYaw = m_pBody->GetCoreAngularVelocity().y;
	const float flYawSign = flYaw < 0.0f ? -1.0f : 1.0f;
	const float flDamping = ( ( ( std::fabs( flYaw ) * YAW_DAMPING ) * flInverseTimestep ) * flMass ) * flYawSign + ( ( ( flInverseTimestep * flMass ) * ( flYaw * flYaw ) ) * YAW_DAMPING_QUADRATIC ) * flYawSign;
	AddRotationImpulse( m_pBody, { 0.0f, -( flDamping + flTorque ), 0.0f } );
}

void CAirboatRaycastSystem::ApplyRighting( int nContacts, float flInverseTimestep )
{
	constexpr b3Vec3 target = { 0.0f, -RIGHTING_COS, RIGHTING_SIN };
	const float flMass = m_pBody->GetMass();
	const b3Vec3 up = m_pBody->IVPToCore( { 0.0f, -1.0f, 0.0f } );

	const auto measure = [&]( b3Vec3 local, b3Vec3 &axis ) {
		NormalizeLength( local );
		axis = b3Cross( target, local );
		const float flLength = NormalizeLength( axis );
		return std::atan2( flLength, b3Dot( local, target ) );
	};

	const auto push = [&]( b3Vec3 axis, float flTorque, float flMaxTorque ) {
		b3Vec3 impulse = b3MulSV( flTorque, axis );
		const float flLength = b3Length( impulse );
		if ( flLength > flMaxTorque )
			impulse = b3MulSV( flMaxTorque / flLength, impulse );
		AddRotationImpulse( m_pBody, impulse );
	};

	b3Vec3 axis;
	const float flPitch = measure( { 0.0f, up.y, up.z }, axis );
	if ( nContacts == 0 )
		push( axis, ( ( flPitch - m_flPreviousPitch ) * flInverseTimestep * PITCH_RIGHTING_RATE + flPitch * PITCH_RIGHTING_STIFFNESS ) * flMass, flMass * PITCH_RIGHTING_MAX_TORQUE );
	m_flPreviousPitch = flPitch;

	const float flRoll = measure( { up.x, -RIGHTING_COS, up.z }, axis );
	if ( nContacts == 0 && !( std::fabs( static_cast<double>( flRoll ) ) < ROLL_RIGHTING_ANGLE ) )
		push( axis, ( ( flRoll - m_flPreviousRoll ) * flInverseTimestep * ROLL_RIGHTING_RATE + flRoll * ROLL_RIGHTING_STIFFNESS ) * flMass, flMass * ROLL_RIGHTING_MAX_TORQUE );
	m_flPreviousRoll = flRoll;
}

void CAirboatRaycastSystem::Simulate( CPhysicsObject *, float flTimestep )
{
	CPhysicsEnvironment *pEnvironment = m_pBody->GetEnvironment();
	const float flInverseTimestep = pEnvironment->GetInverseTimestep();
	const RealMatrix3_t &rotation = m_pBody->GetCoreRotation();
	const b3Vec3 velocity = m_pBody->GetCoreVelocity();
	const float flSpeed = b3Length( velocity );
	const b3Vec3 localVelocity = m_pBody->IVPToCore( velocity );

	Impact_t impacts[VEHICLE_MAX_WHEEL_COUNT] = {};
	for ( int i = 0; i < m_nWheels; ++i )
	{
		impacts[i].m_Start = m_pBody->TransformCorePoint( m_Wheels[i].m_TracePoint );
		impacts[i].m_Direction = ToFloat( Rotate( rotation, m_Wheels[i].m_RayDirection ) );
	}
	TracePontoons( impacts, localVelocity, flSpeed );

	int nContacts = 0;
	for ( int i = 0; i < m_nWheels; ++i )
	{
		Impact_t &impact = impacts[i];
		const Wheel_t &wheel = m_Wheels[i];
		if ( impact.m_bSubmerged )
			impact.m_Direction = b3Neg( impact.m_Direction );

		if ( !impact.m_bHit )
		{
			impact.m_flHitLength = wheel.m_flRayLength;
			impact.m_flInverseNormalFactor = 1.0f;
			impact.m_Normal = b3Neg( impact.m_Direction );
			impact.m_flFriction = 1.0f;
		}
		else
		{
			++nContacts;
			impact.m_flHitLength = static_cast<float>( Length( impact.m_HitPoint - impact.m_Start ) );
			impact.m_flInverseNormalFactor = IMPACT_NORMAL_FACTOR / ( std::fabs( b3Dot( impact.m_Normal, impact.m_Direction ) ) + IMPACT_NORMAL_BIAS );
			impact.m_flFriction = impact.m_flSurfaceFriction * wheel.m_flFrictionScale;
		}

		impact.m_ContactPoint = impact.m_Start + real_t( impact.m_flHitLength ) * ToReal( impact.m_Direction );
		impact.m_ContactVelocity = GetPointVelocity( m_pBody, impact.m_ContactPoint );
		impact.m_TangentVelocity = Tangential( impact.m_ContactVelocity, impact.m_Normal );
		b3Vec3 projectedAxle = Tangential( ToFloat( Rotate( rotation, wheel.m_Axle ) ), impact.m_Normal );
		if ( NormalizeLength( projectedAxle ) < MIN_LENGTH )
		{
			DevMsg( "CPhysics_Airboat::do_simulation_controller projected_axis_direction_ws.normize failed\n" );
			return;
		}
	}

	if ( nContacts == 0 )
	{
		if ( !m_bInAir )
		{
			m_bInAir = true;
			m_flAirTime = 0.0f;
			m_bLowSpeedAir = flSpeed < LOW_AIR_SPEED;
		}
		else
		{
			m_flAirTime += flTimestep;
		}
	}
	else
	{
		m_bInAir = false;
		m_bLowSpeedAir = false;
	}

	const float flMass = m_pBody->GetMass();
	for ( int i = 0; i < m_nWheels; ++i )
	{
		const Impact_t &impact = impacts[i];
		const Wheel_t &wheel = m_Wheels[i];
		if ( !impact.m_bHit )
		{
			if ( impact.m_bSubmerged )
			{
				const float flDepth = std::clamp( impact.m_flWaterDepth, 0.0f, PONTOON_MAX_DEPTH );
				m_pBody->ApplyDirectImpulse( impact.m_ContactPoint, { 0.0f, ( ( flDepth * flMass ) * flTimestep ) * PONTOON_BUOYANCY, 0.0f } );
			}
			continue;
		}

		if ( !( impact.m_flHitLength < wheel.m_flRayLength ) )
			continue;

		const float flNormalFactor = std::clamp( impact.m_flInverseNormalFactor, 0.0f, MAX_NORMAL_FACTOR );
		const float flRelative = b3Dot( b3Sub( impact.m_TangentVelocity, impact.m_ContactVelocity ), impact.m_Direction );
		const float flDamping = flRelative <= 0.0f ? wheel.m_flSpringDampingCompression : wheel.m_flSpringDamping;
		const float flForce = std::max( -( ( flNormalFactor * ( impact.m_flHitLength - wheel.m_flRayLength ) ) * wheel.m_flSpringConstant ) - flRelative * flDamping, 0.0f );
		m_pBody->ApplyDirectImpulse( impact.m_ContactPoint, b3MulSV( flForce * flTimestep, impact.m_Normal ) );
	}

	ApplyDrag( impacts, localVelocity, flSpeed, flTimestep );

	float flThrust = m_flThrust;
	if ( m_bLowSpeedAir || ( m_bInAir && flThrust < 0.0f ) )
		flThrust *= 0.5f;
	const float flPitch = static_cast<float>( rotation.m[1][2] );
	if ( flPitch < -0.5f && flThrust > 0.0f )
		flThrust *= 1.0f + flPitch;
	if ( flPitch > 0.5f && flThrust < 0.0f )
		flThrust *= 1.0f - flPitch;
	const b3Vec3 forward = { static_cast<float>( rotation.m[0][2] ), static_cast<float>( rotation.m[1][2] ), static_cast<float>( rotation.m[2][2] ) };
	AddSpeed( m_pBody, b3MulSV( flThrust * flMass * flTimestep, forward ) );

	ApplySteering( localVelocity, flTimestep, flInverseTimestep );
	if ( !m_bLowSpeedAir )
		ApplyRighting( nContacts, flInverseTimestep );
}
