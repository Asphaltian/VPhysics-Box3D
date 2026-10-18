#include "physics_vehicle_controller.h"

#include "conversion.h"
#include "physics_car_system.h"
#include "physics_environment.h"
#include "physics_object.h"
#include "vphysics_saverestore.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

static constexpr float MPH_TO_METERS_PER_SECOND = 0.44707f;
static constexpr float METERS_PER_SECOND_TO_MPH = 2.2367861f;
static constexpr float POWERSLIDE_SPEED = 18.0f;
static constexpr float IDLE_BRAKE = 0.1f;
static constexpr float REVOLUTIONS_PER_RADIAN = 0.15915494f;
static constexpr float SECONDS_PER_MINUTE = 60.0f;
static constexpr float ENGINE_POWER_SCALE = 44700.0f;
static constexpr double REVOLUTIONS_PER_RADIAN_PRECISE = 0.15915494309189535;
static constexpr float SPEED_LIMIT_FLOOR = 1.0f;
static constexpr float OVERSPEED_THROTTLE_SCALE = 0.1f;
static constexpr float BRAKE_TORQUE_SCALE = 0.5f;
static constexpr float AXLE_TORQUE_SCALE = 0.5f;
static constexpr float BOOST_SPEED_FLOOR = 0.1f;
static constexpr float BOOST_SPEED_RANGE = 0.9f;
static constexpr float BOOST_STEERING_REDUCTION = 0.95f;
static constexpr float BOOST_TIME_SCALE = 100.0f;
static constexpr float WHEEL_LOCK_SPEED = 5.0f;
static constexpr float HANDBRAKE_SKID_SPEED = 30.0f;
static constexpr double TILT_UPRIGHT = 0.05;
static constexpr float ANGULAR_SPEED_BIAS = 1e-10f;
static constexpr int WHEEL_TORQUES_ZEROED = 1 << 0;

CPhysicsVehicleController::CPhysicsVehicleController( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace )
: m_pEnvironment( pEnvironment )
, m_pGameTrace( pGameTrace )
, m_VehicleParams( params )
, m_nVehicleType( nVehicleType )
, m_bTraceData( nVehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST )
{
	VehicleDataReload();
	m_flTorqueScale = 1.0f;

	m_pBody = pBody;
	m_flBodyMass = pBody->GetMass();
	m_flGravityLength = static_cast<float>( Length( pEnvironment->GetIVPGravity() ) );
	m_nWheelCount = m_VehicleParams.axleCount * m_VehicleParams.wheelsPerAxle;
	CreateCarSystem();

	if ( m_nVehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST )
	{
		const float flDamping = 1.0f;
		pBody->SetDamping( &flDamping, &flDamping );
	}
}

CPhysicsVehicleController::CPhysicsVehicleController( CPhysicsEnvironment *pEnvironment, IPhysicsGameTrace *pGameTrace, const vphysics_save_cvehiclecontroller_t &record )
: m_pBody( static_cast<CPhysicsObject *>( record.m_pCarBody ) )
, m_pEnvironment( pEnvironment )
, m_pGameTrace( pGameTrace )
, m_nWheelCount( record.m_wheelCount )
, m_VehicleParams( record.m_vehicleData )
, m_OperatingParams( record.m_currentState )
, m_flBodyMass( record.m_bodyMass )
, m_flTotalWheelMass( record.m_totalWheelMass )
, m_flGravityLength( record.m_gravityLength )
, m_flTorqueScale( record.m_torqueScale )
, m_nTireType( record.m_nTireType )
, m_nVehicleType( static_cast<unsigned int>( record.m_nVehicleType ) )
, m_bWheelTorquesZeroed( ( record.m_vehicleFlags & WHEEL_TORQUES_ZEROED ) != 0 )
, m_bTraceData( record.m_bTraceData )
, m_bOccupied( record.m_bOccupied )
, m_bEngineDisabled( record.m_bEngineDisable )
{
	ApplyRestoredDefaults();
	for ( int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; ++i )
	{
		m_pWheels[i] = static_cast<CPhysicsObject *>( record.m_pWheels[i] );
		m_WheelPositions[i] = SourceToIVP( record.m_wheelPosition_Bs[i] );
		m_TracePositions[i] = SourceToIVP( record.m_tracePosition_Bs[i] );
	}

	if ( !m_pBody )
		return;

	CreateCarSystem();
	const AngularImpulse zero( 0.0f, 0.0f, 0.0f );
	m_pBody->SetVelocity( nullptr, &zero );
	m_pBody->SetPendingVelocity( { record.m_flVelocity[0], record.m_flVelocity[1], record.m_flVelocity[2] } );
}

void CPhysicsVehicleController::ApplyRestoredDefaults()
{
	if ( m_flTorqueScale == 0.0f )
		m_flTorqueScale = 1.0f;

	vehicle_steeringparams_t &steering = m_VehicleParams.steering;
	vehicle_engineparams_t &engine = m_VehicleParams.engine;
	const auto setDefault = []( float &flValue, float flDefault ) {
		if ( flValue == 0.0f )
			flValue = flDefault;
	};
	setDefault( steering.steeringRateSlow, 4.5f );
	setDefault( steering.steeringRateFast, 0.5f );
	setDefault( steering.steeringRestRateSlow, 3.0f );
	setDefault( steering.steeringRestRateFast, 1.8f );
	setDefault( steering.speedSlow, engine.maxSpeed * 0.25f );
	setDefault( steering.speedFast, engine.maxSpeed * 0.75f );
	setDefault( steering.degreesSlow, 50.0f );
	setDefault( steering.degreesFast, 18.0f );
	setDefault( steering.degreesBoost, 10.0f );
	setDefault( steering.turnThrottleReduceSlow, 0.3f );
	setDefault( steering.turnThrottleReduceFast, 3.0f );
	setDefault( steering.brakeSteeringRateFactor, 6.0f );
	setDefault( steering.throttleSteeringRestRateFactor, 2.0f );
	setDefault( steering.boostSteeringRestRateFactor, 1.0f );
	setDefault( steering.boostSteeringRateFactor, 1.0f );
	setDefault( steering.powerSlideAccel, 200.0f );
	setDefault( engine.autobrakeSpeedGain, 1.0f );
	setDefault( engine.autobrakeSpeedFactor, 2.0f );
}

void CPhysicsVehicleController::WriteRecord( vphysics_save_cvehiclecontroller_t &record )
{
	m_bWheelsFixed = false;
	record.m_pCarBody = m_pBody;
	record.m_wheelCount = m_nWheelCount;
	record.m_vehicleData = m_VehicleParams;
	record.m_currentState = m_OperatingParams;
	record.m_bodyMass = m_flBodyMass;
	record.m_totalWheelMass = m_flTotalWheelMass;
	record.m_gravityLength = m_flGravityLength;
	record.m_torqueScale = m_flTorqueScale;
	for ( int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; ++i )
	{
		record.m_pWheels[i] = m_pWheels[i];
		record.m_wheelPosition_Bs[i] = ToSource( IVPToSource( m_WheelPositions[i] ) );
		record.m_tracePosition_Bs[i] = ToSource( IVPToSource( m_TracePositions[i] ) );
	}
	record.m_vehicleFlags = m_bWheelTorquesZeroed ? WHEEL_TORQUES_ZEROED : 0;
	record.m_nTireType = m_nTireType;
	record.m_nVehicleType = static_cast<int>( m_nVehicleType );
	record.m_bTraceData = m_bTraceData;
	record.m_bOccupied = m_bOccupied;
	record.m_bEngineDisable = m_bEngineDisabled;
	if ( m_pBody )
	{
		const b3Vec3 velocity = m_pBody->GetCoreVelocity();
		record.m_flVelocity[0] = velocity.x;
		record.m_flVelocity[1] = velocity.y;
		record.m_flVelocity[2] = velocity.z;
	}
}

CPhysicsVehicleController::~CPhysicsVehicleController()
{
	ShutDown();
	if ( m_pBody )
		m_pBody->SetVehicleController( nullptr );
}

int CPhysicsVehicleController::GetCreatedWheelCount() const
{
	return std::min( m_nWheelCount, VEHICLE_MAX_WHEEL_COUNT );
}

void CPhysicsVehicleController::VehicleDataReload()
{
	m_flTorqueScale = 1.0f;
	float flTorqueSum = 0.0f;
	for ( int i = 0; i < m_VehicleParams.axleCount; ++i )
		flTorqueSum += m_VehicleParams.axles[i].torqueFactor;
	if ( flTorqueSum > 0.0f )
		m_flTorqueScale = 1.0f / flTorqueSum;

	vehicle_engineparams_t &engine = m_VehicleParams.engine;
	engine.maxSpeed = ( engine.maxSpeed * MPH_TO_METERS_PER_SECOND ) * IVP_TO_SOURCE;
	engine.maxRevSpeed = ( engine.maxRevSpeed * MPH_TO_METERS_PER_SECOND ) * IVP_TO_SOURCE;
	engine.boostMaxSpeed = ( engine.boostMaxSpeed * MPH_TO_METERS_PER_SECOND ) * IVP_TO_SOURCE;
}

void CPhysicsVehicleController::CreateWheels( CarSystemTemplate_t &carTemplate )
{
	Vector origin;
	QAngle angles;
	m_pBody->GetPosition( &origin, &angles );
	matrix3x4_t matrix;
	AngleMatrix( angles, origin, matrix );

	const DoubleMatrix3_t &objectRotation = m_pBody->GetCoreRotation();
	const DoubleVec3_t objectOrigin = m_pBody->GetCoreOrigin();
	const auto toObject = [&]( const Vector &local ) {
		Vector world;
		VectorTransform( local, matrix, world );
		return std::pair( world, ToFloat( InverseRotate( objectRotation, ToDouble( SourceToIVP( world ) ) - objectOrigin ) ) );
	};

	int nWheel = 0;
	for ( int i = 0; i < m_VehicleParams.axleCount; ++i )
	{
		const vehicle_axleparams_t &axle = m_VehicleParams.axles[i];
		for ( int j = 0; j < m_VehicleParams.wheelsPerAxle; ++j, ++nWheel )
		{
			if ( nWheel >= VEHICLE_MAX_WHEEL_COUNT )
				continue;

			if ( m_pWheels[nWheel] )
			{
				m_pWheels[nWheel]->SetVehicleController( this );
				carTemplate.m_pWheels[nWheel] = m_pWheels[nWheel];
				carTemplate.m_WheelPositions[nWheel] = m_WheelPositions[nWheel];
				carTemplate.m_TracePositions[nWheel] = m_TracePositions[nWheel];
				continue;
			}

			const bool bOdd = ( nWheel & 1 ) != 0;
			const auto [world, position] = toObject( bOdd ? axle.offset + axle.wheelOffset : axle.offset - axle.wheelOffset );

			const float flRadius = axle.wheels.radius;
			objectparams_t params = {};
			params.mass = axle.wheels.mass;
			params.inertia = axle.wheels.inertia;
			params.damping = axle.wheels.damping;
			params.rotdamping = axle.wheels.rotdamping;
			params.pName = "VehicleWheel";
			params.pGameData = m_pBody->GetGameData();
			params.volume = static_cast<float>( static_cast<double>( ( flRadius * flRadius ) * flRadius ) * std::numbers::pi );

			CPhysicsObject *pWheel = static_cast<CPhysicsObject *>( m_pEnvironment->CreateSphereObject( flRadius, axle.wheels.materialIndex, world, angles, &params, false ) );
			pWheel->Wake();
			pWheel->SetCallbackFlags( 0 );
			pWheel->SetGameFlags( m_pBody->GetGameFlags() );
			pWheel->SetVehicleController( this );
			m_pWheels[nWheel] = pWheel;
			m_WheelPositions[nWheel] = position;
			pWheel->SetCallbackFlags( pWheel->GetCallbackFlags() | CALLBACK_IS_VEHICLE_WHEEL );

			if ( m_bTraceData )
				m_TracePositions[nWheel] = toObject( bOdd ? axle.raytraceCenterOffset + axle.raytraceOffset : axle.raytraceCenterOffset - axle.raytraceOffset ).second;

			carTemplate.m_pWheels[nWheel] = pWheel;
			carTemplate.m_WheelPositions[nWheel] = m_WheelPositions[nWheel];
			carTemplate.m_TracePositions[nWheel] = m_TracePositions[nWheel];
		}
	}

	for ( int i = 0; i < GetCreatedWheelCount(); ++i )
	{
		if ( m_pWheels[i] )
			m_pWheels[i]->EnableCollisions( true );
	}
}

void CPhysicsVehicleController::CreateCarSystem()
{
	CarSystemTemplate_t carTemplate = {};
	carTemplate.m_nWheels = GetCreatedWheelCount();
	carTemplate.m_nAxles = m_VehicleParams.axleCount;
	carTemplate.m_pBody = m_pBody;
	CreateWheels( carTemplate );

	const vehicle_bodyparams_t &body = m_VehicleParams.body;
	const vehicle_engineparams_t &engine = m_VehicleParams.engine;
	carTemplate.m_flCounterTorqueFactor = body.counterTorqueFactor;
	carTemplate.m_flExtraGravityForce = ( m_flGravityLength * body.addGravity ) * m_flBodyMass;
	carTemplate.m_flDownforceHeight = body.tiltForceHeight * SOURCE_TO_IVP;
	if ( carTemplate.m_nWheels == 0 )
		return;

	m_flMaxWheelRadius = 0.0f;
	m_flTotalWheelMass = 0.0f;
	int nWheel = 0;
	for ( int i = 0; i < m_VehicleParams.axleCount; ++i )
	{
		const vehicle_axleparams_t &axle = m_VehicleParams.axles[i];
		for ( int j = 0; j < m_VehicleParams.wheelsPerAxle && nWheel < VEHICLE_MAX_WHEEL_COUNT; ++j, ++nWheel )
		{
			carTemplate.m_SpringConstants[nWheel] = axle.suspension.springConstant * m_flBodyMass;
			carTemplate.m_SpringDampings[nWheel] = axle.suspension.springDamping * m_flBodyMass;
			carTemplate.m_SpringDampingCompressions[nWheel] = axle.suspension.springDampingCompression * m_flBodyMass;
			carTemplate.m_MaxBodyForces[nWheel] = axle.suspension.maxBodyForce * m_flBodyMass;
			carTemplate.m_SpringAdditionalLengths[nWheel] = axle.wheels.springAdditionalLength * SOURCE_TO_IVP;
			m_flTotalWheelMass += axle.wheels.mass;
		}
		carTemplate.m_StabilizerConstants[i] = axle.suspension.stabilizerConstant * m_flBodyMass;
		carTemplate.m_MaxWheelRotationSpeeds[i] = std::max( engine.maxSpeed, engine.boostMaxSpeed ) / ( axle.wheels.radius * SOURCE_TO_IVP );
		m_flMaxWheelRadius = std::max( m_flMaxWheelRadius, axle.wheels.radius * SOURCE_TO_IVP );
	}

	switch ( m_nVehicleType )
	{
	case VEHICLE_TYPE_CAR_WHEELS:
		m_pCarSystem = new CCarWheelsSystem( carTemplate );
		break;
	case VEHICLE_TYPE_CAR_RAYCAST:
		m_pCarSystem = new CCarRaycastSystem( carTemplate );
		break;
	case VEHICLE_TYPE_AIRBOAT_RAYCAST:
		m_pCarSystem = new CAirboatRaycastSystem( carTemplate, m_pGameTrace );
		break;
	default:
		return;
	}
	m_pBody->SetVehicleController( this );
}

void CPhysicsVehicleController::ShutDown()
{
	delete m_pCarSystem;
	m_pCarSystem = nullptr;
	for ( int i = 0; i < GetCreatedWheelCount(); ++i )
	{
		CPhysicsObject *pWheel = m_pWheels[i];
		m_pWheels[i] = nullptr;
		if ( !pWheel )
			continue;

		pWheel->SetVehicleController( nullptr );
		m_pEnvironment->DestroyObject( pWheel );
	}
}

void CPhysicsVehicleController::ObjectDestroyed( CPhysicsObject *pObject )
{
	if ( pObject != m_pBody )
	{
		std::replace( std::begin( m_pWheels ), std::end( m_pWheels ), pObject, static_cast<CPhysicsObject *>( nullptr ) );
		return;
	}

	ShutDown();
	m_pBody = nullptr;
}

bool CPhysicsVehicleController::IsBoosting() const
{
	return m_pCarSystem->GetBoosterTimeToGo() > 0.0f;
}

float CPhysicsVehicleController::GetSteeringAngle( float flSpeed, float flSteering, bool bAnalog ) const
{
	const vehicle_steeringparams_t &steering = m_VehicleParams.steering;
	const vehicle_engineparams_t &engine = m_VehicleParams.engine;

	float flMaxAngle;
	if ( steering.speedSlow == steering.speedFast )
	{
		flMaxAngle = flSpeed >= steering.speedFast ? steering.degreesFast : steering.degreesSlow;
	}
	else
	{
		const float flFraction = std::clamp( ( flSpeed - steering.speedSlow ) / ( steering.speedFast - steering.speedSlow ), 0.0f, 1.0f );
		flMaxAngle = flFraction * ( steering.degreesFast - steering.degreesSlow ) + steering.degreesSlow;
	}

	const float flVelocity = ( flSpeed * MPH_TO_METERS_PER_SECOND ) * IVP_TO_SOURCE;
	if ( flVelocity > engine.maxSpeed )
		flMaxAngle = RemapValClamped( flVelocity, engine.maxSpeed, engine.boostMaxSpeed, steering.degreesFast, steering.degreesBoost );

	if ( steering.steeringExponent == 0.0f )
		return flMaxAngle * flSteering;

	const double flSign = flSteering < 0.0f ? -1.0 : 1.0;
	if ( bAnalog )
		return std::clamp( static_cast<float>( ( ( steering.degreesSlow * flSign ) * flSteering ) * flSteering ), -flMaxAngle, flMaxAngle );

	return static_cast<float>( std::pow( static_cast<double>( std::fabs( flSteering ) ), static_cast<double>( steering.steeringExponent ) ) * flSign * flMaxAngle );
}

void CPhysicsVehicleController::UpdateTireMaterials( const vehicle_controlparams_t &controls, bool bPowerslide, float flSpeed )
{
	const vehicle_steeringparams_t &steering = m_VehicleParams.steering;
	if ( !steering.isSkidAllowed || !m_bOccupied )
		return;

	const bool bLeft = bPowerslide && controls.handbrakeLeft;
	const bool bRight = bPowerslide && controls.handbrakeRight;
	const int nTireType = ( bLeft || bRight ) ? VEHICLE_TIRE_POWERSLIDE : ( bPowerslide ? VEHICLE_TIRE_BRAKING : VEHICLE_TIRE_NORMAL );
	if ( nTireType != m_nTireType )
	{
		int nWheel = 0;
		for ( int i = 0; i < m_VehicleParams.axleCount; ++i )
		{
			const vehicle_wheelparams_t &wheels = m_VehicleParams.axles[i].wheels;
			int nMaterial = wheels.materialIndex;
			if ( nTireType == VEHICLE_TIRE_POWERSLIDE && wheels.skidMaterialIndex != -1 )
				nMaterial = wheels.skidMaterialIndex;
			else if ( nTireType == VEHICLE_TIRE_BRAKING && wheels.brakeMaterialIndex != -1 )
				nMaterial = wheels.brakeMaterialIndex;

			for ( int j = 0; j < m_VehicleParams.wheelsPerAxle; ++j, ++nWheel )
			{
				if ( nWheel < VEHICLE_MAX_WHEEL_COUNT && m_pWheels[nWheel] )
					m_pWheels[nWheel]->SetMaterialIndex( nMaterial );
			}
			m_nTireType = nTireType;
		}
	}

	float flFront = 0.0f;
	float flRear = 0.0f;
	if ( flSpeed > 0.0f && bLeft != bRight )
	{
		const float flFraction = RemapValClamped( flSpeed, steering.speedSlow, steering.speedFast, 0.0f, 1.0f );
		const float flAcceleration = SOURCE_TO_IVP * steering.powerSlideAccel;
		if ( bRight )
		{
			flRear = flAcceleration * flFraction;
			flFront = -flRear;
		}
		else
		{
			flFront = flAcceleration * flFraction;
			flRear = -flFront;
		}
	}
	m_pCarSystem->SetPowerslide( flFront, flRear );
}

void CPhysicsVehicleController::SetAxleTorque( int nAxle, float flTorque )
{
	const int nFirst = nAxle * m_VehicleParams.wheelsPerAxle;
	const int nLast = std::min( nFirst + m_VehicleParams.wheelsPerAxle, GetCreatedWheelCount() );
	for ( int i = nFirst; i < nLast; ++i )
		m_pCarSystem->ChangeWheelTorque( i, flTorque );
}

float CPhysicsVehicleController::UpdateEngine( float flThrottle, float flBrake, float flSteering, bool bTorqueBoosting )
{
	const vehicle_engineparams_t &engine = m_VehicleParams.engine;
	if ( engine.isAutoTransmission )
	{
		float flWheelSpeed = 0.0f;
		for ( int i = 0; i < GetCreatedWheelCount(); ++i )
			flWheelSpeed += std::fabs( m_pCarSystem->GetWheelAngularVelocity( i ) );

		int &nGear = m_OperatingParams.gear;
		const auto getRPM = [&]() { return ( ( flWheelSpeed * ( REVOLUTIONS_PER_RADIAN / static_cast<float>( m_nWheelCount ) ) ) * engine.axleRatio ) * engine.gearRatio[nGear] * SECONDS_PER_MINUTE; };
		float flRPM = getRPM();
		if ( flThrottle > 0.0f )
		{
			while ( flRPM > engine.shiftUpRPM && nGear < engine.gearCount - 1 )
			{
				++nGear;
				flRPM = getRPM();
			}
		}
		while ( flRPM < engine.shiftDownRPM && nGear > 0 )
		{
			--nGear;
			flRPM = getRPM();
		}
		m_OperatingParams.engineRPM = flRPM;
	}

	const float flSpeedLimit = std::max( bTorqueBoosting ? engine.boostMaxSpeed : engine.maxSpeed, SPEED_LIMIT_FLOOR );
	const float flSpeed = std::fabs( m_OperatingParams.speed );
	if ( flThrottle > 0.0f && flSpeed > flSpeedLimit )
	{
		const float flRatio = flSpeed / flSpeedLimit;
		if ( flRatio > engine.autobrakeSpeedGain )
		{
			flThrottle = 0.0f;
			flBrake = ( flRatio - 1.0f ) * engine.autobrakeSpeedFactor;
			if ( m_OperatingParams.wheelsInContact == 0 )
				flBrake = 0.0f;
		}
		flThrottle *= OVERSPEED_THROTTLE_SCALE;
	}

	if ( flThrottle < 0.0f && !bTorqueBoosting && flSpeed > engine.maxRevSpeed )
		flThrottle *= OVERSPEED_THROTTLE_SCALE;

	if ( flThrottle == 0.0f )
	{
		if ( flBrake == 0.0f )
		{
			if ( !m_bWheelTorquesZeroed )
			{
				m_bWheelTorquesZeroed = true;
				for ( int i = 0; i < GetCreatedWheelCount(); ++i )
					m_pCarSystem->ChangeWheelTorque( i, 0.0f );
			}
		}
		else
		{
			m_bWheelTorquesZeroed = false;
			const float flDirection = m_OperatingParams.speed >= 0.0f ? -BRAKE_TORQUE_SCALE : BRAKE_TORQUE_SCALE;
			for ( int i = 0; i < m_VehicleParams.axleCount; ++i )
			{
				const vehicle_axleparams_t &axle = m_VehicleParams.axles[i];
				const float flTorque = ( ( ( ( flDirection * ( m_flTotalWheelMass + m_flBodyMass ) ) * flBrake ) * m_flGravityLength ) * axle.brakeFactor ) * ( SOURCE_TO_IVP * axle.wheels.radius );
				SetAxleTorque( i, flTorque );
			}
		}
	}
	else
	{
		m_bWheelTorquesZeroed = false;
		float flBaseTorque = 0.0f;
		if ( m_OperatingParams.engineRPM < engine.maxRPM )
		{
			const float flPower = ( ( ( flThrottle * engine.horsepower ) * engine.gearRatio[m_OperatingParams.gear] ) * ENGINE_POWER_SCALE ) * engine.axleRatio;
			flBaseTorque = static_cast<float>( static_cast<double>( flPower ) * ( REVOLUTIONS_PER_RADIAN_PRECISE / static_cast<double>( engine.maxRPM * m_flMaxWheelRadius ) ) );
		}

		for ( int i = 0; i < m_VehicleParams.axleCount; ++i )
		{
			const vehicle_axleparams_t &axle = m_VehicleParams.axles[i];
			float flScale = AXLE_TORQUE_SCALE;
			if ( bTorqueBoosting && IsBoosting() )
			{
				const float flBoost = engine.maxSpeed == 0.0f ? ( flSpeed >= 0.0f ? 1.0f : BOOST_SPEED_FLOOR ) : std::clamp( flSpeed / engine.maxSpeed, 0.0f, 1.0f ) * BOOST_SPEED_RANGE + BOOST_SPEED_FLOOR;
				flScale = std::max( ( 1.0f - std::fabs( flSteering ) * BOOST_STEERING_REDUCTION ) * flBoost * engine.boostForce, AXLE_TORQUE_SCALE );
			}
			SetAxleTorque( i, ( ( ( flScale * flBaseTorque ) * axle.torqueFactor ) * m_flTorqueScale ) * ( SOURCE_TO_IVP * axle.wheels.radius ) );
		}
	}

	m_pCarSystem->UpdateThrottle( flThrottle );
	return flThrottle;
}

void CPhysicsVehicleController::UpdateWheelLock( float flThrottle, bool bPowerslide, bool bHandbrake )
{
	const float flSpeed = m_OperatingParams.speed;
	bool bLock = false;
	if ( ( !bPowerslide && ( ( flThrottle < 0.0f && flSpeed > WHEEL_LOCK_SPEED ) || ( flThrottle > 0.0f && flSpeed < -WHEEL_LOCK_SPEED ) ) ) || bHandbrake )
	{
		for ( int i = 0; i < GetCreatedWheelCount() && !bLock; ++i )
			bLock = m_pWheels[i] && m_pWheels[i]->GetContactPoint( nullptr, nullptr );
	}

	if ( bLock == m_bWheelsFixed )
		return;

	m_bWheelsFixed = bLock;
	for ( int i = 0; i < GetCreatedWheelCount(); ++i )
		m_pCarSystem->FixWheel( i, bLock );
}

void CPhysicsVehicleController::UpdateContacts( bool bHandbrake )
{
	vehicle_operatingparams_t &operating = m_OperatingParams;
	operating.wheelsInContact = m_nWheelCount;
	operating.skidSpeed = 0.0f;
	operating.skidMaterial = 0;
	operating.wheelsNotInContact = 0;
	if ( !m_VehicleParams.steering.isSkidAllowed )
		return;

	operating.wheelsInContact = 0;
	for ( int i = 0; i < m_nWheelCount; ++i )
	{
		Vector point;
		int nSurface = 0;
		if ( !GetWheelContactPoint( i, &point, &nSurface ) )
		{
			++operating.wheelsNotInContact;
			continue;
		}

		Vector velocity;
		m_pWheels[i]->GetVelocityAtPoint( point, &velocity );
		const float flSkidSpeed = velocity.Length();
		if ( flSkidSpeed > operating.skidSpeed || operating.skidSpeed <= 0.0f )
		{
			operating.skidSpeed = flSkidSpeed;
			operating.skidMaterial = nSurface;
		}
		++operating.wheelsInContact;
	}

	if ( bHandbrake && std::fabs( operating.speed ) > HANDBRAKE_SKID_SPEED )
		operating.skidSpeed = std::fabs( operating.speed );
}

void CPhysicsVehicleController::UpdateBody()
{
	const vehicle_bodyparams_t &body = m_VehicleParams.body;
	DoubleMatrix3_t rotation;
	DoubleVec3_t origin;
	m_pBody->GetPose( m_pEnvironment->GetCurrentTime(), rotation, origin );
	const bool bUpright = std::fabs( static_cast<double>( static_cast<float>( rotation.m[1][1] ) ) ) >= TILT_UPRIGHT;
	m_pCarSystem->ChangeBodyDownforce( bUpright ? 0.0f : ( m_flGravityLength * body.tiltForce ) * m_flBodyMass );
	m_pCarSystem->UpdateBodyCounterTorque();

	AngularImpulse angular;
	m_pBody->GetVelocity( nullptr, &angular );
	const float flSpeedSquared = angular.LengthSqr();
	if ( body.maxAngularVelocity > 0.0f && flSpeedSquared > body.maxAngularVelocity * body.maxAngularVelocity )
	{
		angular *= FastInverseSquareRoot( flSpeedSquared + ANGULAR_SPEED_BIAS ) * body.maxAngularVelocity;
		m_pBody->SetVelocityInstantaneous( nullptr, &angular );
	}
}

void CPhysicsVehicleController::Update( float dt, vehicle_controlparams_t &controlParams )
{
	if ( !m_pCarSystem )
		return;

	vehicle_controlparams_t controls = controlParams;
	vehicle_operatingparams_t &operating = m_OperatingParams;
	const vehicle_engineparams_t &engine = m_VehicleParams.engine;

	operating.speed = m_pCarSystem->GetBodySpeed() * IVP_TO_SOURCE;
	const float flSpeed = ( operating.speed * SOURCE_TO_IVP ) * METERS_PER_SECOND_TO_MPH;
	const float flAbsoluteSpeed = std::fabs( flSpeed );
	const bool bPowerslide = controls.handbrake && flAbsoluteSpeed > POWERSLIDE_SPEED;

	if ( controls.handbrake )
		controls.throttle = 0.0f;

	if ( IsBoosting() )
	{
		controls.boost = 1.0f;
		controls.throttle = controls.throttle < 0.0f ? -1.0f : 1.0f;
	}
	else if ( controls.throttle == 0.0f && controls.brake == 0.0f && !controls.handbrake )
	{
		controls.brake = IDLE_BRAKE;
	}

	const float flSteeringAngle = GetSteeringAngle( flAbsoluteSpeed, controls.steering, controls.bAnalogSteering );
	m_pCarSystem->DoSteering( flSteeringAngle * DEGREES_TO_RADIANS, controls.bAnalogSteering );
	operating.steeringAngle = flSteeringAngle;

	UpdateTireMaterials( controls, bPowerslide, flSpeed );

	bool bTorqueBoosting = false;
	if ( controls.boost > 0.0f )
	{
		if ( !engine.torqueBoost )
		{
			controls.boost *= engine.boostForce;
		}
		else
		{
			controls.boost = 0.0f;
			bTorqueBoosting = true;
		}
		m_pCarSystem->ActivateBooster( controls.boost, engine.boostDuration, engine.boostDelay );
	}
	m_pCarSystem->UpdateBooster( dt );
	operating.boostDelay = m_pCarSystem->GetBoosterDelay();
	operating.isTorqueBoosting = bTorqueBoosting;

	const float flThrottle = UpdateEngine( controls.throttle, controls.brake, controls.steering, bTorqueBoosting );

	const float flBoostTotal = engine.boostDelay + engine.boostDuration;
	if ( flBoostTotal > 0.0f )
		operating.boostTimeLeft = operating.boostDelay <= 0.0f ? static_cast<int>( BOOST_TIME_SCALE ) : static_cast<int>( BOOST_TIME_SCALE - ( operating.boostDelay / flBoostTotal ) * BOOST_TIME_SCALE );

	operating.speed = m_pCarSystem->GetBodySpeed() * IVP_TO_SOURCE;
	UpdateWheelLock( flThrottle, bPowerslide, controls.handbrake );
	UpdateContacts( controls.handbrake );
	UpdateBody();
	m_pCarSystem->UpdateWheelPositions();
}

float CPhysicsVehicleController::UpdateBooster( float dt )
{
	if ( !m_pCarSystem )
		return m_OperatingParams.boostDelay;

	m_pCarSystem->UpdateBooster( dt );
	m_OperatingParams.boostDelay = m_pCarSystem->GetBoosterDelay();
	return m_OperatingParams.boostDelay;
}

IPhysicsObject *CPhysicsVehicleController::GetWheel( int index )
{
	if ( !m_pCarSystem || index < 0 || index >= GetCreatedWheelCount() )
		return nullptr;
	return m_pWheels[index];
}

bool CPhysicsVehicleController::GetWheelContactPoint( int index, Vector *pContactPoint, int *pSurfaceProps )
{
	if ( index < 0 || index >= GetCreatedWheelCount() || !m_pWheels[index] )
	{
		pContactPoint->Init();
		*pSurfaceProps = 0;
		return false;
	}

	m_pWheels[index]->GetPosition( pContactPoint, nullptr );
	return m_pEnvironment->GetStrongestContact( m_pWheels[index], *pContactPoint, *pSurfaceProps );
}

void CPhysicsVehicleController::SetSpringLength( int wheelIndex, float length )
{
	if ( m_pCarSystem && wheelIndex >= 0 && wheelIndex < GetCreatedWheelCount() )
		m_pCarSystem->SetSpringLength( wheelIndex, length );
}

void CPhysicsVehicleController::SetWheelFriction( int wheelIndex, float friction )
{
	if ( m_pCarSystem && wheelIndex >= 0 && wheelIndex < GetCreatedWheelCount() )
		m_pCarSystem->SetWheelFriction( wheelIndex, friction );
}

void CPhysicsVehicleController::OnVehicleEnter()
{
	m_bOccupied = true;
	if ( m_nVehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST && m_pBody )
	{
		const float flDamping = 0.0f;
		m_pBody->SetDamping( &flDamping, &flDamping );
	}
}

void CPhysicsVehicleController::OnVehicleExit()
{
	m_bOccupied = false;
	if ( m_VehicleParams.steering.isSkidAllowed && m_pCarSystem )
	{
		int nWheel = 0;
		for ( int i = 0; i < m_VehicleParams.axleCount; ++i )
		{
			for ( int j = 0; j < m_VehicleParams.wheelsPerAxle; ++j, ++nWheel )
			{
				if ( nWheel >= VEHICLE_MAX_WHEEL_COUNT )
					continue;

				if ( m_nTireType != VEHICLE_TIRE_NORMAL )
					m_pWheels[nWheel]->SetMaterialIndex( m_VehicleParams.axles[i].wheels.materialIndex );
				m_pCarSystem->FixWheel( nWheel, true );
			}
		}
		m_nTireType = VEHICLE_TIRE_NORMAL;
		m_OperatingParams.skidSpeed = 0.0f;
	}

	if ( m_nVehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST && m_pBody )
	{
		const float flDamping = 1.0f;
		m_pBody->SetDamping( &flDamping, &flDamping );
	}
	SetEngineDisabled( false );
}

void CPhysicsVehicleController::GetCarSystemDebugData( vehicle_debugcarsystem_t &debugCarSystem )
{
	CarSystemDebugData_t data = {};
	if ( m_pCarSystem )
		m_pCarSystem->GetDebugData( data );

	for ( int i = 0; i < VEHICLE_DEBUGRENDERDATA_MAX_WHEELS; ++i )
	{
		const Vector start = ToSource( data.m_WheelRaycasts[i][0] );
		const Vector end = ToSource( data.m_WheelRaycasts[i][1] );
		debugCarSystem.vecWheelRaycasts[i][0] = start;
		debugCarSystem.vecWheelRaycasts[i][1] = end;
		debugCarSystem.vecWheelRaycastImpacts[i] = start + ( end - start ) * data.m_WheelRaycastFractions[i];
	}
	for ( int i = 0; i < VEHICLE_DEBUGRENDERDATA_MAX_AXLES; ++i )
		debugCarSystem.vecAxlePos[i] = IVPToSourcePosition( data.m_AxlePositions[i] );
}
