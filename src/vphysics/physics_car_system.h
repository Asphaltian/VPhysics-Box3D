#pragma once

#include "physics_controller.h"

#include "vphysics_interface.h"
#include "vphysics/vehicles.h"
#include "tier0/valve_minmax_off.h"

#include "compact_builder/math_util.h"

#include "box3d/id.h"
#include "box3d/math_functions.h"

#include <cstdint>

class CPhysicsEnvironment;
class CPhysicsObject;

struct CarSystemTemplate_t
{
	int m_nWheels;
	int m_nAxles;
	CPhysicsObject *m_pBody;
	CPhysicsObject *m_pWheels[VEHICLE_MAX_WHEEL_COUNT];
	b3Vec3 m_WheelPositions[VEHICLE_MAX_WHEEL_COUNT];
	b3Vec3 m_TracePositions[VEHICLE_MAX_WHEEL_COUNT];
	float m_SpringConstants[VEHICLE_MAX_WHEEL_COUNT];
	float m_SpringDampings[VEHICLE_MAX_WHEEL_COUNT];
	float m_SpringDampingCompressions[VEHICLE_MAX_WHEEL_COUNT];
	float m_MaxBodyForces[VEHICLE_MAX_WHEEL_COUNT];
	float m_SpringAdditionalLengths[VEHICLE_MAX_WHEEL_COUNT];
	float m_StabilizerConstants[VEHICLE_MAX_AXLE_COUNT];
	float m_MaxWheelRotationSpeeds[VEHICLE_MAX_AXLE_COUNT];
	float m_flCounterTorqueFactor;
	float m_flExtraGravityForce;
	float m_flDownforceHeight;
};

struct CarSystemDebugData_t
{
	b3Vec3 m_WheelRaycasts[VEHICLE_DEBUGRENDERDATA_MAX_WHEELS][2];
	float m_WheelRaycastFractions[VEHICLE_DEBUGRENDERDATA_MAX_WHEELS];
	DoubleVec3_t m_AxlePositions[VEHICLE_DEBUGRENDERDATA_MAX_AXLES];
};

class CCarSystem
{
public:
	virtual ~CCarSystem() = default;

	virtual void DoSteering( float flAngle, bool bAnalog ) = 0;
	virtual float GetBodySpeed() const = 0;

	virtual void ChangeWheelTorque( int, float ) {}

	virtual void UpdateBodyCounterTorque() {}

	virtual void UpdateThrottle( float ) {}

	virtual void ChangeBodyDownforce( float ) {}

	virtual void FixWheel( int, bool ) {}

	virtual void SetPowerslide( float, float ) {}

	virtual float GetWheelAngularVelocity( int ) const { return 0.0f; }

	virtual void SetSpringLength( int, float ) {}

	virtual void SetWheelFriction( int, float ) {}

	virtual void UpdateWheelPositions() {}

	virtual void ActivateBooster( float, float, float ) {}

	virtual void UpdateBooster( float ) {}

	virtual float GetBoosterDelay() const { return 0.0f; }

	virtual float GetBoosterTimeToGo() const { return 0.0f; }

	virtual void GetDebugData( CarSystemDebugData_t & ) const {}
};

class CSuspensionActuator final : public IController
{
public:
	static constexpr int PRIORITY = 1500;

	CSuspensionActuator( CPhysicsObject *pBody, CPhysicsObject *pWheel, b3Vec3 bodyAnchor, float flLength, float flConstant, float flReboundDamping, float flCompressionDamping, float flMaxBodyForce );
	~CSuspensionActuator() override;

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "sys:suspension"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;
	void ObjectDestroyed( CPhysicsObject *pObject ) override;

	void SetLength( float flLength );

private:
	CPhysicsObject *m_pBody;
	CPhysicsObject *m_pWheel;
	b3Vec3 m_BodyAnchor;
	b3Vec3 m_WheelAnchor;
	float m_flLength;
	float m_flConstant;
	float m_flReboundDamping;
	float m_flCompressionDamping;
	float m_flMaxBodyForce;
	uint32_t m_nSimulatedStep = 0;
};

class CTorqueActuator final : public IController
{
public:
	static constexpr int PRIORITY = 1500;

	CTorqueActuator( CPhysicsObject *pObject, float flMaxRotationSpeed );
	~CTorqueActuator() override;

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "sys:torque_actuator"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;
	void ObjectDestroyed( CPhysicsObject *pObject ) override;

	void SetPower( float flPower );

	float GetPower() const { return m_flPower; }

	float GetMeasuredSpeed() const;

private:
	CPhysicsObject *m_pObject;
	float m_flPower = 0.0f;
	float m_flMaxRotationSpeed;
	float m_flMeasuredSpeed = 0.0f;
};

class CForceActuator final : public IController
{
public:
	static constexpr int PRIORITY = 1500;

	CForceActuator( CPhysicsObject *pObject, b3Vec3 anchor, b3Vec3 targetAnchor, float flForce );
	CForceActuator( CPhysicsObject *pObject, b3Vec3 anchor, DoubleVec3_t worldTarget, float flForce );
	~CForceActuator() override;

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "sys:force_actuator"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;
	void ObjectDestroyed( CPhysicsObject *pObject ) override;

	void SetForce( float flForce );

	DoubleVec3_t GetAnchorPosition() const;
	DoubleVec3_t GetTargetPosition() const;

private:
	CPhysicsObject *m_pObject;
	b3Vec3 m_Anchor;
	b3Vec3 m_TargetAnchor = b3Vec3_zero;
	DoubleVec3_t m_WorldTarget = {};
	float m_flForce;
	bool m_bWorldTarget;
};

class CCarWheelsSystem final : public CCarSystem
{
public:
	explicit CCarWheelsSystem( const CarSystemTemplate_t &carTemplate );
	~CCarWheelsSystem() override;

	void DoSteering( float flAngle, bool bAnalog ) override;
	float GetBodySpeed() const override;
	void ChangeWheelTorque( int nWheel, float flTorque ) override;
	void UpdateBodyCounterTorque() override;
	void ChangeBodyDownforce( float flForce ) override;
	void FixWheel( int nWheel, bool bFix ) override;
	void SetPowerslide( float flFront, float flRear ) override;
	float GetWheelAngularVelocity( int nWheel ) const override;
	void SetSpringLength( int nWheel, float flLength ) override;
	void ActivateBooster( float flForce, float flDuration, float flDelay ) override;
	void UpdateBooster( float flTimestep ) override;

	float GetBoosterDelay() const override { return m_flBoosterDelay; }

	float GetBoosterTimeToGo() const override { return m_flBoosterTimeToGo; }

	void GetDebugData( CarSystemDebugData_t &data ) const override;

private:
	void DoSteeringWheel( int nWheel, float flAngle );
	void SetBoosterForce( float flForce );
	float GetTrackWidth() const;
	float GetWheelbase() const;

	CPhysicsObject *m_pBody;
	int m_nWheels;
	int m_nAxles;
	CPhysicsObject *m_pWheels[VEHICLE_MAX_WHEEL_COUNT] = {};
	b3JointId m_Joints[VEHICLE_MAX_WHEEL_COUNT] = {};
	b3Vec3 m_Attachments[VEHICLE_MAX_WHEEL_COUNT] = {};
	CSuspensionActuator *m_pSuspensions[VEHICLE_MAX_WHEEL_COUNT] = {};
	CTorqueActuator *m_pWheelMotors[VEHICLE_MAX_WHEEL_COUNT] = {};
	CTorqueActuator *m_pBodyMotor;
	CForceActuator *m_pDownforce;
	CForceActuator *m_pFrontLateral;
	CForceActuator *m_pRearLateral;
	CForceActuator *m_pBoosterThrust = nullptr;
	CForceActuator *m_pBoosterAntiGravity = nullptr;
	bool m_FixedWheels[VEHICLE_MAX_WHEEL_COUNT] = {};
	float m_flCounterTorqueFactor;
	float m_flBoosterTimeToGo = 0.0f;
	float m_flBoosterDelay = 0.0f;
	float m_flSteeringAngle = -1.0f;
};

class CCarRaycastSystem final : public CCarSystem, public IController
{
public:
	static constexpr int PRIORITY = 410;

	explicit CCarRaycastSystem( const CarSystemTemplate_t &carTemplate );
	~CCarRaycastSystem() override;

	void DoSteering( float flAngle, bool bAnalog ) override;
	float GetBodySpeed() const override;
	void ChangeWheelTorque( int nWheel, float flTorque ) override;
	void FixWheel( int nWheel, bool bFix ) override;
	float GetWheelAngularVelocity( int nWheel ) const override;
	void SetSpringLength( int nWheel, float flLength ) override;
	void UpdateWheelPositions() override;
	void ActivateBooster( float flForce, float flDuration, float flDelay ) override;

	float GetBoosterDelay() const override { return m_flBoosterDelay; }

	float GetBoosterTimeToGo() const override { return m_flBoosterTimeToGo; }

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "sys"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;

	void ObjectDestroyed( CPhysicsObject * ) override {}

private:
	struct Wheel_t
	{
		b3Vec3 m_HardPoint;
		b3Vec3 m_SpringDirection;
		float m_flSpringLength;
		float m_flSpringConstant;
		float m_flSpringDamping;
		float m_flSpringDampingCompression;
		float m_flRadius;
		float m_flInverseRadius;
		float m_flTorque;
		bool m_bFixed;
		b3Vec3 m_Axle;
		float m_flSpinAngle;
		float m_flAngularVelocity;
		float m_flSuspensionForce;
		float m_flHitDistance;
	};

	void DoSteeringWheel( int nWheel, float flAngle );

	CPhysicsObject *m_pBody;
	int m_nWheels;
	int m_nAxles;
	int m_nWheelsPerAxle;
	CPhysicsObject *m_pWheels[VEHICLE_MAX_WHEEL_COUNT] = {};
	Wheel_t m_Wheels[VEHICLE_MAX_WHEEL_COUNT] = {};
	float m_StabilizerConstants[VEHICLE_MAX_AXLE_COUNT] = {};
	b3Vec3 m_GravityDirection;
	float m_flExtraGravityForce;
	float m_flBoosterAcceleration = 0.0f;
	float m_flBoosterTimeToGo = 0.0f;
	float m_flBoosterDelay = 0.0f;
	float m_flSteeringAngle = -1.0f;
};

class CAirboatRaycastSystem final : public CCarSystem, public IController
{
public:
	static constexpr int PRIORITY = 410;

	CAirboatRaycastSystem( const CarSystemTemplate_t &carTemplate, IPhysicsGameTrace *pGameTrace );
	~CAirboatRaycastSystem() override;

	void DoSteering( float flAngle, bool bAnalog ) override;
	float GetBodySpeed() const override;
	void ChangeWheelTorque( int nWheel, float flTorque ) override;
	void UpdateThrottle( float flThrottle ) override;
	void SetWheelFriction( int nWheel, float flFriction ) override;
	void GetDebugData( CarSystemDebugData_t &data ) const override;

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "sys:airboat"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;

	void ObjectDestroyed( CPhysicsObject * ) override {}

private:
	struct Wheel_t
	{
		b3Vec3 m_TracePoint;
		b3Vec3 m_RayDirection;
		float m_flRayLength;
		float m_flSpringConstant;
		float m_flSpringDamping;
		float m_flSpringDampingCompression;
		float m_flFrictionScale;
		b3Vec3 m_Axle;
	};

	struct Impact_t
	{
		DoubleVec3_t m_Start;
		b3Vec3 m_Direction;
		DoubleVec3_t m_HitPoint;
		b3Vec3 m_Normal;
		DoubleVec3_t m_ContactPoint;
		b3Vec3 m_ContactVelocity;
		b3Vec3 m_TangentVelocity;
		float m_flHitLength;
		float m_flInverseNormalFactor;
		float m_flFriction;
		float m_flSurfaceFriction;
		float m_flWaterDepth;
		int m_nSurfaceProps;
		bool m_bHit;
		bool m_bInWater;
		bool m_bSubmerged;
	};

	void DoSteeringWheel( int nWheel, float flAngle );
	float GetWave( int nWheel, float flSpeedFactor ) const;
	void TracePontoons( Impact_t *pImpacts, b3Vec3 localVelocity, float flSpeed );
	void ApplyDrag( const Impact_t *pImpacts, b3Vec3 localVelocity, float flSpeed, float flTimestep );
	void ApplySteering( b3Vec3 localVelocity, float flTimestep, float flInverseTimestep );
	void ApplyRighting( int nContacts, float flInverseTimestep );

	CPhysicsObject *m_pBody;
	IPhysicsGameTrace *m_pGameTrace;
	int m_nWheels;
	int m_nWheelsPerAxle;
	Wheel_t m_Wheels[VEHICLE_MAX_WHEEL_COUNT] = {};
	float m_flSteeringAngle = -1.0f;
	bool m_bReverseSteering = false;
	bool m_bAnalogSteering = false;
	float m_flLastSteering = 0.0f;
	float m_flSteeringHoldTime = 0.0f;
	float m_flThrust = 0.0f;
	bool m_bInAir = false;
	float m_flAirTime = 0.0f;
	bool m_bLowSpeedAir = false;
	float m_flPreviousPitch = 0.0f;
	float m_flPreviousRoll = 0.0f;
	CarSystemDebugData_t m_DebugData = {};
};
