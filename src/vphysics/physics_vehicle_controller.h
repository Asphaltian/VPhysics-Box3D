#pragma once

#include "vphysics_interface.h"
#include "vphysics/vehicles.h"
#include "tier0/valve_minmax_off.h"

#include "box3d/math_functions.h"

class CCarSystem;
class CPhysicsEnvironment;
class CPhysicsObject;
struct CarSystemTemplate_t;
struct vphysics_save_cvehiclecontroller_t;

class CPhysicsVehicleController final : public IPhysicsVehicleController
{
public:
	CPhysicsVehicleController( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace );
	CPhysicsVehicleController( CPhysicsEnvironment *pEnvironment, IPhysicsGameTrace *pGameTrace, const vphysics_save_cvehiclecontroller_t &record );
	~CPhysicsVehicleController() override;

	void Update( float dt, vehicle_controlparams_t &controls ) override;

	const vehicle_operatingparams_t &GetOperatingParams() override { return m_OperatingParams; }

	const vehicleparams_t &GetVehicleParams() override { return m_VehicleParams; }

	vehicleparams_t &GetVehicleParamsForChange() override { return m_VehicleParams; }

	float UpdateBooster( float dt ) override;

	int GetWheelCount() override { return m_nWheelCount; }

	IPhysicsObject *GetWheel( int index ) override;
	bool GetWheelContactPoint( int index, Vector *pContactPoint, int *pSurfaceProps ) override;
	void SetSpringLength( int wheelIndex, float length ) override;
	void SetWheelFriction( int wheelIndex, float friction ) override;
	void OnVehicleEnter() override;
	void OnVehicleExit() override;

	void SetEngineDisabled( bool bDisable ) override { m_bEngineDisabled = bDisable; }

	bool IsEngineDisabled() override { return m_bEngineDisabled; }

	void GetCarSystemDebugData( vehicle_debugcarsystem_t &debugCarSystem ) override;
	void VehicleDataReload() override;

	void ObjectDestroyed( CPhysicsObject *pObject );
	void WriteRecord( vphysics_save_cvehiclecontroller_t &record );

private:
	int GetCreatedWheelCount() const;
	bool IsBoosting() const;
	void CreateWheels( CarSystemTemplate_t &carTemplate );
	void CreateCarSystem();
	void ShutDown();
	float GetSteeringAngle( float flSpeed, float flSteering, bool bAnalog ) const;
	void UpdateTireMaterials( const vehicle_controlparams_t &controls, bool bPowerslide, float flSpeed );
	float UpdateEngine( float flThrottle, float flBrake, float flSteering, bool bTorqueBoosting );
	void UpdateWheelLock( float flThrottle, bool bPowerslide, bool bHandbrake );
	void UpdateContacts( bool bHandbrake );
	void UpdateBody();
	void SetAxleTorque( int nAxle, float flTorque );
	void ApplyRestoredDefaults();

	CCarSystem *m_pCarSystem = nullptr;
	CPhysicsObject *m_pBody = nullptr;
	CPhysicsEnvironment *m_pEnvironment;
	IPhysicsGameTrace *m_pGameTrace;
	int m_nWheelCount = 0;
	vehicleparams_t m_VehicleParams;
	vehicle_operatingparams_t m_OperatingParams = {};
	float m_flMaxWheelRadius = 0.0f;
	float m_flBodyMass = 0.0f;
	float m_flTotalWheelMass = 0.0f;
	float m_flGravityLength = 0.0f;
	float m_flTorqueScale = 1.0f;
	CPhysicsObject *m_pWheels[VEHICLE_MAX_WHEEL_COUNT] = {};
	b3Vec3 m_WheelPositions[VEHICLE_MAX_WHEEL_COUNT] = {};
	b3Vec3 m_TracePositions[VEHICLE_MAX_WHEEL_COUNT] = {};
	int m_nTireType = 0;
	unsigned int m_nVehicleType;
	bool m_bWheelTorquesZeroed = false;
	bool m_bWheelsFixed = false;
	bool m_bTraceData;
	bool m_bOccupied = false;
	bool m_bEngineDisabled = false;
};
