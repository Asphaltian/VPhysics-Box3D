#pragma once

#include "physics_controller.h"
#include "physics_environment.h"

#include "vphysics_interface.h"
#include "vphysics/player_controller.h"
#include "tier0/valve_minmax_off.h"

#include "compact_builder/precision.h"

#include <vector>

class CPhysicsPlayerController final : public IPhysicsPlayerController, public IController
{
public:
	static constexpr int PRIORITY = 501;

	explicit CPhysicsPlayerController( CPhysicsObject *pObject );
	~CPhysicsPlayerController() override;

	void Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground ) override;

	void SetEventHandler( IPhysicsPlayerControllerEvent *handler ) override { m_pHandler = handler; }

	bool IsInContact() override;
	void MaxSpeed( const Vector &maxVelocity ) override;
	void SetObject( IPhysicsObject *pObject ) override;
	int GetShadowPosition( Vector *position, QAngle *angles ) override;
	void StepUp( float height ) override;

	void Jump() override {}

	void GetShadowVelocity( Vector *velocity ) override;
	IPhysicsObject *GetObject() override;
	void GetLastImpulse( Vector *pOut ) override;

	void SetPushMassLimit( float maxPushMass ) override { m_flPushMassLimit = maxPushMass; }

	void SetPushSpeedLimit( float maxPushSpeed ) override { m_flPushSpeedLimit = maxPushSpeed; }

	float GetPushMassLimit() override { return m_flPushMassLimit; }

	float GetPushSpeedLimit() override { return m_flPushSpeedLimit; }

	bool WasFrozen() override { return false; }

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "vphysics:player"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;
	void ObjectDestroyed( CPhysicsObject *pObject ) override;

	void GroundDestroyed( const CPhysicsObject *pObject );
	void DetachFromObject();
	void AttachToObject();

private:
	void AttachObject( CPhysicsObject *pObject );
	void DetachObject();
	void ClipToContacts( CPhysicsObject *pObject, bool &bStanding );

	CPhysicsObject *m_pObject = nullptr;
	CPhysicsObject *m_pGround = nullptr;
	IPhysicsPlayerControllerEvent *m_pHandler = nullptr;
	float m_flSavedRotDamping = 0.0f;
	float m_flSecondsToArrival = 0.0f;
	float m_flPushMassLimit = 50000.0f;
	float m_flPushSpeedLimit = 10000.0f;
	RealVec3_t m_TargetPosition = {};
	b3Vec3 m_GroundPosition = b3Vec3_zero;
	b3Vec3 m_MaxSpeed = b3Vec3_zero;
	b3Vec3 m_TargetVelocity = b3Vec3_zero;
	b3Vec3 m_LastImpulse = b3Vec3_zero;
	bool m_bActive = false;
	bool m_bUpdated = false;
};
