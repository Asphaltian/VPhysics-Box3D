#pragma once

#include "physics_controller.h"
#include "physics_environment.h"
#include "physics_object.h"

#include "vphysics_interface.h"
#include "tier0/valve_minmax_off.h"

#include <vector>

class CPhysicsShadowController final : public IPhysicsShadowController, public IController
{
public:
	static constexpr int PRIORITY = 500;

	CPhysicsShadowController( CPhysicsObject *pObject, bool allowTranslation, bool allowRotation );
	~CPhysicsShadowController() override;

	void Update( const Vector &position, const QAngle &angles, float timeOffset ) override;
	void MaxSpeed( float maxSpeed, float maxAngularSpeed ) override;
	void StepUp( float height ) override;
	void SetTeleportDistance( float teleportDistance ) override;

	bool AllowsTranslation() override { return m_bAllowTranslation; }

	bool AllowsRotation() override { return m_bAllowRotation; }

	void SetPhysicallyControlled( bool isPhysicallyControlled ) override { m_bPhysicallyControlled = isPhysicallyControlled; }

	bool IsPhysicallyControlled() override { return m_bPhysicallyControlled; }

	void GetLastImpulse( Vector *pOut ) override;
	void UseShadowMaterial( bool bUseShadowMaterial ) override;
	void ObjectMaterialChanged( int materialIndex ) override;
	float GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut ) override;
	float GetTeleportDistance() override;
	void GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut ) override;

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "vphysics:shadow"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;

	void ObjectDestroyed( CPhysicsObject * ) override { m_pObject = nullptr; }

private:
	CPhysicsObject *m_pObject;
	CPhysicsObject::ShadowControl_t m_Control;
	float m_flSecondsToArrival = 0.0f;
	float m_flSavedRotDamping;
	Vector m_SavedInertia;
	float m_flSavedMass;
	unsigned short m_nSavedCallbackFlags;
	unsigned short m_nSavedMaterialIndex;
	bool m_bHasTarget = false;
	bool m_bAllowTranslation;
	bool m_bAllowRotation;
	bool m_bPhysicallyControlled = false;
};
