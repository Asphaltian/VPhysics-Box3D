#pragma once

#include "physics_controller.h"

#include "vphysics_interface.h"
#include "tier0/valve_minmax_off.h"

#include "box3d/id.h"
#include "box3d/math_functions.h"

#include <cstdint>

class CPhysicsEnvironment;
class CPhysicsObject;
struct vphysics_save_cphysicsspring_t;

class CPhysicsSpring final : public IPhysicsSpring, public IController
{
public:
	static constexpr int PRIORITY = 1500;

	CPhysicsSpring( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pStart, CPhysicsObject *pEnd, const springparams_t &params );
	~CPhysicsSpring() override;

	void GetEndpoints( Vector *worldPositionStart, Vector *worldPositionEnd ) override;
	void SetSpringConstant( float flConstant ) override;
	void SetSpringDamping( float flDamping ) override;
	void SetSpringLength( float flLength ) override;

	IPhysicsObject *GetStartObject() override;
	IPhysicsObject *GetEndObject() override;

	int GetPriority() const override { return PRIORITY; }

	const char *GetName() const override { return "sys:spring"; }

	void Simulate( CPhysicsObject *pObject, float flTimestep ) override;
	void ObjectDestroyed( CPhysicsObject *pObject ) override;

	void WriteRecord( vphysics_save_cphysicsspring_t &record ) const;

private:
	void Detach();
	void EnsureInSimulation() const;
	b3Vec3 GetAnchorVelocity( const CPhysicsObject *pObject, int nAnchor ) const;

	CPhysicsEnvironment *m_pEnvironment;
	CPhysicsObject *m_pStart;
	CPhysicsObject *m_pEnd;
	b3JointId m_Link = b3_nullJointId;
	b3Vec3 m_ObjectAnchors[2];
	b3Vec3 m_CoreAnchors[2];
	float m_flLength;
	float m_flConstant;
	float m_flDamping;
	float m_flRelativeDamping;
	uint32_t m_nSimulatedStep = 0;
	bool m_bOnlyStretch;
};
