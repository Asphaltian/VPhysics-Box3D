#pragma once

#include "vphysics_interface.h"
#include "vphysics/friction.h"
#include "tier0/valve_minmax_off.h"

#include <vector>

class CPhysicsObject;
struct FrictionContact_t;

class CPhysicsFrictionSnapshot final : public IPhysicsFrictionSnapshot
{
public:
	explicit CPhysicsFrictionSnapshot( CPhysicsObject *pObject );

	bool IsValid() override { return m_pCurrent != nullptr; }

	IPhysicsObject *GetObject( int index ) override;
	int GetMaterial( int index ) override;
	void GetContactPoint( Vector &out ) override;
	void GetSurfaceNormal( Vector &out ) override;
	float GetNormalForce() override;
	float GetEnergyAbsorbed() override;

	void RecomputeFriction() override {}

	void ClearFrictionForce() override {}

	void MarkContactForDelete() override;
	void DeleteAllMarkedContacts( bool wakeObjects ) override;
	void NextFrictionData() override;
	float GetFrictionCoefficient() override;

private:
	int GetSide() const;

	CPhysicsObject *m_pObject;
	const FrictionContact_t *m_pCurrent;
	std::vector<CPhysicsObject *> m_MarkedPartners;
	bool m_bHasMarkedList = false;
};
