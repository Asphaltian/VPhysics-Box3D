#pragma once

#include "vphysics_interface.h"
#include "vphysics/friction.h"
#include "tier0/valve_minmax_off.h"

#include <cstdint>
#include <vector>

class CPhysicsObject;

class CPhysicsFrictionSnapshot final : public IPhysicsFrictionSnapshot
{
public:
	struct Contact_t
	{
		uint64_t m_nKey;
		CPhysicsObject *m_pOther;
		Vector m_Point;
		Vector m_Normal;
		float m_flNormalForce;
		float m_flEnergy;
		float m_flFriction;
		int m_Materials[2];
	};

	CPhysicsFrictionSnapshot( CPhysicsObject *pObject, std::vector<Contact_t> contacts );

	bool IsValid() override { return m_nCurrent < m_Contacts.size(); }

	IPhysicsObject *GetObject( int index ) override;
	int GetMaterial( int index ) override;

	void GetContactPoint( Vector &out ) override { out = m_Contacts[m_nCurrent].m_Point; }

	void GetSurfaceNormal( Vector &out ) override { out = m_Contacts[m_nCurrent].m_Normal; }

	float GetNormalForce() override { return m_Contacts[m_nCurrent].m_flNormalForce; }

	float GetEnergyAbsorbed() override { return m_Contacts[m_nCurrent].m_flEnergy; }

	void RecomputeFriction() override;

	void ClearFrictionForce() override {}

	void MarkContactForDelete() override;
	void DeleteAllMarkedContacts( bool wakeObjects ) override;

	void NextFrictionData() override { ++m_nCurrent; }

	float GetFrictionCoefficient() override { return m_Contacts[m_nCurrent].m_flFriction; }

private:
	CPhysicsObject *m_pObject;
	std::vector<Contact_t> m_Contacts;
	std::vector<CPhysicsObject *> m_MarkedPartners;
	size_t m_nCurrent = 0;
	bool m_bHasMarkedList = false;
};
