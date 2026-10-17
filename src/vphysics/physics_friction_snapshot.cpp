#include "physics_friction_snapshot.h"

#include "physics_environment.h"
#include "physics_object.h"

CPhysicsFrictionSnapshot::CPhysicsFrictionSnapshot( CPhysicsObject *pObject )
: m_pObject( pObject )
, m_pCurrent( pObject->GetFrictionHead() )
{
}

int CPhysicsFrictionSnapshot::GetSide() const
{
	return m_pCurrent->m_pObjects[0] == m_pObject ? 0 : 1;
}

IPhysicsObject *CPhysicsFrictionSnapshot::GetObject( int index )
{
	return index == 1 ? m_pCurrent->m_pObjects[1 - GetSide()] : m_pObject;
}

int CPhysicsFrictionSnapshot::GetMaterial( int index )
{
	return m_pCurrent->m_Materials[( index & 1 ) ^ GetSide()];
}

void CPhysicsFrictionSnapshot::GetContactPoint( Vector &out )
{
	out = m_pCurrent->m_Point;
}

void CPhysicsFrictionSnapshot::GetSurfaceNormal( Vector &out )
{
	out = m_pObject->GetEnvironment()->GetFrictionNormal( *m_pCurrent, m_pObject );
}

float CPhysicsFrictionSnapshot::GetNormalForce()
{
	return m_pCurrent->m_flNormalImpulse * m_pObject->GetEnvironment()->GetInverseTimestep();
}

float CPhysicsFrictionSnapshot::GetEnergyAbsorbed()
{
	return m_pCurrent->m_flEnergy;
}

float CPhysicsFrictionSnapshot::GetFrictionCoefficient()
{
	return m_pCurrent->m_flFriction;
}

void CPhysicsFrictionSnapshot::NextFrictionData()
{
	m_pCurrent = m_pCurrent->GetNext( m_pObject );
}

void CPhysicsFrictionSnapshot::MarkContactForDelete()
{
	m_bHasMarkedList = true;
	CPhysicsObject *pOther = m_pCurrent->m_pObjects[1 - GetSide()];
	if ( pOther != m_pObject )
		m_MarkedPartners.push_back( pOther );
}

void CPhysicsFrictionSnapshot::DeleteAllMarkedContacts( bool wakeObjects )
{
	if ( !m_bHasMarkedList )
		return;

	for ( CPhysicsObject *pPartner : m_MarkedPartners )
	{
		if ( wakeObjects )
			pPartner->Wake();
		m_pObject->GetEnvironment()->RemoveContacts( m_pObject, pPartner );
	}

	m_pCurrent = nullptr;
}
