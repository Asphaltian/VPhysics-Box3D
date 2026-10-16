#include "physics_friction_snapshot.h"

#include "physics_environment.h"
#include "physics_object.h"

CPhysicsFrictionSnapshot::CPhysicsFrictionSnapshot( CPhysicsObject *pObject, std::vector<Contact_t> contacts )
: m_pObject( pObject )
, m_Contacts( std::move( contacts ) )
{
}

IPhysicsObject *CPhysicsFrictionSnapshot::GetObject( int index )
{
	return index == 1 ? m_Contacts[m_nCurrent].m_pOther : m_pObject;
}

int CPhysicsFrictionSnapshot::GetMaterial( int index )
{
	return m_Contacts[m_nCurrent].m_Materials[index & 1];
}

void CPhysicsFrictionSnapshot::RecomputeFriction()
{
	Contact_t &contact = m_Contacts[m_nCurrent];
	m_pObject->GetEnvironment()->RefreshFrictionContact( m_pObject, contact.m_nKey, contact.m_Point, contact.m_Normal );
}

void CPhysicsFrictionSnapshot::MarkContactForDelete()
{
	m_bHasMarkedList = true;
	CPhysicsObject *pOther = m_Contacts[m_nCurrent].m_pOther;
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

	m_nCurrent = m_Contacts.size();
}
