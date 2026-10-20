#include "physics.h"

#include "physics_collision_set.h"
#include "physics_environment.h"
#include "physics_object_pair_hash.h"

#include "tier1/convar.h"
#include "tier1/interface.h"

#include <algorithm>

CON_COMMAND( vbox_version, "Prints the VPhysics Box3D version, Box3D revision and build platform" )
{
	Msg( "VPhysics Box3D %s (Box3D %s) %s\n", VPHYSICS_VERSION, VPHYSICS_BOX3D_VERSION, VPHYSICS_PLATFORM );
}

CPhysics g_Physics;

CPhysics::CPhysics() = default;

CPhysics::~CPhysics() = default;

void *CPhysics::QueryInterface( const char *pInterfaceName )
{
	return CreateInterface( pInterfaceName, nullptr );
}

IPhysicsEnvironment *CPhysics::CreateEnvironment()
{
	CPhysicsEnvironment *pEnvironment = new CPhysicsEnvironment;
	m_Environments.push_back( pEnvironment );
	return pEnvironment;
}

void CPhysics::DestroyEnvironment( IPhysicsEnvironment *pEnvironment )
{
	auto it = std::find( m_Environments.begin(), m_Environments.end(), pEnvironment );
	if ( it != m_Environments.end() )
		m_Environments.erase( it );

	delete pEnvironment;
}

IPhysicsEnvironment *CPhysics::GetActiveEnvironmentByIndex( int index )
{
	if ( index < 0 || index >= static_cast<int>( m_Environments.size() ) )
		return nullptr;

	return m_Environments[index];
}

void CPhysics::UpdateWorldMaterials()
{
	for ( CPhysicsEnvironment *pEnvironment : m_Environments )
		pEnvironment->UpdateWorldMaterials();
}

IPhysicsObjectPairHash *CPhysics::CreateObjectPairHash()
{
	return new CPhysicsObjectPairHash;
}

void CPhysics::DestroyObjectPairHash( IPhysicsObjectPairHash *pHash )
{
	delete pHash;
}

IPhysicsCollisionSet *CPhysics::FindOrCreateCollisionSet( unsigned int id, int maxElementCount )
{
	if ( maxElementCount > CPhysicsCollisionSet::MAX_ELEMENT_COUNT )
		return nullptr;

	if ( IPhysicsCollisionSet *pSet = FindCollisionSet( id ) )
		return pSet;

	return m_CollisionSets.emplace_back( id, std::make_unique<CPhysicsCollisionSet>() ).second.get();
}

IPhysicsCollisionSet *CPhysics::FindCollisionSet( unsigned int id )
{
	for ( const auto &[nId, pSet] : m_CollisionSets )
	{
		if ( nId == id )
			return pSet.get();
	}

	return nullptr;
}

void CPhysics::DestroyAllCollisionSets()
{
	m_CollisionSets.clear();
}
