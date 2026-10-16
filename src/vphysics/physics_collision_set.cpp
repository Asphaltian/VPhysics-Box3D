#include "physics_collision_set.h"

void CPhysicsCollisionSet::EnableCollisions( int index0, int index1 )
{
	m_CollisionBits[index0] |= 1u << index1;
	m_CollisionBits[index1] |= 1u << index0;
}

void CPhysicsCollisionSet::DisableCollisions( int index0, int index1 )
{
	m_CollisionBits[index0] &= ~( 1u << index1 );
	m_CollisionBits[index1] &= ~( 1u << index0 );
}

bool CPhysicsCollisionSet::ShouldCollide( int index0, int index1 )
{
	return ( m_CollisionBits[index0] >> index1 ) & 1u;
}
