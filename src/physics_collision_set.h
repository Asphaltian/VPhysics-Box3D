#pragma once

#include "vphysics/collision_set.h"

#include <array>

class CPhysicsCollisionSet final : public IPhysicsCollisionSet
{
public:
	static constexpr int MAX_ELEMENT_COUNT = 32;

	void EnableCollisions( int index0, int index1 ) override;
	void DisableCollisions( int index0, int index1 ) override;
	bool ShouldCollide( int index0, int index1 ) override;

private:
	std::array<unsigned int, MAX_ELEMENT_COUNT> m_CollisionBits = {};
};
