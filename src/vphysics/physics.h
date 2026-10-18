#pragma once

#include "appframework/IAppSystem.h"
#include "vphysics_interface.h"
#include "tier0/valve_minmax_off.h"

#include <memory>
#include <utility>
#include <vector>

class CPhysicsCollisionSet;
class CPhysicsEnvironment;

class CPhysics final : public CTier0AppSystem<IPhysics>
{
public:
	CPhysics();
	~CPhysics();

	void *QueryInterface( const char *pInterfaceName ) override;

	IPhysicsEnvironment *CreateEnvironment() override;
	void DestroyEnvironment( IPhysicsEnvironment *pEnvironment ) override;
	IPhysicsEnvironment *GetActiveEnvironmentByIndex( int index ) override;

	IPhysicsObjectPairHash *CreateObjectPairHash() override;
	void DestroyObjectPairHash( IPhysicsObjectPairHash *pHash ) override;

	IPhysicsCollisionSet *FindOrCreateCollisionSet( unsigned int id, int maxElementCount ) override;
	IPhysicsCollisionSet *FindCollisionSet( unsigned int id ) override;
	void DestroyAllCollisionSets() override;

	void UpdateWorldMaterials();

private:
	std::vector<CPhysicsEnvironment *> m_Environments;
	std::vector<std::pair<unsigned int, std::unique_ptr<CPhysicsCollisionSet>>> m_CollisionSets;
};

extern CPhysics g_Physics;
