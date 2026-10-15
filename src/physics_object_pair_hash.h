#pragma once

#include "vphysics/object_hash.h"

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

class CPhysicsObjectPairHash final : public IPhysicsObjectPairHash
{
public:
	void AddObjectPair( void *pObject0, void *pObject1 ) override;
	void RemoveObjectPair( void *pObject0, void *pObject1 ) override;
	bool IsObjectPairInHash( void *pObject0, void *pObject1 ) override;
	void RemoveAllPairsForObject( void *pObject0 ) override;
	bool IsObjectInHash( void *pObject0 ) override;
	int GetPairCountForObject( void *pObject0 ) override;
	int GetPairListForObject( void *pObject0, int nMaxCount, void **ppObjectList ) override;

private:
	using ObjectPair_t = std::pair<void *, void *>;

	struct ObjectPairHash
	{
		size_t operator()( const ObjectPair_t &pair ) const;
	};

	static ObjectPair_t MakePair( void *pObject0, void *pObject1 );

	void RemovePartner( void *pObject, void *pPartner );

	std::unordered_set<ObjectPair_t, ObjectPairHash> m_Pairs;
	std::unordered_map<void *, std::vector<void *>> m_Partners;
};
