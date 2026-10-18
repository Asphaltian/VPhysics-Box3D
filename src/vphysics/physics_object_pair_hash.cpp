#include "physics_object_pair_hash.h"

#include <algorithm>
#include <cstdint>
#include <functional>

size_t CPhysicsObjectPairHash::ObjectPairHash::operator()( const ObjectPair_t &pair ) const
{
	const uint64_t nFirst = std::hash<void *>()( pair.first );
	const uint64_t nSecond = std::hash<void *>()( pair.second );
	return static_cast<size_t>( nFirst ^ ( nSecond + 0x9E3779B97F4A7C15ull + ( nFirst << 6 ) + ( nFirst >> 2 ) ) );
}

CPhysicsObjectPairHash::ObjectPair_t CPhysicsObjectPairHash::MakePair( void *pObject0, void *pObject1 )
{
	if ( reinterpret_cast<uintptr_t>( pObject1 ) < reinterpret_cast<uintptr_t>( pObject0 ) )
		std::swap( pObject0, pObject1 );

	return { pObject0, pObject1 };
}

void CPhysicsObjectPairHash::RemovePartner( void *pObject, void *pPartner )
{
	auto it = m_Partners.find( pObject );
	if ( it == m_Partners.end() )
		return;

	std::vector<void *> &partners = it->second;
	if ( auto partner = std::ranges::find( partners, pPartner ); partner != partners.end() )
		partners.erase( partner );

	if ( partners.empty() )
		m_Partners.erase( it );
}

void CPhysicsObjectPairHash::AddObjectPair( void *pObject0, void *pObject1 )
{
	if ( !m_Pairs.insert( MakePair( pObject0, pObject1 ) ).second )
		return;

	m_Partners[pObject0].push_back( pObject1 );
	m_Partners[pObject1].push_back( pObject0 );
}

void CPhysicsObjectPairHash::RemoveObjectPair( void *pObject0, void *pObject1 )
{
	if ( !m_Pairs.erase( MakePair( pObject0, pObject1 ) ) )
		return;

	RemovePartner( pObject0, pObject1 );
	RemovePartner( pObject1, pObject0 );
}

bool CPhysicsObjectPairHash::IsObjectPairInHash( void *pObject0, void *pObject1 )
{
	return m_Pairs.contains( MakePair( pObject0, pObject1 ) );
}

void CPhysicsObjectPairHash::RemoveAllPairsForObject( void *pObject0 )
{
	auto it = m_Partners.find( pObject0 );
	if ( it == m_Partners.end() )
		return;

	const std::vector<void *> partners = std::move( it->second );
	m_Partners.erase( it );

	for ( void *pPartner : partners )
	{
		RemovePartner( pPartner, pObject0 );
		m_Pairs.erase( MakePair( pPartner, pObject0 ) );
	}
}

bool CPhysicsObjectPairHash::IsObjectInHash( void *pObject0 )
{
	return m_Partners.contains( pObject0 );
}

int CPhysicsObjectPairHash::GetPairCountForObject( void *pObject0 )
{
	auto it = m_Partners.find( pObject0 );
	return it != m_Partners.end() ? static_cast<int>( it->second.size() ) : 0;
}

int CPhysicsObjectPairHash::GetPairListForObject( void *pObject0, int nMaxCount, void **ppObjectList )
{
	auto it = m_Partners.find( pObject0 );
	if ( it == m_Partners.end() || nMaxCount <= 0 )
		return 0;

	const int nCount = std::min( nMaxCount, static_cast<int>( it->second.size() ) );
	std::copy_n( it->second.begin(), nCount, ppObjectList );
	return nCount;
}
