#include "physics_filter_table.h"

#include <algorithm>
#include <bit>

static constexpr uint32_t COLLIDE_BIT = 0x80000000u;
static constexpr uint32_t GENERATION_MASK = ~COLLIDE_BIT;
static constexpr size_t MIN_CAPACITY = 1024;
static constexpr uint64_t HASH_MULTIPLIER = 0x9E3779B97F4A7C15ull;

uint64_t CFilterDecisionTable::GetKey( uint32_t nId0, uint32_t nId1 )
{
	return nId0 < nId1 ? ( static_cast<uint64_t>( nId0 ) << 32 ) | nId1 : ( static_cast<uint64_t>( nId1 ) << 32 ) | nId0;
}

size_t CFilterDecisionTable::FindSlot( uint64_t nKey ) const
{
	const size_t nMask = m_Entries.size() - 1;
	size_t nSlot = static_cast<size_t>( ( nKey * HASH_MULTIPLIER ) >> 32 ) & nMask;
	while ( m_Entries[nSlot].m_nKey != 0 && m_Entries[nSlot].m_nKey != nKey )
		nSlot = ( nSlot + 1 ) & nMask;
	return nSlot;
}

uint32_t CFilterDecisionTable::GetGeneration( uint32_t nId ) const
{
	return nId < m_Generations.size() ? m_Generations[nId] : 0;
}

bool CFilterDecisionTable::IsLive( const Entry_t &entry ) const
{
	return entry.m_nGeneration0 == GetGeneration( static_cast<uint32_t>( entry.m_nKey >> 32 ) ) && ( entry.m_nGeneration1 & GENERATION_MASK ) == GetGeneration( static_cast<uint32_t>( entry.m_nKey ) );
}

bool CFilterDecisionTable::Find( uint32_t nId0, uint32_t nId1, bool &bCollide ) const
{
	if ( m_Entries.empty() )
		return false;

	const Entry_t &entry = m_Entries[FindSlot( GetKey( nId0, nId1 ) )];
	if ( entry.m_nKey == 0 || !IsLive( entry ) )
		return false;

	bCollide = ( entry.m_nGeneration1 & COLLIDE_BIT ) != 0;
	return true;
}

void CFilterDecisionTable::Store( uint32_t nId0, uint32_t nId1, bool bCollide )
{
	if ( ( m_nCount + 1 ) * 2 > m_Entries.size() )
		Rebuild();

	const uint64_t nKey = GetKey( nId0, nId1 );
	Entry_t &entry = m_Entries[FindSlot( nKey )];
	if ( entry.m_nKey == 0 )
	{
		entry.m_nKey = nKey;
		++m_nCount;
	}
	entry.m_nGeneration0 = GetGeneration( static_cast<uint32_t>( nKey >> 32 ) );
	entry.m_nGeneration1 = GetGeneration( static_cast<uint32_t>( nKey ) ) | ( bCollide ? COLLIDE_BIT : 0 );
}

void CFilterDecisionTable::Invalidate( uint32_t nId )
{
	if ( nId >= m_Generations.size() )
		m_Generations.resize( static_cast<size_t>( nId ) + 1, 0 );
	m_Generations[nId] = ( m_Generations[nId] + 1 ) & GENERATION_MASK;
}

void CFilterDecisionTable::Rebuild()
{
	std::vector<Entry_t> entries;
	entries.reserve( m_nCount );
	for ( const Entry_t &entry : m_Entries )
	{
		if ( entry.m_nKey != 0 && IsLive( entry ) )
			entries.push_back( entry );
	}

	const size_t nCapacity = std::bit_ceil( std::max( MIN_CAPACITY, ( entries.size() + 1 ) * 4 ) );
	m_Entries.assign( nCapacity, Entry_t{} );
	m_nCount = entries.size();
	for ( const Entry_t &entry : entries )
		m_Entries[FindSlot( entry.m_nKey )] = entry;
}
