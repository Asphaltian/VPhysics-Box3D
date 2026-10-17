#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

class CFilterDecisionTable
{
public:
	bool Find( uint32_t nId0, uint32_t nId1, bool &bCollide ) const;
	void Store( uint32_t nId0, uint32_t nId1, bool bCollide );
	void Invalidate( uint32_t nId );

private:
	struct Entry_t
	{
		uint64_t m_nKey;
		uint32_t m_nGeneration0;
		uint32_t m_nGeneration1;
	};

	static uint64_t GetKey( uint32_t nId0, uint32_t nId1 );
	size_t FindSlot( uint64_t nKey ) const;
	uint32_t GetGeneration( uint32_t nId ) const;
	bool IsLive( const Entry_t &entry ) const;
	void Rebuild();

	std::vector<Entry_t> m_Entries;
	std::vector<uint32_t> m_Generations;
	size_t m_nCount = 0;
};
