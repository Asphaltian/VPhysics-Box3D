#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <utility>
#include <vector>

template<size_t nWords>
class CVectorHash
{
public:
	using Key_t = std::array<uint32_t, nWords>;

	explicit CVectorHash( size_t nCapacity )
	: m_Slots( std::bit_ceil( std::max<size_t>( nCapacity * 2, 2 ) ), UINT32_MAX )
	{
	}

	template<typename LoadKey>
	std::pair<uint32_t, bool> FindOrInsert( const Key_t &key, uint32_t nNextIndex, LoadKey &&loadKey )
	{
		uint64_t nHash = 0;
		for ( size_t i = 0; i < nWords; ++i )
			nHash = ( nHash ^ key[i] ) * 0x9E3779B97F4A7C15ull;
		nHash ^= nHash >> 32;

		const size_t nMask = m_Slots.size() - 1;
		for ( size_t nSlot = nHash & nMask;; nSlot = ( nSlot + 1 ) & nMask )
		{
			const uint32_t nIndex = m_Slots[nSlot];
			if ( nIndex == UINT32_MAX )
			{
				m_Slots[nSlot] = nNextIndex;
				return { nNextIndex, true };
			}

			if ( loadKey( nIndex ) == key )
				return { nIndex, false };
		}
	}

private:
	std::vector<uint32_t> m_Slots;
};
