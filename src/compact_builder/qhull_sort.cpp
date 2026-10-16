#include "qhull_sort.h"

#include <algorithm>

static constexpr size_t SHORT_SORT_COUNT = 8;
static constexpr int MAXIMUM_PARTITIONS = 62;

static void SwapElements( unsigned char *a, unsigned char *b, size_t nSize )
{
	if ( a != b )
		std::swap_ranges( a, a + nSize, b );
}

static void ShortSort( unsigned char *pLow, unsigned char *pHigh, size_t nSize, int ( *pfnCompare )( const void *, const void * ) )
{
	for ( ; pHigh > pLow; pHigh -= nSize )
	{
		unsigned char *pMax = pLow;
		for ( unsigned char *p = pLow + nSize; p <= pHigh; p += nSize )
		{
			if ( pfnCompare( p, pMax ) > 0 )
				pMax = p;
		}
		SwapElements( pMax, pHigh, nSize );
	}
}

void QhullSort( void *pBase, size_t nCount, size_t nSize, int ( *pfnCompare )( const void *, const void * ) )
{
	if ( nCount < 2 || nSize == 0 )
		return;

	unsigned char *lowStack[MAXIMUM_PARTITIONS];
	unsigned char *highStack[MAXIMUM_PARTITIONS];
	int nStack = 0;

	unsigned char *pLow = static_cast<unsigned char *>( pBase );
	unsigned char *pHigh = pLow + ( nCount - 1 ) * nSize;

	for ( ;; )
	{
		const size_t nPartition = static_cast<size_t>( pHigh - pLow ) / nSize + 1;
		if ( nPartition <= SHORT_SORT_COUNT )
		{
			ShortSort( pLow, pHigh, nSize, pfnCompare );
		}
		else
		{
			unsigned char *pMid = pLow + ( nPartition / 2 ) * nSize;
			if ( pfnCompare( pLow, pMid ) > 0 )
				SwapElements( pLow, pMid, nSize );
			if ( pfnCompare( pLow, pHigh ) > 0 )
				SwapElements( pLow, pHigh, nSize );
			if ( pfnCompare( pMid, pHigh ) > 0 )
				SwapElements( pMid, pHigh, nSize );

			unsigned char *pLeft = pLow;
			unsigned char *pRight = pHigh;
			for ( ;; )
			{
				if ( pMid > pLeft )
				{
					do
					{
						pLeft += nSize;
					} while ( pLeft < pMid && pfnCompare( pLeft, pMid ) <= 0 );
				}
				if ( pMid <= pLeft )
				{
					do
					{
						pLeft += nSize;
					} while ( pLeft <= pHigh && pfnCompare( pLeft, pMid ) <= 0 );
				}

				do
				{
					pRight -= nSize;
				} while ( pRight > pMid && pfnCompare( pRight, pMid ) > 0 );

				if ( pRight < pLeft )
					break;

				SwapElements( pLeft, pRight, nSize );
				if ( pMid == pRight )
					pMid = pLeft;
			}

			pRight += nSize;
			if ( pMid < pRight )
			{
				do
				{
					pRight -= nSize;
				} while ( pRight > pMid && pfnCompare( pRight, pMid ) == 0 );
			}
			if ( pMid >= pRight )
			{
				do
				{
					pRight -= nSize;
				} while ( pRight > pLow && pfnCompare( pRight, pMid ) == 0 );
			}

			if ( pRight - pLow >= pHigh - pLeft )
			{
				if ( pLow < pRight )
				{
					lowStack[nStack] = pLow;
					highStack[nStack] = pRight;
					++nStack;
				}
				if ( pLeft < pHigh )
				{
					pLow = pLeft;
					continue;
				}
			}
			else
			{
				if ( pLeft < pHigh )
				{
					lowStack[nStack] = pLeft;
					highStack[nStack] = pHigh;
					++nStack;
				}
				if ( pLow < pRight )
				{
					pHigh = pRight;
					continue;
				}
			}
		}

		if ( --nStack < 0 )
			return;

		pLow = lowStack[nStack];
		pHigh = highStack[nStack];
	}
}
