#pragma once

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

void QhullSort( void *pBase, size_t nCount, size_t nSize, int ( *pfnCompare )( const void *, const void * ) );

#ifdef __cplusplus
}
#endif

#define qsort QhullSort
