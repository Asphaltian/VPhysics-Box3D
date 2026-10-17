#pragma once

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

int BuildQhull( char *pCommand, double *pPoints, int nCount, FILE *pOutput );

#ifdef __cplusplus
}
#endif
