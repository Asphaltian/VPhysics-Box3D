#pragma once

#include <cstddef>

class CPhysCollide;

CPhysCollide *CreateCollideFromCompactSurface( const void *pSurface, size_t nSize, bool bSwap );
