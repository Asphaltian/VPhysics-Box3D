#include "box3d_setup.h"

#include "conversion.h"

#include "tier0/dbg.h"
#include "tier0/memalloc.h"
#include "tier0/platform.h"

#include "box3d/base.h"
#include "box3d/constants.h"

static void *Box3DAlloc( size_t nSize, int32_t nAlignment )
{
	return MemAlloc_AllocAligned( nSize, nAlignment );
}

static void Box3DFree( void *pMemory, size_t )
{
	MemAlloc_FreeAligned( pMemory );
}

static void Box3DLog( const char *pMessage )
{
	DevMsg( 2, "Box3D: %s\n", pMessage );
}

static int Box3DAssert( const char *pCondition, const char *pFileName, int nLineNumber )
{
	Warning( "BOX3D ASSERTION: %s, %s, line %d\n", pCondition, pFileName, nLineNumber );
	return Plat_IsInDebugSession() ? 1 : 0;
}

void SetupBox3D()
{
	b3SetAllocator( Box3DAlloc, Box3DFree );
	b3SetLengthUnitsPerMeter( IVP_TO_SOURCE );
	b3SetLogFcn( Box3DLog );
	b3SetAssertFcn( Box3DAssert );
}
