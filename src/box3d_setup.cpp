#include "box3d_setup.h"

#include "tier0/dbg.h"
#include "tier0/platform.h"

#include "box3d/base.h"
#include "box3d/constants.h"

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
	b3SetLengthUnitsPerMeter( 1.0f / 0.0254f );
	b3SetLogFcn( Box3DLog );
	b3SetAssertFcn( Box3DAssert );
}
