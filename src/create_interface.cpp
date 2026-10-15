#include "box3d_setup.h"
#include "physics_collision.h"
#include "physics_surface_props.h"

#include "tier1/interface.h"

#include <cstring>

DLL_EXPORT void *CreateInterface( const char *pName, int *pReturnCode )
{
	SetupBox3D();

	void *pInterface = nullptr;
	if ( !std::strcmp( pName, VPHYSICS_COLLISION_INTERFACE_VERSION ) )
		pInterface = static_cast<IPhysicsCollision *>( &g_PhysicsCollision );
	else if ( !std::strcmp( pName, VPHYSICS_SURFACEPROPS_INTERFACE_VERSION ) )
		pInterface = static_cast<IPhysicsSurfaceProps *>( &g_PhysicsSurfaceProps );

	if ( pReturnCode )
		*pReturnCode = pInterface ? IFACE_OK : IFACE_FAILED;

	return pInterface;
}
