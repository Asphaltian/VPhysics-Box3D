#include "physics_controller.h"

#include "physics_environment.h"
#include "physics_object.h"

void CGravityController::Simulate( CPhysicsObject *pObject, float flTimestep )
{
	pObject->ApplyGravity( flTimestep, pObject->GetEnvironment()->GetIVPGravity(), true );
}

void CDragController::Simulate( CPhysicsObject *pObject, float flTimestep )
{
	pObject->ApplyDrag( flTimestep, pObject->GetEnvironment()->GetAirDensity() );
}
