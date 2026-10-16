#pragma once

#include "tier0/valve_minmax_off.h"
#include "vcollide_parse.h"
#include "vphysics/constraints.h"
#include "vphysics/vehicles.h"

class CVPhysicsKeyParser final : public IVPhysicsKeyParser
{
public:
	explicit CVPhysicsKeyParser( const char *pKeyData );

	const char *GetCurrentBlockName() override;
	bool Finished() override;
	void ParseSolid( solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void ParseFluid( fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void ParseRagdollConstraint( constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void ParseSurfaceTable( int *table, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void ParseCustom( void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void ParseVehicle( vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void SkipBlock() override;

private:
	static constexpr size_t MAX_TOKEN_LENGTH = 1024;

	bool ReadKeyValue( char *pKey, char *pValue );
	void NextBlock();

	void ParseAxle( vehicle_axleparams_t &axle );
	void ParseWheel( vehicle_wheelparams_t &wheel );
	void ParseSuspension( vehicle_suspensionparams_t &suspension );
	void ParseBody( vehicle_bodyparams_t &body );
	void ParseEngine( vehicle_engineparams_t &engine );
	void ParseBoost( vehicle_engineparams_t &engine );
	void ParseSteering( vehicle_steeringparams_t &steering );

	const char *m_pText;
	char m_BlockName[MAX_TOKEN_LENGTH];
};
