#include "vphysics_key_parser.h"

#include "parse_file.h"
#include "physics_surface_props.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static bool EqualsCaseInsensitive( const char *pLhs, const char *pRhs )
{
	for ( ;; ++pLhs, ++pRhs )
	{
		char lhs = *pLhs;
		char rhs = *pRhs;
		if ( lhs >= 'A' && lhs <= 'Z' )
			lhs += 'a' - 'A';
		if ( rhs >= 'A' && rhs <= 'Z' )
			rhs += 'a' - 'A';
		if ( lhs != rhs )
			return false;
		if ( !lhs )
			return true;
	}
}

static void CopyString( char *pDest, const char *pSource, size_t nDestSize )
{
	const size_t nLength = strnlen( pSource, nDestSize - 1 );
	std::memcpy( pDest, pSource, nLength );
	std::memset( pDest + nLength, 0, nDestSize - nLength );
}

static float ReadFloat( const char *pValue )
{
	return static_cast<float>( std::atof( pValue ) );
}

static Vector ReadVector( const char *pValue )
{
	Vector vector( 0.0f, 0.0f, 0.0f );
	std::sscanf( pValue, "%f %f %f", &vector.x, &vector.y, &vector.z );
	return vector;
}

static Vector4D ReadVector4D( const char *pValue )
{
	Vector4D vector( 0.0f, 0.0f, 0.0f, 0.0f );
	std::sscanf( pValue, "%f %f %f %f", &vector.x, &vector.y, &vector.z, &vector.w );
	return vector;
}

CVPhysicsKeyParser::CVPhysicsKeyParser( const char *pKeyData )
: m_pText( pKeyData )
{
	m_BlockName[0] = '\0';
	NextBlock();
}

bool CVPhysicsKeyParser::ReadKeyValue( char *pKey, char *pValue )
{
	if ( !m_pText )
		return false;

	m_pText = ParseKeyValue( m_pText, pKey, pValue, MAX_TOKEN_LENGTH );
	return true;
}

void CVPhysicsKeyParser::NextBlock()
{
	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	do
	{
		if ( !ReadKeyValue( key, value ) )
		{
			m_BlockName[0] = '\0';
			return;
		}
	} while ( std::strcmp( value, "{" ) );

	CopyString( m_BlockName, key, sizeof( m_BlockName ) );
}

const char *CVPhysicsKeyParser::GetCurrentBlockName()
{
	return m_pText ? m_BlockName : nullptr;
}

bool CVPhysicsKeyParser::Finished()
{
	return !m_pText;
}

void CVPhysicsKeyParser::ParseSolid( solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
		unknownKeyHandler->SetDefaults( pSolid );
	else
		std::memset( static_cast<void *>( pSolid ), 0, sizeof( *pSolid ) );

	pSolid->params.enableCollisions = false;

	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
		{
			NextBlock();
			return;
		}

		if ( EqualsCaseInsensitive( key, "index" ) )
			pSolid->index = std::atoi( value );
		else if ( EqualsCaseInsensitive( key, "name" ) )
			CopyString( pSolid->name, value, sizeof( pSolid->name ) );
		else if ( EqualsCaseInsensitive( key, "parent" ) )
			CopyString( pSolid->parent, value, sizeof( pSolid->parent ) );
		else if ( EqualsCaseInsensitive( key, "surfaceprop" ) )
			CopyString( pSolid->surfaceprop, value, sizeof( pSolid->surfaceprop ) );
		else if ( EqualsCaseInsensitive( key, "mass" ) )
			pSolid->params.mass = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "massCenterOverride" ) )
		{
			pSolid->massCenterOverride = ReadVector( value );
			pSolid->params.massCenterOverride = &pSolid->massCenterOverride;
		}
		else if ( EqualsCaseInsensitive( key, "inertia" ) )
			pSolid->params.inertia = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "damping" ) )
			pSolid->params.damping = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "rotdamping" ) )
			pSolid->params.rotdamping = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "volume" ) )
			pSolid->params.volume = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "drag" ) )
			pSolid->params.dragCoefficient = ReadFloat( value );
		else if ( !EqualsCaseInsensitive( key, "rollingdrag" ) && unknownKeyHandler )
			unknownKeyHandler->ParseKeyValue( pSolid, key, value );
	}
}

void CVPhysicsKeyParser::ParseFluid( fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler )
{
	pFluid->index = -1;

	if ( unknownKeyHandler )
	{
		unknownKeyHandler->SetDefaults( pFluid );
	}
	else
	{
		std::memset( static_cast<void *>( pFluid ), 0, sizeof( *pFluid ) );
		CopyString( pFluid->surfaceprop, "water", sizeof( pFluid->surfaceprop ) );
	}

	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
		{
			NextBlock();
			return;
		}

		fluidparams_t &params = pFluid->params;
		if ( EqualsCaseInsensitive( key, "index" ) )
			pFluid->index = std::atoi( value );
		else if ( EqualsCaseInsensitive( key, "damping" ) )
			params.damping = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "surfaceplane" ) )
			params.surfacePlane = ReadVector4D( value );
		else if ( EqualsCaseInsensitive( key, "currentvelocity" ) )
			params.currentVelocity = ReadVector( value );
		else if ( EqualsCaseInsensitive( key, "contents" ) )
			params.contents = std::atoi( value );
		else if ( EqualsCaseInsensitive( key, "surfaceprop" ) )
			CopyString( pFluid->surfaceprop, value, sizeof( pFluid->surfaceprop ) );
		else if ( unknownKeyHandler )
			unknownKeyHandler->ParseKeyValue( pFluid, key, value );
	}
}

void CVPhysicsKeyParser::ParseRagdollConstraint( constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
	{
		unknownKeyHandler->SetDefaults( pConstraint );
	}
	else
	{
		std::memset( static_cast<void *>( pConstraint ), 0, sizeof( *pConstraint ) );
		pConstraint->parentIndex = -1;
		pConstraint->childIndex = -1;
	}

	pConstraint->useClockwiseRotations = true;

	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
		{
			NextBlock();
			return;
		}

		constraint_axislimit_t *axes = pConstraint->axes;
		if ( EqualsCaseInsensitive( key, "parent" ) )
			pConstraint->parentIndex = std::atoi( value );
		else if ( EqualsCaseInsensitive( key, "child" ) )
			pConstraint->childIndex = std::atoi( value );
		else if ( EqualsCaseInsensitive( key, "xmin" ) )
			axes[0].minRotation = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "xmax" ) )
			axes[0].maxRotation = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "xfriction" ) )
		{
			axes[0].angularVelocity = 0.0f;
			axes[0].torque = ReadFloat( value );
		}
		else if ( EqualsCaseInsensitive( key, "ymin" ) )
			axes[1].minRotation = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "ymax" ) )
			axes[1].maxRotation = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "yfriction" ) )
		{
			axes[1].angularVelocity = 0.0f;
			axes[1].torque = ReadFloat( value );
		}
		else if ( EqualsCaseInsensitive( key, "zmin" ) )
			axes[2].minRotation = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "zmax" ) )
			axes[2].maxRotation = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "zfriction" ) )
		{
			axes[2].angularVelocity = 0.0f;
			axes[2].torque = ReadFloat( value );
		}
		else if ( unknownKeyHandler )
			unknownKeyHandler->ParseKeyValue( pConstraint, key, value );
	}
}

void CVPhysicsKeyParser::ParseSurfaceTable( int *table, IVPhysicsKeyHandler * )
{
	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
		{
			NextBlock();
			return;
		}

		const int nSurfaceIndex = g_PhysicsSurfaceProps.GetSurfaceIndex( key );
		const unsigned int nTableIndex = static_cast<unsigned int>( std::atoi( value ) );
		if ( nTableIndex < 128 )
			table[nTableIndex] = nSurfaceIndex;
	}
}

void CVPhysicsKeyParser::ParseCustom( void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
		unknownKeyHandler->SetDefaults( pCustom );

	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];
	int nDepth = 0;

	while ( ReadKeyValue( key, value ) )
	{
		if ( !m_pText )
			continue;

		if ( key[0] == '{' )
		{
			++nDepth;
			continue;
		}

		if ( value[0] == '{' )
		{
			++nDepth;
		}
		else if ( key[0] == '}' )
		{
			if ( --nDepth < 0 )
			{
				NextBlock();
				return;
			}
			continue;
		}

		if ( unknownKeyHandler )
			unknownKeyHandler->ParseKeyValue( pCustom, key, value );
	}
}

void CVPhysicsKeyParser::ParseVehicle( vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
		unknownKeyHandler->SetDefaults( pVehicle );
	else
		std::memset( static_cast<void *>( pVehicle ), 0, sizeof( *pVehicle ) );

	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
		{
			NextBlock();
			return;
		}

		if ( value[0] == '{' )
		{
			if ( EqualsCaseInsensitive( key, "axle" ) )
			{
				if ( static_cast<unsigned int>( pVehicle->axleCount ) < VEHICLE_MAX_AXLE_COUNT )
				{
					ParseAxle( pVehicle->axles[pVehicle->axleCount] );
					++pVehicle->axleCount;
				}
			}
			else if ( EqualsCaseInsensitive( key, "body" ) )
				ParseBody( pVehicle->body );
			else if ( EqualsCaseInsensitive( key, "engine" ) )
				ParseEngine( pVehicle->engine );
			else if ( EqualsCaseInsensitive( key, "steering" ) )
				ParseSteering( pVehicle->steering );
			else
				SkipBlock();
		}
		else if ( EqualsCaseInsensitive( key, "wheelsperaxle" ) )
		{
			pVehicle->wheelsPerAxle = std::atoi( value );
		}
	}
}

void CVPhysicsKeyParser::ParseAxle( vehicle_axleparams_t &axle )
{
	std::memset( static_cast<void *>( &axle ), 0, sizeof( axle ) );

	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
			return;

		if ( value[0] == '{' )
		{
			if ( EqualsCaseInsensitive( key, "wheel" ) )
				ParseWheel( axle.wheels );
			else if ( EqualsCaseInsensitive( key, "suspension" ) )
				ParseSuspension( axle.suspension );
			else
				SkipBlock();
		}
		else if ( EqualsCaseInsensitive( key, "offset" ) )
			axle.offset = ReadVector( value );
		else if ( EqualsCaseInsensitive( key, "wheeloffset" ) )
			axle.wheelOffset = ReadVector( value );
		else if ( EqualsCaseInsensitive( key, "torquefactor" ) )
			axle.torqueFactor = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "brakefactor" ) )
			axle.brakeFactor = ReadFloat( value );
	}
}

void CVPhysicsKeyParser::ParseWheel( vehicle_wheelparams_t &wheel )
{
	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
			return;

		if ( EqualsCaseInsensitive( key, "radius" ) )
			wheel.radius = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "mass" ) )
			wheel.mass = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "inertia" ) )
			wheel.inertia = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "damping" ) )
			wheel.damping = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "rotdamping" ) )
			wheel.rotdamping = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "frictionscale" ) )
			wheel.frictionScale = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "material" ) )
			wheel.materialIndex = g_PhysicsSurfaceProps.GetSurfaceIndex( value );
		else if ( EqualsCaseInsensitive( key, "skidmaterial" ) )
			wheel.skidMaterialIndex = g_PhysicsSurfaceProps.GetSurfaceIndex( value );
		else if ( EqualsCaseInsensitive( key, "brakematerial" ) )
			wheel.brakeMaterialIndex = g_PhysicsSurfaceProps.GetSurfaceIndex( value );
	}
}

void CVPhysicsKeyParser::ParseSuspension( vehicle_suspensionparams_t &suspension )
{
	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
			return;

		if ( EqualsCaseInsensitive( key, "springconstant" ) )
			suspension.springConstant = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "springdamping" ) )
			suspension.springDamping = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "stabilizerconstant" ) )
			suspension.stabilizerConstant = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "springdampingcompression" ) )
			suspension.springDampingCompression = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "maxbodyforce" ) )
			suspension.maxBodyForce = ReadFloat( value );
	}
}

void CVPhysicsKeyParser::ParseBody( vehicle_bodyparams_t &body )
{
	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
			return;

		if ( EqualsCaseInsensitive( key, "massCenterOverride" ) )
			body.massCenterOverride = ReadVector( value );
		else if ( EqualsCaseInsensitive( key, "addgravity" ) )
			body.addGravity = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "maxAngularVelocity" ) )
			body.maxAngularVelocity = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "massOverride" ) )
			body.massOverride = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "tiltforce" ) )
			body.tiltForce = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "tiltforceheight" ) )
			body.tiltForceHeight = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "countertorquefactor" ) )
			body.counterTorqueFactor = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "keepuprighttorque" ) )
			body.keepUprightTorque = ReadFloat( value );
	}
}

void CVPhysicsKeyParser::ParseEngine( vehicle_engineparams_t &engine )
{
	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
			return;

		if ( value[0] == '{' )
		{
			if ( EqualsCaseInsensitive( key, "boost" ) )
				ParseBoost( engine );
			else
				SkipBlock();
		}
		else if ( EqualsCaseInsensitive( key, "gear" ) )
		{
			if ( static_cast<unsigned int>( engine.gearCount ) < VEHICLE_MAX_GEAR_COUNT )
			{
				engine.gearRatio[engine.gearCount] = ReadFloat( value );
				++engine.gearCount;
			}
		}
		else if ( EqualsCaseInsensitive( key, "horsepower" ) )
			engine.horsepower = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "maxSpeed" ) )
			engine.maxSpeed = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "maxReverseSpeed" ) )
			engine.maxRevSpeed = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "axleratio" ) )
			engine.axleRatio = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "maxRPM" ) )
			engine.maxRPM = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "throttleTime" ) )
			engine.throttleTime = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "AutoTransmission" ) )
			engine.isAutoTransmission = std::atoi( value ) != 0;
		else if ( EqualsCaseInsensitive( key, "shiftUpRPM" ) )
			engine.shiftUpRPM = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "shiftDownRPM" ) )
			engine.shiftDownRPM = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "autobrakeSpeedGain" ) )
			engine.autobrakeSpeedGain = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "autobrakeSpeedFactor" ) )
			engine.autobrakeSpeedFactor = ReadFloat( value );
	}
}

void CVPhysicsKeyParser::ParseBoost( vehicle_engineparams_t &engine )
{
	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
			return;

		if ( EqualsCaseInsensitive( key, "force" ) )
			engine.boostForce = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "duration" ) )
			engine.boostDuration = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "delay" ) )
			engine.boostDelay = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "maxspeed" ) )
			engine.boostMaxSpeed = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "torqueboost" ) )
			engine.torqueBoost = std::atoi( value ) != 0;
	}
}

void CVPhysicsKeyParser::ParseSteering( vehicle_steeringparams_t &steering )
{
	char key[MAX_TOKEN_LENGTH];
	char value[MAX_TOKEN_LENGTH];

	while ( ReadKeyValue( key, value ) )
	{
		if ( key[0] == '}' )
			return;

		if ( EqualsCaseInsensitive( key, "degreesSlow" ) )
			steering.degreesSlow = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "degreesFast" ) )
			steering.degreesFast = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "degreesBoost" ) )
			steering.degreesBoost = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "fastcarspeed" ) )
			steering.speedFast = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "slowcarspeed" ) )
			steering.speedSlow = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "slowsteeringrate" ) )
			steering.steeringRateSlow = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "faststeeringrate" ) )
			steering.steeringRateFast = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "steeringRestRateSlow" ) )
			steering.steeringRestRateSlow = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "steeringRestRateFast" ) )
			steering.steeringRestRateFast = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "throttleSteeringRestRateFactor" ) )
			steering.throttleSteeringRestRateFactor = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "boostSteeringRestRateFactor" ) )
			steering.boostSteeringRestRateFactor = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "boostSteeringRateFactor" ) )
			steering.boostSteeringRateFactor = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "steeringExponent" ) )
			steering.steeringExponent = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "turnThrottleReduceSlow" ) )
			steering.turnThrottleReduceSlow = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "turnThrottleReduceFast" ) )
			steering.turnThrottleReduceFast = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "brakeSteeringRateFactor" ) )
			steering.brakeSteeringRateFactor = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "powerSlideAccel" ) )
			steering.powerSlideAccel = ReadFloat( value );
		else if ( EqualsCaseInsensitive( key, "skidallowed" ) )
			steering.isSkidAllowed = std::atoi( value ) != 0;
		else if ( EqualsCaseInsensitive( key, "dustcloud" ) )
			steering.dustCloud = std::atoi( value ) != 0;
	}
}

void CVPhysicsKeyParser::SkipBlock()
{
	ParseCustom( nullptr, nullptr );
}
