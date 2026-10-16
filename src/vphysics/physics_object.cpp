#include "physics_object.h"

#include "conversion.h"
#include "physics_collide.h"
#include "physics_collision.h"
#include "physics_constraint.h"
#include "physics_controller.h"
#include "physics_environment.h"
#include "physics_fluid_controller.h"
#include "physics_friction_snapshot.h"
#include "physics_spring.h"
#include "physics_surface_props.h"
#include "vphysics_saverestore.h"

#include "tier0/dbg.h"
#include "tier0/platform.h"

#include "box3d/box3d.h"

#include <algorithm>
#include <cmath>
#include <new>
#include <numbers>
#include <vector>

static constexpr float MAX_INERTIA = 1e18f;
static constexpr float MIN_BUOYANCY_VOLUME = 5.0f;
static constexpr float INERTIA_SCALE = IVP_TO_SOURCE * IVP_TO_SOURCE;
static constexpr float ENERGY_SCALE = 775.0016f;
static constexpr float SPHERE_VOLUME_SCALE = 1.0471976f;
static constexpr float SPHERE_INERTIA_SCALE = 0.4f;
static constexpr float LINEAR_DAMPING_EXPONENTIAL_LIMIT = 0.25f;
static constexpr float ANGULAR_DAMPING_EXPONENTIAL_LIMIT = 0.5f;
static constexpr float STEP_ANGULAR_CLAMP_SCALE = 0.9f;
static constexpr float STEP_LINEAR_CLAMP_SCALE = 0.99f;
static constexpr float REST_DISTANCE = 0.01f;
static constexpr float REST_TIME = 0.3f;
static constexpr float SLEEP_THRESHOLD = REST_DISTANCE / REST_TIME * IVP_TO_SOURCE;
static constexpr float MIN_STEP_INTERVAL = 1e-10f;
static constexpr float MAX_INVERSE_STEP_INTERVAL = 1e10f;
static constexpr float SHADOW_REST_SPEED_SQUARED = 1e-6f;
static constexpr float AXIS_LENGTH_BIAS = 1e-10f;
static constexpr float MASS_CENTER_OVERRIDE_DISTANCE = 1.0f / 256.0f;
static constexpr short COLLIDE_TYPE_POLYGON = 0;
static constexpr short COLLIDE_TYPE_SPHERE = 2;
#if defined( VPHYSICS_GMOD )
static constexpr double NO_SHIFT_DISTANCE_SQUARED = 1e-16;
#else
static constexpr float NO_SHIFT_DISTANCE_SQUARED = 9e-6f;
#endif

static RealVec3_t TransformShift( const RealMatrix3_t &rotation, b3Vec3 shift, RealVec3_t origin )
{
	const RealVec3_t rotated = Rotate( rotation, shift );
	return { rotated.x + origin.x, rotated.y + origin.y, rotated.z + origin.z };
}

static void MatrixTransformPoint( const Vector &v, const matrix3x4_t &m, Vector &out )
{
	for ( int i = 0; i < 3; ++i )
		out[i] = ( v.x * m[i][0] + v.y * m[i][1] ) + ( v.z * m[i][2] + m[i][3] );
}

static void MatrixInverseTransformPoint( const Vector &v, const matrix3x4_t &m, Vector &out )
{
	const Vector d( v.x - m[0][3], v.y - m[1][3], v.z - m[2][3] );
	for ( int i = 0; i < 3; ++i )
		out[i] = ( d.x * m[0][i] + d.y * m[1][i] ) + d.z * m[2][i];
}

template<typename Function>
static void ForEachShape( b3BodyId body, Function function )
{
	static thread_local std::vector<b3ShapeId> s_Shapes;
	s_Shapes.resize( b3Body_GetShapeCount( body ) );
	b3Body_GetShapes( body, s_Shapes.data(), static_cast<int>( s_Shapes.size() ) );
	for ( b3ShapeId shape : s_Shapes )
		function( shape );
}

static b3Filter GetCollisionFilter( bool bEnabled )
{
	b3Filter filter = b3DefaultFilter();
	if ( !bEnabled )
		filter.maskBits = 0;
	return filter;
}

static b3SurfaceMaterial GetSurfaceMaterial( int materialIndex )
{
	float flDensity, flThickness, flFriction, flElasticity;
	g_PhysicsSurfaceProps.GetPhysicsProperties( materialIndex, &flDensity, &flThickness, &flFriction, &flElasticity );

	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = flFriction;
	material.restitution = flElasticity;
	material.userMaterialId = static_cast<uint64_t>( materialIndex );
	return material;
}

static b3Vec3 ClampToLength( b3Vec3 v, float flMaxLength )
{
	const float flLength = b3Length( v );
	return flLength > flMaxLength ? b3MulSV( flMaxLength / flLength, v ) : v;
}

static b3Vec3 ComputeControlVelocity( b3Vec3 velocity, b3Vec3 delta, float flMaxSpeed, float flMaxDampSpeed, float flScale, float flDampFactor, b3Vec3 *pAcceleration )
{
	if ( b3LengthSquared( velocity ) < SHADOW_REST_SPEED_SQUARED )
		velocity = b3Vec3_zero;

	const b3Vec3 acceleration = flMaxSpeed <= 0.0f ? b3Vec3_zero : ClampToLength( b3MulSV( flScale, delta ), flMaxSpeed );
	const b3Vec3 damping = flMaxDampSpeed <= 0.0f ? b3Vec3_zero : ClampToLength( b3MulSV( -flDampFactor, velocity ), flMaxDampSpeed );
	if ( pAcceleration )
		*pAcceleration = acceleration;
	return b3Add( b3Add( damping, velocity ), acceleration );
}

static b3Vec3 GetRotationDelta( RealQuat_t from, RealQuat_t to )
{
	const RealQuat_t delta = NormalizeQuat( MultiplyQuat( ConjugateQuat( from ), to ) );

	double flAngle = 2.0 * std::acos( std::clamp( static_cast<double>( delta.w ), -1.0, 1.0 ) );
#if defined( VPHYSICS_GMOD )
	float flWrapped = static_cast<float>( flAngle );
	if ( flWrapped > 3.1415927f )
		flWrapped -= 6.2831855f;
#else
	if ( flAngle > std::numbers::pi )
		flAngle -= 2.0 * std::numbers::pi;
	const float flWrapped = static_cast<float>( flAngle );
#endif

	b3Vec3 axis = ToFloat( { delta.x, delta.y, delta.z } );
	axis = b3MulSV( FastInverseSquareRoot( b3LengthSquared( axis ) + AXIS_LENGTH_BIAS ), axis );
	axis = b3MulSV( FastInverseSquareRoot( b3LengthSquared( axis ) + AXIS_LENGTH_BIAS ), axis );
	return b3MulSV( flWrapped, axis );
}

CPhysicsObject::CPhysicsObject( CPhysicsEnvironment *pEnvironment, const objectparams_t &params, bool isStatic )
: m_pEnvironment( pEnvironment )
, m_nId( pEnvironment->AllocateObjectId() )
, m_Name( params.pName ? params.pName : "" )
, m_bStatic( isStatic )
, m_bHasName( params.pName != nullptr )
{
}

CPhysicsObject *CPhysicsObject::CreatePolyObject( CPhysicsEnvironment *pEnvironment, const CPhysCollide *pCollide, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic )
{
	if ( materialIndex < 0 )
		materialIndex = g_PhysicsSurfaceProps.GetSurfaceIndex( "default" );

	Vector origin = position;
	QAngle rotation = angles;
	if ( !origin.IsValid() || !rotation.IsValid() )
	{
		if ( Plat_IsInDebugSession() )
			DebuggerBreak();

		Warning( "Invalid initial position on %s\n", pParams->pName );
		if ( !origin.IsValid() )
			origin.Init();
		if ( !rotation.IsValid() )
			rotation.Init();
	}

	if ( !pCollide )
		return nullptr;

	b3Vec3 massCenter = SourceToIVP( pCollide->GetMassCenter() );
	b3Vec3 unitInertia = b3Vec3_zero;
	if ( const CPhysCollideCompactSurface *pSurface = pCollide->GetCompactSurface() )
	{
		massCenter = pSurface->GetIVPMassCenter();
		const Vector inertia = pSurface->GetRotationInertia();
		unitInertia = { inertia.x, inertia.z, inertia.y };
	}

	if ( pParams->massCenterOverride )
		massCenter = SourceToIVP( *pParams->massCenterOverride );

	CPhysicsObject *pObject = new CPhysicsObject( pEnvironment, *pParams, isStatic );
	pObject->m_pCollide = pCollide;
	pObject->ComputeMassProperties( *pParams, massCenter, unitInertia );
	pObject->m_bAsleepSinceCreation = true;
	pObject->Initialize( *pParams, materialIndex, ToReal( SourceToIVP( origin ) ), AnglesToIVP( rotation ), massCenter, pParams->volume );
	if ( !isStatic && pParams->dragCoefficient != 0.0f )
		pObject->EnableDrag( true );
	return pObject;
}

CPhysicsObject *CPhysicsObject::CreateSphereObject( CPhysicsEnvironment *pEnvironment, float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic )
{
	const float flRadius = radius * SOURCE_TO_IVP;
	const float flUnitInertia = ( flRadius * SPHERE_INERTIA_SCALE ) * flRadius;

	CPhysicsObject *pObject = new CPhysicsObject( pEnvironment, *pParams, isStatic );
	pObject->m_flSphereRadius = flRadius;
	pObject->ComputeMassProperties( *pParams, b3Vec3_zero, { flUnitInertia, flUnitInertia, flUnitInertia } );
	const float flVolume = pParams->volume > 0.0f ? pParams->volume : ( ( radius * radius ) * radius ) * 4.0f * SPHERE_VOLUME_SCALE;
	pObject->Initialize( *pParams, materialIndex, ToReal( SourceToIVP( position ) ), AnglesToIVP( angles ), b3Vec3_zero, flVolume );
	return pObject;
}

static objectparams_t GetRecordParams( const vphysics_save_cphysicsobject_t &record )
{
	objectparams_t params = {};
	params.mass = record.mass;
	params.damping = record.speedDamping;
	params.rotdamping = record.rotSpeedDamping;
	params.dragCoefficient = record.dragCoefficient;
	params.pName = record.pName;
	return params;
}

CPhysicsObject *CPhysicsObject::CreateFromRecord( CPhysicsEnvironment *pEnvironment, const vphysics_save_cphysicsobject_t &record, void *pGameData )
{
	if ( record.collideType != COLLIDE_TYPE_SPHERE && !record.pCollide )
		return nullptr;

	const objectparams_t params = GetRecordParams( record );
	CPhysicsObject *pObject = new CPhysicsObject( pEnvironment, params, record.isStatic );
	pObject->InitializeFromRecord( params, record, pGameData );
	return pObject;
}

void CPhysicsObject::MoveToEnvironment( CPhysicsEnvironment *pDestination, const vphysics_save_cphysicsobject_t &record, void *pGameData )
{
	this->~CPhysicsObject();
	const objectparams_t params = GetRecordParams( record );
	new ( this ) CPhysicsObject( pDestination, params, record.isStatic );
	InitializeFromRecord( params, record, pGameData );
}

void CPhysicsObject::InitializeFromRecord( const objectparams_t &params, const vphysics_save_cphysicsobject_t &record, void *pGameData )
{
	const bool bSphere = record.collideType == COLLIDE_TYPE_SPHERE;
	b3Vec3 massCenter = b3Vec3_zero;
	if ( bSphere )
	{
		m_flSphereRadius = record.sphereRadius * SOURCE_TO_IVP;
	}
	else
	{
		m_pCollide = record.pCollide;
		massCenter = SourceToIVP( m_pCollide->GetMassCenter() );
		if ( const CPhysCollideCompactSurface *pSurface = m_pCollide->GetCompactSurface() )
			massCenter = pSurface->GetIVPMassCenter();
	}

	if ( record.massCenterOverride.x != 0.0f || record.massCenterOverride.y != 0.0f || record.massCenterOverride.z != 0.0f )
		massCenter = SourceToIVP( record.massCenterOverride );

	ComputeMassProperties( params, massCenter, b3Vec3_zero );
	m_flMass = record.mass;
	m_Inertia = Vector( std::fabs( record.rotInertia.x ), std::fabs( record.rotInertia.y ), std::fabs( record.rotInertia.z ) );
	UpdateInverseMass();

	const int nMaterialIndex = record.materialIndex < 0 ? g_PhysicsSurfaceProps.GetSurfaceIndex( "default" ) : record.materialIndex;
	Initialize( params, nMaterialIndex, ToReal( SourceToIVP( record.origin ) ), AnglesToIVP( record.angles ), massCenter, record.volume );

	SetCallbackFlags( static_cast<unsigned short>( record.callbacks ) );
	SetGameFlags( static_cast<unsigned short>( record.gameFlags ) );
	SetGameIndex( static_cast<unsigned short>( record.gameIndex ) );
	SetGameData( pGameData );
	SetContents( static_cast<unsigned int>( record.contentsMask ) );

	if ( record.dragEnabled )
		EnableDrag( true );
	if ( !record.motionEnabled )
		EnableMotion( false );
	if ( record.isTrigger )
		BecomeTrigger();
	if ( !record.gravityEnabled )
		EnableGravity( false );
	if ( record.collisionEnabled )
		EnableCollisions( true );

	if ( record.velocity.LengthSqr() != 0.0f || record.angVelocity.LengthSqr() != 0.0f )
	{
		SetVelocityInstantaneous( &record.velocity, &record.angVelocity );
		if ( record.isAsleep )
			Sleep();
	}

	m_bAsleepSinceCreation = record.asleepSinceCreation;
	if ( !record.isAsleep )
		Wake();
	if ( record.hingeAxis != 0 )
		BecomeHinged( record.hingeAxis - 1 );
	m_bHasTouchedDynamic = record.hasTouchedDynamic;
}

void CPhysicsObject::WriteRecord( vphysics_save_cphysicsobject_t &record ) const
{
	record.pCollide = m_pCollide;
	record.sphereRadius = m_pCollide ? 0.0f : GetSphereRadius();
	record.isStatic = m_bStatic;
	record.collisionEnabled = m_bCollisionEnabled;
	record.gravityEnabled = IsGravityEnabled();
	record.dragEnabled = IsDragEnabled();
	record.motionEnabled = m_bMotionEnabled;
	record.isAsleep = IsAsleep();
	record.isTrigger = m_bTrigger;
	record.asleepSinceCreation = m_bAsleepSinceCreation;
	record.hasTouchedDynamic = m_bHasTouchedDynamic;
	record.hasShadowController = m_pShadow != nullptr;
	record.collideType = m_pCollide ? COLLIDE_TYPE_POLYGON : COLLIDE_TYPE_SPHERE;
	record.gameIndex = static_cast<short>( m_nGameIndex );
	record.hingeAxis = m_nHingedAxis + 1;
	record.materialIndex = m_nMaterialIndex;
	record.mass = GetMass();
	record.rotInertia = GetInertia();
	GetDamping( &record.speedDamping, &record.rotSpeedDamping );

	Vector collideMassCenter( 0.0f, 0.0f, 0.0f );
	if ( m_pCollide )
	{
		const CPhysCollideCompactSurface *pSurface = m_pCollide->GetCompactSurface();
		collideMassCenter = pSurface ? ToSource( IVPToSource( pSurface->GetIVPMassCenter() ) ) : m_pCollide->GetMassCenter();
	}
	const Vector massCenterOffset = m_MassCenter - collideMassCenter;
	record.massCenterOverride = std::sqrt( massCenterOffset.LengthSqr() ) > MASS_CENTER_OVERRIDE_DISTANCE ? m_MassCenter : Vector( 0.0f, 0.0f, 0.0f );

	record.callbacks = m_nCallbackFlags;
	record.gameFlags = m_nGameFlags;
	record.contentsMask = static_cast<int>( m_nContents );
	record.volume = m_flVolume;
	record.dragCoefficient = m_flDragCoefficient;
	record.angDragCoefficient = m_flAngularDragCoefficient;
	record.pShadow = m_pShadow;
	GetPosition( &record.origin, &record.angles );
	GetVelocity( &record.velocity, &record.angVelocity );
}

void CPhysicsObject::Initialize( const objectparams_t &params, int materialIndex, RealVec3_t origin, RealQuat_t rotation, b3Vec3 massCenter, float flVolume )
{
	m_nMaterialIndex = static_cast<unsigned short>( materialIndex );
	m_nCallbackFlags = CALLBACK_GLOBAL_COLLISION | CALLBACK_GLOBAL_FRICTION | CALLBACK_GLOBAL_TOUCH | CALLBACK_GLOBAL_COLLIDE_STATIC | CALLBACK_FLUID_TOUCH | CALLBACK_DO_FLUID_SIMULATION;

	m_flSpeedDamping = params.damping;
	m_flRotDamping = params.rotdamping;

	InitializePose( origin, rotation, massCenter );
	const b3Transform transform = GetBodyTransform();

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = m_bStatic ? b3_staticBody : b3_dynamicBody;
	bodyDef.position = transform.p;
	bodyDef.rotation = transform.q;
	bodyDef.isAwake = false;
	bodyDef.sleepThreshold = SLEEP_THRESHOLD;
	bodyDef.userData = this;
	m_Body = b3CreateBody( m_pEnvironment->GetWorld(), &bodyDef );

	m_bCollisionEnabled = params.enableCollisions;
	CreateShapes( false );
	if ( m_bCollisionEnabled )
	{
		m_nCallbackFlags |= CALLBACK_ENABLING_COLLISION;
		m_pEnvironment->ResolveFilterDecisions( this, true );
		m_nCallbackFlags &= ~CALLBACK_ENABLING_COLLISION;
	}
	UpdateMassData();
	if ( !m_bStatic )
		AttachController( m_pEnvironment->GetGravityController() );

	if ( !m_bStatic && m_pCollide )
	{
		m_flDragCoefficient = params.dragCoefficient;
		m_flAngularDragCoefficient = params.dragCoefficient;
		ComputeDragBases();
	}

	SetVolume( flVolume );
	SetGameData( params.pGameData );
}

void CPhysicsObject::CreateShapes( bool bSensor )
{
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial = GetSurfaceMaterial( m_nMaterialIndex );
	shapeDef.filter = GetCollisionFilter( m_bCollisionEnabled );
	shapeDef.isSensor = bSensor;
	shapeDef.enableSensorEvents = true;
	shapeDef.enableContactEvents = true;
	shapeDef.enableHitEvents = true;
	shapeDef.enableCustomFiltering = true;
	shapeDef.enablePreSolveEvents = true;
	shapeDef.updateBodyMass = false;
	if ( m_pCollide )
	{
		for ( const std::unique_ptr<CPhysConvex> &pConvex : m_pCollide->GetConvexes() )
		{
			const b3HullData *pHull = pConvex->GetHull();
			if ( !pHull )
				continue;

			shapeDef.userData = const_cast<CPhysConvex *>( pConvex.get() );
			b3CreateHullShape( m_Body, &shapeDef, pHull );
		}
		return;
	}

	const b3Sphere sphere = { b3Vec3_zero, GetSphereRadius() };
	b3CreateSphereShape( m_Body, &shapeDef, &sphere );
}

void CPhysicsObject::DestroyShapes()
{
	m_pEnvironment->RemoveVisitorOverlaps( this );
	ForEachShape( m_Body, []( b3ShapeId shape ) { b3DestroyShape( shape, false ); } );
}

void CPhysicsObject::ResetShapeProxies()
{
	m_pEnvironment->PrepareShapeReset( this );
	ForEachShape( m_Body, []( b3ShapeId shape ) {
		const b3Filter filter = b3Shape_GetFilter( shape );
		b3Filter changed = filter;
		changed.maskBits = ~filter.maskBits;
		b3Shape_SetFilter( shape, changed, false );
		b3Shape_SetFilter( shape, filter, true );
	} );
}

bool CPhysicsObject::FindFilterDecision( const CPhysicsObject *pPartner, bool &bCollide ) const
{
	auto it = m_FilterDecisions.find( pPartner->m_nId );
	if ( it == m_FilterDecisions.end() )
		return false;

	bCollide = it->second.m_bCollide;
	return true;
}

void CPhysicsObject::StoreFilterDecision( CPhysicsObject *pPartner, bool bCollide )
{
	m_FilterDecisions[pPartner->m_nId] = { pPartner, bCollide };
	pPartner->m_FilterDecisions[m_nId] = { this, bCollide };
}

void CPhysicsObject::ClearFilterDecisions()
{
	for ( const auto &[nPartner, decision] : m_FilterDecisions )
		decision.m_pPartner->m_FilterDecisions.erase( m_nId );
	m_FilterDecisions.clear();
}

void CPhysicsObject::QueueVelocityFlush()
{
	if ( !m_bVelocityFlushQueued )
	{
		m_bVelocityFlushQueued = true;
		m_pEnvironment->QueueVelocityFlush( this );
	}
}

b3Vec3 CPhysicsObject::GetBodyAngularVelocity( b3Vec3 coreAngular ) const
{
	return b3RotateVector( IVPToBox3D( m_NextRotation ), { coreAngular.x, coreAngular.z, -coreAngular.y } );
}

void CPhysicsObject::FlushVelocity()
{
	m_bVelocityFlushQueued = false;
	if ( m_bCoreVelocityValid )
	{
		b3Vec3 velocity = IVPToSource( m_CoreVelocity );
		if ( m_bGravityApplied )
			velocity = b3MulSub( velocity, m_pEnvironment->GetSimulationTimestep(), ToBox3D( m_pEnvironment->GetGravityVector() ) );
		b3Body_SetLinearVelocity( m_Body, velocity );
	}
	if ( m_bCoreAngularVelocityValid )
		b3Body_SetAngularVelocity( m_Body, GetBodyAngularVelocity( m_CoreAngularVelocity ) );
	m_bGravityApplied = false;
}

void CPhysicsObject::CapturePreStepVelocity( uint32_t nStepCount )
{
	m_PreStepVelocity = IVPToSource( GetCoreVelocity() );
	m_PreStepAngularVelocity = GetBodyAngularVelocity( GetCoreAngularVelocity() );
	m_nPreStepCount = nStepCount;
}

Vector CPhysicsObject::GetPreStepVelocityAtPoint( const Vector &worldPosition ) const
{
	if ( m_nPreStepCount != m_pEnvironment->GetStepCount() )
		return Vector( 0.0f, 0.0f, 0.0f );

	const b3Vec3 offset = b3Sub( ToBox3D( worldPosition ), b3ToVec3( b3Body_GetWorldCenter( m_Body ) ) );
	return ToSource( b3Add( m_PreStepVelocity, b3Cross( m_PreStepAngularVelocity, offset ) ) );
}

void CPhysicsObject::InitializePose( RealVec3_t origin, RealQuat_t rotation, b3Vec3 massCenter )
{
	const RealMatrix3_t matrix = QuatToMatrix( NormalizeQuat( rotation ) );
	const auto &a = matrix.m;
	RealMatrix3_t frame;
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			const real_t b0 = j == 0 ? real_t( 1 ) : real_t( 0 );
			const real_t b1 = j == 1 ? real_t( 1 ) : real_t( 0 );
			const real_t b2 = j == 2 ? real_t( 1 ) : real_t( 0 );
			frame.m[i][j] = ( a[i][0] * b0 + a[i][1] * b1 ) + a[i][2] * b2;
		}
	}

	const real_t c[3] = { massCenter.x, massCenter.y, massCenter.z };
	const real_t p[3] = { origin.x, origin.y, origin.z };
	real_t core[3];
	for ( int k = 0; k < 3; ++k )
		core[k] = ( c[0] * a[k][0] + c[2] * a[k][2] ) + ( c[1] * a[k][1] + p[k] );

	const RealQuat_t coreRotation = MatrixToQuat( frame );
	m_LastRotation = coreRotation;
	m_NextRotation = coreRotation;
	m_CorePosition = { core[0], core[1], core[2] };
	m_CoreRotation = frame;
	m_ImplicitVelocity = b3Vec3_zero;
	m_flLastStepTime = 0.0;

	const float flInterval = static_cast<float>( m_pEnvironment->GetNextStepTime() - m_pEnvironment->GetCurrentTime() );
	m_flInverseStepInterval = flInterval <= MIN_STEP_INTERVAL ? MAX_INVERSE_STEP_INTERVAL : 1.0f / flInterval;
}

RealVec3_t CPhysicsObject::GetCorePositionAt( double flTime ) const
{
	const real_t flElapsed = static_cast<float>( flTime - m_flLastStepTime );
	return {
		real_t( m_ImplicitVelocity.x ) * flElapsed + m_CorePosition.x,
		real_t( m_ImplicitVelocity.y ) * flElapsed + m_CorePosition.y,
		real_t( m_ImplicitVelocity.z ) * flElapsed + m_CorePosition.z,
	};
}

void CPhysicsObject::GetPose( double flTime, RealMatrix3_t &rotation, RealVec3_t &origin ) const
{
	const RealVec3_t position = GetCorePositionAt( flTime );
	rotation = QuatToMatrix( GetRotationAt( flTime ) );
	origin = m_bNoShift ? position : TransformShift( rotation, m_Shift, position );
}

RealQuat_t CPhysicsObject::GetRotationAt( double flTime ) const
{
	const float flElapsed = static_cast<float>( flTime - m_flLastStepTime );
	return InterpolateQuat( m_LastRotation, m_NextRotation, flElapsed * m_flInverseStepInterval );
}

b3Transform CPhysicsObject::GetBodyTransform() const
{
	const RealMatrix3_t rotation = QuatToMatrix( m_NextRotation );
	const RealVec3_t origin = m_bNoShift ? m_CorePosition : TransformShift( rotation, m_Shift, m_CorePosition );
	return { ToBox3D( IVPToSourcePosition( origin ) ), IVPToBox3D( m_NextRotation ) };
}

void CPhysicsObject::MoveTo( RealVec3_t origin, RealQuat_t rotation, bool isTeleport )
{
	const bool bCycleCollisions = isTeleport && m_bCollisionEnabled;
	if ( bCycleCollisions )
		EnableCollisions( false );

	const RealMatrix3_t matrix = QuatToMatrix( rotation );
	const RealVec3_t core = m_bNoShift ? origin : TransformShift( matrix, b3Neg( m_Shift ), origin );

	const float flElapsed = static_cast<float>( m_pEnvironment->GetCurrentTime() - m_flLastStepTime );
	m_flInverseStepInterval = flElapsed < m_pEnvironment->GetSimulationTimestep() ? m_pEnvironment->GetInverseTimestep() : 1.0f / flElapsed;
	m_LastRotation = rotation;
	m_NextRotation = rotation;
	m_flLastStepTime = m_pEnvironment->GetLastStepTime();
	m_CorePosition = core;
	m_CoreRotation = matrix;
	m_ImplicitVelocity = b3Vec3_zero;

	const b3Vec3 angular = GetCoreAngularVelocity();
	const b3Transform transform = GetBodyTransform();
	b3Body_SetTransform( m_Body, transform.p, transform.q );
	SetCoreVelocity( b3Vec3_zero );
	SetCoreAngularVelocity( angular );

	if ( bCycleCollisions )
		EnableCollisions( true );
}

void CPhysicsObject::StepUp( float height )
{
	const double flTime = m_pEnvironment->GetCurrentTime();
	const RealQuat_t rotation = GetRotationAt( flTime );
	const RealVec3_t position = GetCorePositionAt( flTime );
	RealVec3_t origin = m_bNoShift ? position : TransformShift( QuatToMatrix( rotation ), m_Shift, position );
	origin.y -= height * SOURCE_TO_IVP;
	MoveTo( origin, rotation, false );
}

void CPhysicsObject::RebuildCoreMatrix( double flTime )
{
	const real_t flElapsed = static_cast<float>( flTime - m_flLastStepTime );
	m_CorePosition = {
		m_CorePosition.x + flElapsed * real_t( m_ImplicitVelocity.x ),
		m_CorePosition.y + flElapsed * real_t( m_ImplicitVelocity.y ),
		m_CorePosition.z + flElapsed * real_t( m_ImplicitVelocity.z ),
	};
	m_LastRotation = m_NextRotation;
	m_CoreRotation = QuatToMatrix( m_LastRotation );
	m_flLastStepTime = flTime;
}

void CPhysicsObject::UpdateTransform( const b3Transform &transform )
{
	InvalidateVelocityCache();

	if ( m_flLastStepTime != m_pEnvironment->GetLastStepTime() )
		RebuildCoreMatrix( m_pEnvironment->GetLastStepTime() );

	const RealVec3_t next = ToReal( SourceToIVP( b3TransformPoint( transform, ToBox3D( m_MassCenter ) ) ) );
	const real_t flInverseTimestep = m_pEnvironment->GetInverseTimestep();
	m_ImplicitVelocity = ToFloat( flInverseTimestep * ( next - m_CorePosition ) );
	m_NextRotation = Box3DToIVP( transform.q );
	m_flInverseStepInterval = m_pEnvironment->GetInverseTimestep();
}

void CPhysicsObject::NotifySleep( const b3Transform &transform )
{
	InvalidateVelocityCache();
	m_nSleepState = m_nSleepState == AWAKE ? FELL_ASLEEP : ASLEEP;

	m_NextRotation = Box3DToIVP( transform.q );
	m_LastRotation = m_NextRotation;
	m_CorePosition = ToReal( SourceToIVP( b3TransformPoint( transform, ToBox3D( m_MassCenter ) ) ) );
	m_CoreRotation = QuatToMatrix( m_NextRotation );
	m_ImplicitVelocity = b3Vec3_zero;
	m_flLastStepTime = m_pEnvironment->GetLastStepTime();
}

CPhysicsObject::SleepState_t CPhysicsObject::NotifyWake()
{
	const SleepState_t nPrevious = m_nSleepState;
	m_nSleepState = AWAKE;
	m_bAsleepSinceCreation = false;
	return nPrevious;
}

void CPhysicsObject::ResetClock( double flLastStepTime )
{
	m_flLastStepTime -= flLastStepTime;
}

CPhysicsObject::~CPhysicsObject()
{
	RemoveShadowController();
	m_nCallbackFlags = 0;
	m_pGameData = nullptr;
	if ( m_pFluidController )
		static_cast<CPhysicsFluidController *>( m_pFluidController )->FluidObjectDestroyed();
	while ( !m_Constraints.empty() )
		m_Constraints.back()->ObjectDestroyed( this );
	while ( !m_Springs.empty() )
		m_Springs.back()->ObjectDestroyed( this );
	const std::vector<IController *> controllers = m_Controllers;
	for ( IController *pController : controllers )
		pController->ObjectDestroyed( this );
	if ( m_bTrigger )
		m_pEnvironment->RemoveTriggerOverlaps( this );
	ClearFilterDecisions();
	m_pEnvironment->RemoveWoken( this );
	m_pEnvironment->RemovePlayerGround( this );
	m_pEnvironment->RemoveObjectContacts( this );
	if ( m_bPendingVelocity )
		m_pEnvironment->RemovePendingVelocity( this );
	if ( m_bVelocityFlushQueued )
		m_pEnvironment->RemoveVelocityFlush( this );
	b3DestroyBody( m_Body );
}

bool CPhysicsObject::IsAsleep() const
{
	return m_nSleepState != AWAKE && !b3Body_IsAwake( m_Body );
}

void CPhysicsObject::EnableCollisions( bool enable )
{
	if ( enable )
	{
		m_nCallbackFlags |= CALLBACK_ENABLING_COLLISION;
		m_bCollisionEnabled = true;
		const b3Filter filter = GetCollisionFilter( true );
		ForEachShape( m_Body, [&]( b3ShapeId shape ) { b3Shape_SetFilter( shape, filter, true ); } );
		m_pEnvironment->ResolveFilterDecisions( this, true );
		m_nCallbackFlags &= ~CALLBACK_ENABLING_COLLISION;
		return;
	}

	if ( !m_bCollisionEnabled )
		return;

	m_pEnvironment->RemoveAllContacts( this );
	m_bCollisionEnabled = false;
	m_pEnvironment->PrepareShapeReset( this );
	const b3Filter filter = GetCollisionFilter( false );
	ForEachShape( m_Body, [&]( b3ShapeId shape ) { b3Shape_SetFilter( shape, filter, true ); } );
}

bool CPhysicsObject::IsGravityEnabled() const
{
	return !m_bStatic && HasController( m_pEnvironment->GetGravityController() );
}

bool CPhysicsObject::IsDragEnabled() const
{
	return !m_bStatic && HasController( m_pEnvironment->GetDragController() );
}

void CPhysicsObject::EnableGravity( bool enable )
{
	if ( m_bStatic || IsGravityEnabled() == enable )
		return;

	if ( enable )
		AttachController( m_pEnvironment->GetGravityController() );
	else
		DetachController( m_pEnvironment->GetGravityController() );
	b3Body_SetGravityScale( m_Body, enable ? 1.0f : 0.0f );
}

void CPhysicsObject::AttachController( IController *pController )
{
	m_Controllers.push_back( pController );
	const int nPriority = pController->GetPriority();
	auto position = std::upper_bound( m_ControllerOrder.begin(), m_ControllerOrder.end(), nPriority, []( int n, const IController *p ) { return n < p->GetPriority(); } );
	if ( position - m_ControllerOrder.begin() <= m_nControllerCursor )
		++m_nControllerCursor;
	m_ControllerOrder.insert( position, pController );
}

void CPhysicsObject::DetachController( IController *pController )
{
	auto it = std::find( m_Controllers.begin(), m_Controllers.end(), pController );
	if ( it == m_Controllers.end() )
		return;

	m_Controllers.erase( it );
	auto order = std::find( m_ControllerOrder.begin(), m_ControllerOrder.end(), pController );
	if ( order - m_ControllerOrder.begin() < m_nControllerCursor )
		--m_nControllerCursor;
	m_ControllerOrder.erase( order );
}

bool CPhysicsObject::HasController( const IController *pController ) const
{
	return std::find( m_Controllers.begin(), m_Controllers.end(), pController ) != m_Controllers.end();
}

void CPhysicsObject::AttachConstraint( CPhysicsConstraint *pConstraint )
{
	m_Constraints.push_back( pConstraint );
}

void CPhysicsObject::DetachConstraint( CPhysicsConstraint *pConstraint )
{
	m_Constraints.erase( std::remove( m_Constraints.begin(), m_Constraints.end(), pConstraint ), m_Constraints.end() );
}

void CPhysicsObject::AttachSpring( CPhysicsSpring *pSpring )
{
	m_Springs.push_back( pSpring );
}

void CPhysicsObject::DetachSpring( CPhysicsSpring *pSpring )
{
	m_Springs.erase( std::remove( m_Springs.begin(), m_Springs.end(), pSpring ), m_Springs.end() );
}

bool CPhysicsObject::IsAttachedToConstraint( bool bExternalOnly ) const
{
	for ( IController *pController : m_Controllers )
	{
		if ( pController->GetPriority() != CPhysicsConstraintGroup::PRIORITY )
			continue;

		if ( !bExternalOnly || static_cast<const CPhysicsConstraintGroup *>( pController )->HasExternalObject( this ) )
			return true;
	}
	return false;
}

void CPhysicsObject::RunControllers( float flTimestep, int nPriorityFloor )
{
	for ( ; m_nControllerCursor >= 0; --m_nControllerCursor )
	{
		if ( m_nControllerCursor >= static_cast<int>( m_ControllerOrder.size() ) )
			continue;

		IController *pController = m_ControllerOrder[m_nControllerCursor];
		if ( pController->GetPriority() <= nPriorityFloor )
			return;
		pController->Simulate( this, flTimestep );
	}
}

void CPhysicsObject::EnableDrag( bool enable )
{
	if ( m_bStatic || IsDragEnabled() == enable )
		return;

	if ( enable )
		AttachController( m_pEnvironment->GetDragController() );
	else
		DetachController( m_pEnvironment->GetDragController() );
}

void CPhysicsObject::EnableMotion( bool enable )
{
	if ( m_bStatic || m_bMotionEnabled == enable )
		return;

	m_bMotionEnabled = enable;
	SetCoreVelocity( b3Vec3_zero );
	SetCoreAngularVelocity( b3Vec3_zero );
	m_PendingVelocity = b3Vec3_zero;
	m_PendingAngularVelocity = b3Vec3_zero;
	b3Body_SetType( m_Body, enable ? b3_dynamicBody : b3_kinematicBody );
	if ( enable )
	{
		UpdateInverseMass();
	}
	else
	{
		m_flInvMass = 0.0f;
		m_InvInertia.Init();
	}
	UpdateMassData();

	if ( enable && IsHinged() )
		BecomeHinged( m_nHingedAxis );

	RecheckCollisionFilter();
	RecheckContactPoints();
}

void CPhysicsObject::Wake()
{
	b3Body_SetAwake( m_Body, true );
	m_pEnvironment->NotifyWoken( this );
}

void CPhysicsObject::Sleep()
{
	if ( m_bStatic )
		return;

	m_pEnvironment->RemoveWoken( this );
	b3Body_SetAwake( m_Body, false );
	m_pEnvironment->SyncSleepingObjects();
}

void CPhysicsObject::SetMass( float mass )
{
	const bool bWasMoveable = IsMoveable();
	if ( !bWasMoveable )
		EnableMotion( true );

	ChangeMass( std::min( std::max( 0.0f, mass ), VPHYSICS_MAX_MASS ) );
	SetVolume( m_flVolume );
	ComputeDragBases();

	if ( !bWasMoveable )
		EnableMotion( false );
}

void CPhysicsObject::ChangeMass( float mass )
{
	const float flRatio = mass / m_flMass;
	m_flMass = mass;
	m_Inertia *= flRatio;
	UpdateInverseMass();
	UpdateMassData();
}

void CPhysicsObject::SetInertia( const Vector &inertia )
{
	m_Inertia = Vector( std::fabs( inertia.x ), std::fabs( inertia.y ), std::fabs( inertia.z ) );
	UpdateInverseMass();
	UpdateMassData();
}

void CPhysicsObject::SetDamping( const float *speed, const float *rot )
{
	if ( speed )
		m_flSpeedDamping = *speed;
	if ( rot )
		m_flRotDamping = *rot;
}

void CPhysicsObject::GetDamping( float *speed, float *rot ) const
{
	if ( speed )
		*speed = m_flSpeedDamping;
	if ( rot )
		*rot = m_flRotDamping;
}

void CPhysicsObject::SetDragCoefficient( float *pDrag, float *pAngularDrag )
{
	if ( pDrag )
		m_flDragCoefficient = *pDrag;
	if ( pAngularDrag )
		m_flAngularDragCoefficient = *pAngularDrag;

	EnableDrag( m_flDragCoefficient != 0.0f || m_flAngularDragCoefficient != 0.0f );
}

void CPhysicsObject::SetMaterialIndex( int materialIndex )
{
	if ( m_nMaterialIndex == materialIndex )
		return;

	m_nMaterialIndex = static_cast<unsigned short>( materialIndex );
	m_nCallbackFlags |= CALLBACK_ENABLING_COLLISION;
	const b3SurfaceMaterial material = GetSurfaceMaterial( materialIndex );
	ForEachShape( m_Body, [&]( b3ShapeId shape ) { b3Shape_SetSurfaceMaterial( shape, material ); } );
	m_nCallbackFlags &= ~CALLBACK_ENABLING_COLLISION;

	if ( m_pShadow )
		m_pShadow->ObjectMaterialChanged( materialIndex );
}

float CPhysicsObject::GetEnergy() const
{
	const b3Vec3 v = GetCoreVelocity();
	const b3Vec3 w = GetCoreAngularVelocity();
	const b3Vec3 inertia = { m_Inertia.x, m_Inertia.z, m_Inertia.y };
	const float flRotational = ( ( w.y * inertia.y ) * w.y + ( w.x * inertia.x ) * w.x ) + ( w.z * inertia.z ) * w.z;
	const float flLinear = ( ( v.x * v.x + v.y * v.y ) + v.z * v.z ) * m_flMass;
	return ( flRotational + flLinear ) * ENERGY_SCALE;
}

float CPhysicsObject::GetSphereRadius() const
{
	return IVP_TO_SOURCE * m_flSphereRadius;
}

b3Vec3 CPhysicsObject::GetCoreVelocity() const
{
	if ( m_bCoreVelocityValid )
		return m_CoreVelocity;

	return SourceToIVP( b3Body_GetLinearVelocity( m_Body ) );
}

void CPhysicsObject::SetCoreVelocity( b3Vec3 velocity )
{
	m_CoreVelocity = velocity;
	m_bCoreVelocityValid = true;
	QueueVelocityFlush();
}

void CPhysicsObject::InvalidateVelocityCache()
{
	m_bCoreVelocityValid = false;
	m_bCoreAngularVelocityValid = false;
}

b3Vec3 CPhysicsObject::GetCoreAngularVelocity() const
{
	if ( m_bCoreAngularVelocityValid )
		return m_CoreAngularVelocity;

	const b3Vec3 local = b3InvRotateVector( b3Body_GetRotation( m_Body ), b3Body_GetAngularVelocity( m_Body ) );
	return { local.x, -local.z, local.y };
}

void CPhysicsObject::SetCoreAngularVelocity( b3Vec3 angular )
{
	m_CoreAngularVelocity = angular;
	m_bCoreAngularVelocityValid = true;
	QueueVelocityFlush();
}

b3Vec3 CPhysicsObject::GetCoreInverseInertia() const
{
	return { m_InvInertia.x, m_InvInertia.z, m_InvInertia.y };
}

b3Vec3 CPhysicsObject::IVPToCore( b3Vec3 worldIVP ) const
{
	return ToFloat( InverseRotate( m_CoreRotation, worldIVP ) );
}

b3Vec3 CPhysicsObject::CoreToIVP( b3Vec3 local ) const
{
	return ToFloat( Rotate( m_CoreRotation, local ) );
}

RealVec3_t CPhysicsObject::TransformCorePoint( b3Vec3 localPoint ) const
{
	return Rotate( m_CoreRotation, localPoint ) + m_CorePosition;
}

b3Vec3 CPhysicsObject::GetCoreSurfaceVelocity( b3Vec3 localPoint ) const
{
	return b3Add( CoreToIVP( b3Cross( GetCoreAngularVelocity(), localPoint ) ), GetCoreVelocity() );
}

b3Vec3 CPhysicsObject::GetCoreOffset( const Vector &worldPosition ) const
{
	const b3Vec3 position = SourceToIVP( worldPosition );
	return {
		static_cast<float>( real_t( position.x ) - m_CorePosition.x ),
		static_cast<float>( real_t( position.y ) - m_CorePosition.y ),
		static_cast<float>( real_t( position.z ) - m_CorePosition.z ),
	};
}

void CPhysicsObject::QueuePendingVelocity()
{
	if ( !m_bPendingVelocity )
	{
		m_bPendingVelocity = true;
		m_pEnvironment->QueuePendingVelocity( this );
	}
}

void CPhysicsObject::ApplyPendingVelocity()
{
	m_bPendingVelocity = false;
	SetCoreAngularVelocity( b3Add( GetCoreAngularVelocity(), m_PendingAngularVelocity ) );
	SetCoreVelocity( b3Add( GetCoreVelocity(), m_PendingVelocity ) );
	m_PendingVelocity = b3Vec3_zero;
	m_PendingAngularVelocity = b3Vec3_zero;
}

void CPhysicsObject::SetPosition( const Vector &worldPosition, const QAngle &angles, bool isTeleport )
{
	if ( m_pShadow )
		UpdateShadow( worldPosition, angles, false, 0.0f );

	MoveTo( ToReal( SourceToIVP( worldPosition ) ), AnglesToIVP( angles ), isTeleport );
}

void CPhysicsObject::SetPositionMatrix( const matrix3x4_t &matrix, bool isTeleport )
{
	if ( m_pShadow )
		UpdateShadow( Vector( matrix[0][3], matrix[1][3], matrix[2][3] ), MatrixAngles( matrix ), false, 0.0f );

	RealVec3_t origin;
	const RealQuat_t rotation = MatrixToIVP( matrix, origin );
	MoveTo( origin, rotation, isTeleport );
}

void CPhysicsObject::GetPosition( Vector *worldPosition, QAngle *angles ) const
{
	RealMatrix3_t rotation;
	RealVec3_t origin;
	GetPose( m_pEnvironment->GetCurrentTime(), rotation, origin );
	if ( worldPosition )
		*worldPosition = IVPToSourcePosition( origin );
	if ( angles )
		*angles = IVPToAngles( rotation );
}

void CPhysicsObject::GetPositionMatrix( matrix3x4_t *positionMatrix ) const
{
	RealMatrix3_t rotation;
	RealVec3_t origin;
	GetPose( m_pEnvironment->GetCurrentTime(), rotation, origin );
	IVPToMatrix( rotation, origin, *positionMatrix );
}

void CPhysicsObject::SetVelocity( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( !IsMoveable() )
		return;

	Wake();
	if ( velocity )
	{
		m_PendingVelocity = SourceToIVP( *velocity );
		SetCoreVelocity( b3Vec3_zero );
	}
	if ( angularVelocity )
	{
		m_PendingAngularVelocity = SourceToIVPAngular( *angularVelocity );
		SetCoreAngularVelocity( b3Vec3_zero );
	}
	QueuePendingVelocity();
	ClampVelocity( true );
}

void CPhysicsObject::SetVelocityInstantaneous( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( !IsMoveable() )
		return;

	Wake();
	if ( velocity )
	{
		SetCoreVelocity( SourceToIVP( *velocity ) );
		m_PendingVelocity = b3Vec3_zero;
	}
	if ( angularVelocity )
	{
		SetCoreAngularVelocity( SourceToIVPAngular( *angularVelocity ) );
		m_PendingAngularVelocity = b3Vec3_zero;
	}
	ClampVelocity( true );
}

void CPhysicsObject::AddVelocity( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( !IsMoveable() )
		return;

	Wake();
	if ( velocity )
		m_PendingVelocity = b3Add( m_PendingVelocity, SourceToIVP( *velocity ) );
	if ( angularVelocity )
		m_PendingAngularVelocity = b3Add( m_PendingAngularVelocity, SourceToIVPAngular( *angularVelocity ) );
	QueuePendingVelocity();
	ClampVelocity( true );
}

b3Vec3 CPhysicsObject::GetReportedVelocity() const
{
	if ( m_bReportPreStepVelocity )
		return SourceToIVP( m_nPreStepCount == m_pEnvironment->GetStepCount() ? m_PreStepVelocity : b3Vec3_zero );
	return GetCoreVelocity();
}

b3Vec3 CPhysicsObject::GetReportedAngularVelocity() const
{
	if ( m_bReportPreStepVelocity )
	{
		const b3Vec3 w = m_nPreStepCount == m_pEnvironment->GetStepCount() ? m_PreStepAngularVelocity : b3Vec3_zero;
		return IVPToCore( { w.x, -w.z, w.y } );
	}
	return GetCoreAngularVelocity();
}

void CPhysicsObject::GetVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const
{
	if ( velocity )
		*velocity = ToSource( IVPToSource( b3Add( GetReportedVelocity(), m_PendingVelocity ) ) );
	if ( angularVelocity )
		*angularVelocity = ToSource( IVPToSourceAngular( b3Add( GetReportedAngularVelocity(), m_PendingAngularVelocity ) ) );
}

void CPhysicsObject::GetVelocityAtPoint( const Vector &worldPosition, Vector *pVelocity ) const
{
	const b3Vec3 w = ToFloat( Rotate( m_CoreRotation, b3Add( GetReportedAngularVelocity(), m_PendingAngularVelocity ) ) );
	const b3Vec3 r = GetCoreOffset( worldPosition );
	const b3Vec3 speed = GetReportedVelocity();
	const b3Vec3 change = m_PendingVelocity;
	pVelocity->x = ( ( ( r.z * w.y - r.y * w.z ) + speed.x ) + change.x ) * IVP_TO_SOURCE;
	pVelocity->y = ( ( ( r.y * w.x - r.x * w.y ) + speed.z ) + change.z ) * IVP_TO_SOURCE;
	pVelocity->z = -( ( ( ( r.x * w.z - r.z * w.x ) + speed.y ) + change.y ) * IVP_TO_SOURCE );
}

void CPhysicsObject::GetImplicitVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const
{
	if ( velocity )
		*velocity = ToSource( IVPToSource( m_ImplicitVelocity ) );

	if ( angularVelocity )
	{
		const RealQuat_t delta = MultiplyQuat( ConjugateQuat( m_LastRotation ), m_NextRotation );
		float flAngle = static_cast<float>( 2.0 * std::acos( std::clamp( static_cast<double>( delta.w ), -1.0, 1.0 ) ) ) * RADIANS_TO_DEGREES;
		if ( flAngle > 180.0f )
			flAngle -= 360.0f;

		const b3Vec3 axis = ToFloat( { delta.x, delta.y, delta.z } );
		const float flInverseLength = FastInverseSquareRoot( b3LengthSquared( axis ) + AXIS_LENGTH_BIAS );
		flAngle *= m_flInverseStepInterval;
		*angularVelocity = Vector( axis.x * flInverseLength * flAngle, axis.z * flInverseLength * flAngle, -( axis.y * flInverseLength * flAngle ) );
	}
}

void CPhysicsObject::LocalToWorld( Vector *worldPosition, const Vector &localPosition ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	const Vector local = localPosition;
	MatrixTransformPoint( local, matrix, *worldPosition );
}

void CPhysicsObject::WorldToLocal( Vector *localPosition, const Vector &worldPosition ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	const Vector world = worldPosition;
	MatrixInverseTransformPoint( world, matrix, *localPosition );
}

void CPhysicsObject::LocalToWorldVector( Vector *worldVector, const Vector &localVector ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	*worldVector = MatrixRotateVector( matrix, localVector );
}

void CPhysicsObject::WorldToLocalVector( Vector *localVector, const Vector &worldVector ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	*localVector = MatrixInverseRotateVector( matrix, worldVector );
}

void CPhysicsObject::ApplyForceCenter( const Vector &forceVector )
{
	if ( !IsMoveable() )
		return;

	Wake();
	m_PendingVelocity = b3Add( m_PendingVelocity, b3MulSV( m_flInvMass, SourceToIVP( forceVector ) ) );
	QueuePendingVelocity();
	ClampVelocity( true );
}

void CPhysicsObject::ApplyForceOffset( const Vector &forceVector, const Vector &worldPosition )
{
	if ( !IsMoveable() )
		return;

	ApplyImpulse( ToReal( SourceToIVP( worldPosition ) ), SourceToIVP( forceVector ) );
	Wake();
	ClampVelocity( true );
}

void CPhysicsObject::ApplyTorqueCenter( const AngularImpulse &torque )
{
	if ( !IsMoveable() )
		return;

	m_PendingAngularVelocity = b3Add( m_PendingAngularVelocity, b3Mul( IVPToCore( SourceToIVPAngular( torque ) ), GetCoreInverseInertia() ) );
	Wake();
	QueuePendingVelocity();
	ClampVelocity( true );
}

void CPhysicsObject::CalculateForceOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerForce, AngularImpulse *centerTorque ) const
{
	const b3Vec3 force = SourceToIVP( forceVector );
	const b3Vec3 torque = IVPToCore( b3Cross( GetCoreOffset( worldPosition ), force ) );

	if ( centerForce )
		*centerForce = ToSource( IVPToSource( force ) );
	if ( centerTorque )
		*centerTorque = ToSource( IVPToSourceAngular( torque ) );
}

void CPhysicsObject::CalculateVelocityOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerVelocity, AngularImpulse *centerAngularVelocity ) const
{
	const b3Vec3 force = SourceToIVP( forceVector );
	const b3Vec3 torque = IVPToCore( b3Cross( GetCoreOffset( worldPosition ), force ) );

	if ( centerVelocity )
		*centerVelocity = ToSource( IVPToSource( b3MulSV( m_flInvMass, force ) ) );
	if ( centerAngularVelocity )
		*centerAngularVelocity = ToSource( IVPToSourceAngular( b3Mul( torque, GetCoreInverseInertia() ) ) );
}

float CPhysicsObject::GetLinearDrag( b3Vec3 coreDirection ) const
{
	return ( std::fabs( coreDirection.x * m_LinearDragBasis[0] ) * m_flDragCoefficient + std::fabs( coreDirection.y * m_LinearDragBasis[1] ) ) + std::fabs( coreDirection.z * m_LinearDragBasis[2] );
}

float CPhysicsObject::GetAngularDrag( b3Vec3 coreAxis ) const
{
	return ( std::fabs( m_AngularDragBasis[1] * coreAxis.y ) + std::fabs( m_AngularDragBasis[0] * coreAxis.x ) * m_flAngularDragCoefficient ) + std::fabs( m_AngularDragBasis[2] * coreAxis.z );
}

float CPhysicsObject::CalculateLinearDrag( const Vector &unitDirection ) const
{
	return GetLinearDrag( IVPToCore( { unitDirection.x, -unitDirection.z, unitDirection.y } ) );
}

float CPhysicsObject::CalculateAngularDrag( const Vector &objectSpaceRotationAxis ) const
{
	const Vector &axis = objectSpaceRotationAxis;
	return GetAngularDrag( { axis.x, axis.z, axis.y } ) * DEGREES_TO_RADIANS;
}

void CPhysicsObject::ClampVelocity( bool bExemptShadow )
{
	if ( bExemptShadow && m_pShadow )
		return;

	const float flMaxSpeed = m_pEnvironment->GetMaxSpeed();
	if ( flMaxSpeed > 0.0f )
	{
		SetCoreVelocity( ClampToLength( GetCoreVelocity(), flMaxSpeed ) );
		m_PendingVelocity = ClampToLength( m_PendingVelocity, flMaxSpeed );
	}

	const float flMaxAngularSpeed = m_pEnvironment->GetMaxRotationPerStep() * m_pEnvironment->GetInverseTimestep();
	if ( flMaxAngularSpeed > 0.0f )
	{
		SetCoreAngularVelocity( ClampToLength( GetCoreAngularVelocity(), flMaxAngularSpeed ) );
		const float flPendingSpeed = b3Length( m_PendingVelocity );
		if ( flPendingSpeed > flMaxAngularSpeed )
			m_PendingAngularVelocity = b3MulSV( flMaxAngularSpeed / flPendingSpeed, m_PendingAngularVelocity );
	}
}

Vector CPhysicsObject::GetInertia() const
{
	return Vector( std::fabs( m_Inertia.x ), std::fabs( m_Inertia.y ), std::fabs( m_Inertia.z ) );
}

Vector CPhysicsObject::GetInvInertia() const
{
	return Vector( std::fabs( m_InvInertia.x ), std::fabs( m_InvInertia.y ), std::fabs( m_InvInertia.z ) );
}

void CPhysicsObject::ComputeMassProperties( const objectparams_t &params, b3Vec3 massCenter, b3Vec3 unitInertia )
{
	m_flMass = std::min( std::max( VPHYSICS_MIN_MASS, params.mass ), VPHYSICS_MAX_MASS );

	float flFactor = params.inertia;
	if ( flFactor <= 0.0f )
		flFactor = 1.0f;
	else if ( flFactor >= MAX_INERTIA )
		flFactor = MAX_INERTIA;

	const real_t flShiftSquared = ( real_t( massCenter.x ) * massCenter.x + real_t( massCenter.y ) * massCenter.y ) + real_t( massCenter.z ) * massCenter.z;
	m_bNoShift = flShiftSquared < NO_SHIFT_DISTANCE_SQUARED;
	if ( m_bNoShift )
	{
		m_Shift = b3Vec3_zero;
		m_MassCenter = Vector( 0.0f, 0.0f, 0.0f );
	}
	else
	{
		m_Shift = b3Neg( massCenter );
		m_MassCenter = Vector( -( IVP_TO_SOURCE * m_Shift.x ), -( IVP_TO_SOURCE * m_Shift.z ), IVP_TO_SOURCE * m_Shift.y );
	}

	b3Vec3 inertia = { ( flFactor * unitInertia.x ) * m_flMass, ( unitInertia.y * flFactor ) * m_flMass, ( unitInertia.z * flFactor ) * m_flMass };
	if ( params.rotInertiaLimit != 0.0f )
	{
		const float flLengthSquared = ( inertia.x * inertia.x + inertia.y * inertia.y ) + inertia.z * inertia.z;
		const float flFloor = static_cast<float>( std::sqrt( real_t( flLengthSquared ) ) * params.rotInertiaLimit );
		inertia.x = flFloor > inertia.x ? flFloor : inertia.x;
		inertia.y = flFloor > inertia.y ? flFloor : inertia.y;
		inertia.z = flFloor > inertia.z ? flFloor : inertia.z;
	}

	m_Inertia = Vector( inertia.x, inertia.z, inertia.y );
	UpdateInverseMass();
}

void CPhysicsObject::ApplyImpulse( RealVec3_t worldPosition, b3Vec3 impulse )
{
	const b3Vec3 offset = ToFloat( worldPosition - m_CorePosition );
	m_PendingAngularVelocity = b3Add( m_PendingAngularVelocity, b3Mul( IVPToCore( b3Cross( offset, impulse ) ), GetCoreInverseInertia() ) );
	m_PendingVelocity = {
		static_cast<float>( real_t( impulse.x ) * real_t( m_flInvMass ) + real_t( m_PendingVelocity.x ) ),
		static_cast<float>( real_t( impulse.y ) * real_t( m_flInvMass ) + real_t( m_PendingVelocity.y ) ),
		static_cast<float>( real_t( impulse.z ) * real_t( m_flInvMass ) + real_t( m_PendingVelocity.z ) ),
	};
	QueuePendingVelocity();
}

void CPhysicsObject::AddPendingAngularVelocity( b3Vec3 change )
{
	m_PendingAngularVelocity = b3Add( m_PendingAngularVelocity, change );
	QueuePendingVelocity();
}

void CPhysicsObject::ApplyGravity( float flTimestep )
{
	if ( !m_bMotionEnabled )
		return;

	const real_t b = real_t( flTimestep ) * m_flSpeedDamping;
	const float a = flTimestep * m_flRotDamping;
	b3Vec3 angular = GetCoreAngularVelocity();
	if ( ( a * a + a * a ) + a * a < ANGULAR_DAMPING_EXPONENTIAL_LIMIT )
	{
		angular = b3MulSV( 1.0f - a, angular );
	}
	else
	{
		const real_t flExponential = std::exp( -real_t( a ) );
		angular = { std::exp( -a ) * angular.x, static_cast<float>( flExponential ) * angular.y, static_cast<float>( flExponential ) * angular.z };
	}

	const real_t flLinearFactor = b < real_t( LINEAR_DAMPING_EXPONENTIAL_LIMIT ) ? real_t( 1 ) - b : std::exp( -b );
	const b3Vec3 speed = GetCoreVelocity();
	b3Vec3 damped = { static_cast<float>( real_t( speed.x ) * flLinearFactor ), static_cast<float>( real_t( speed.y ) * flLinearFactor ), static_cast<float>( real_t( speed.z ) * flLinearFactor ) };
	SetCoreAngularVelocity( b3Add( angular, m_PendingAngularVelocity ) );
	damped = b3Add( damped, m_PendingVelocity );
	m_PendingVelocity = b3Vec3_zero;
	m_PendingAngularVelocity = b3Vec3_zero;

	const RealVec3_t gravity = m_pEnvironment->GetIVPGravity();
	damped = { flTimestep * static_cast<float>( gravity.x ) + damped.x, flTimestep * static_cast<float>( gravity.y ) + damped.y, flTimestep * static_cast<float>( gravity.z ) + damped.z };
	SetCoreVelocity( damped );
	m_bGravityApplied = true;
}

static b3Vec3 ScaleDrag( b3Vec3 v, float k )
{
#if defined( VPHYSICS_GMOD )
	const float x = v.x * k + v.x;
#else
	const float x = ( k + 1.0f ) * v.x;
#endif
	return { x, v.y * k + v.y, v.z * k + v.z };
}

void CPhysicsObject::ApplyDrag( float flTimestep, float flAirDensity )
{
	const b3Vec3 velocity = GetCoreVelocity();
	const float flLinearScale = ( flTimestep * flAirDensity ) * ( GetLinearDrag( IVPToCore( velocity ) ) * -0.5f );
	if ( !( flLinearScale >= 0.0f ) )
		SetCoreVelocity( ScaleDrag( velocity, flLinearScale > -1.0f ? flLinearScale : -1.0f ) );

	const b3Vec3 w = GetCoreAngularVelocity();
	const float flAngularScale = -( ( GetAngularDrag( w ) * flAirDensity ) * flTimestep );
	if ( !( flAngularScale >= 0.0f ) )
		SetCoreAngularVelocity( ScaleDrag( w, flAngularScale > -1.0f ? flAngularScale : -1.0f ) );
}

void CPhysicsObject::ClampStepVelocity()
{
	const float flInverseTimestep = m_pEnvironment->GetInverseTimestep();
	const float flRotationLimit = m_pEnvironment->GetMaxRotationPerStep();
	const float flMaxAngularSpeed = flInverseTimestep * flRotationLimit;
	const b3Vec3 angular = GetCoreAngularVelocity();
	const float flAngularSpeedSquared = b3LengthSquared( angular );
	if ( flMaxAngularSpeed * flMaxAngularSpeed < flAngularSpeedSquared )
		SetCoreAngularVelocity( b3MulSV( ( ( flInverseTimestep * flRotationLimit ) * STEP_ANGULAR_CLAMP_SCALE ) / std::sqrt( flAngularSpeedSquared ), angular ) );

	if ( m_pShadow )
		return;

	const float flMaxSpeed = m_pEnvironment->GetMaxSpeed();
	const b3Vec3 velocity = GetCoreVelocity();
	const float flSpeedSquared = b3LengthSquared( velocity );
	if ( flMaxSpeed * flMaxSpeed < flSpeedSquared )
		SetCoreVelocity( b3MulSV( ( flMaxSpeed * STEP_LINEAR_CLAMP_SCALE ) / std::sqrt( flSpeedSquared ), velocity ) );
}

void CPhysicsObject::SetVolume( float volume )
{
	m_flVolume = volume;
	if ( volume == 0.0f )
	{
		m_flBuoyancyRatio = 1.0f;
		return;
	}

	float flDensity, flThickness, flFriction, flElasticity;
	g_PhysicsSurfaceProps.GetPhysicsProperties( m_nMaterialIndex, &flDensity, &flThickness, &flFriction, &flElasticity );
	const float flCubicMetresPerCubicInch = ( SOURCE_TO_IVP * SOURCE_TO_IVP ) * SOURCE_TO_IVP;
	m_flBuoyancyRatio = ( GetMass() / ( flCubicMetresPerCubicInch * std::max( volume, MIN_BUOYANCY_VOLUME ) ) ) / flDensity;
}

void CPhysicsObject::ComputeDragBases()
{
	if ( m_bStatic || !m_pCollide )
		return;

	Vector mins, maxs;
	g_PhysicsCollision.CollideGetAABB( &mins, &maxs, m_pCollide, Vector( 0.0f, 0.0f, 0.0f ), QAngle( 0.0f, 0.0f, 0.0f ) );
	const Vector areas = g_PhysicsCollision.CollideGetOrthographicAreas( m_pCollide );

	const float dx = std::fabs( ( maxs.x - mins.x ) * SOURCE_TO_IVP );
	const float dy = std::fabs( ( maxs.y - mins.y ) * SOURCE_TO_IVP );
	const float dz = std::fabs( ( maxs.z - mins.z ) * SOURCE_TO_IVP );

	const float flInvMass = GetInvMass();
	m_LinearDragBasis[0] = flInvMass * ( ( dy * dz ) * areas.x );
	m_LinearDragBasis[1] = flInvMass * ( ( dx * dy ) * areas.y );
	m_LinearDragBasis[2] = flInvMass * ( ( dz * dx ) * areas.z );

	const float ex = dx * 0.5f, ey = dy * 0.5f, ez = dz * 0.5f;
	const float x2 = ( dx * dx ) * 0.25f, y2 = ( dy * dy ) * 0.25f, z2 = ( dz * dz ) * 0.25f;
	const float flThird = 0.33333334f;
	const b3Vec3 inverseInertia = GetCoreInverseInertia();

	const float p0y = ( ( ( ( y2 * flThird ) * ex ) * x2 ) + ( ( ( y2 * 0.5f ) * y2 ) * ex ) ) + ( ( ex * y2 ) * z2 );
	const float p0z = ( ( ( ( z2 * flThird ) * ex ) * x2 ) + ( ( ( z2 * 0.5f ) * z2 ) * ex ) ) + ( ( ex * z2 ) * y2 );
	const float p1z = ( ( ( ( x2 * flThird ) * ez ) * z2 ) + ( ( ( x2 * 0.5f ) * x2 ) * ez ) ) + ( ( ez * x2 ) * y2 );
	const float p1x = ( ( ( ( y2 * flThird ) * ez ) * z2 ) + ( ( ( y2 * 0.5f ) * y2 ) * ez ) ) + ( ( ez * y2 ) * x2 );
	const float p2y = ( ( ( ( x2 * flThird ) * ey ) * y2 ) + ( ( ( x2 * 0.5f ) * x2 ) * ey ) ) + ( ( x2 * ey ) * z2 );
	const float p2x = ( ( ( ( z2 * flThird ) * ey ) * y2 ) + ( ( ( z2 * 0.5f ) * z2 ) * ey ) ) + ( ( ey * z2 ) * x2 );

	m_AngularDragBasis[0] = ( ( p0y * inverseInertia.x ) * areas.y ) + ( ( p0z * inverseInertia.x ) * areas.z );
	m_AngularDragBasis[1] = ( ( p1z * inverseInertia.y ) * areas.z ) + ( ( p1x * inverseInertia.y ) * areas.x );
	m_AngularDragBasis[2] = ( ( p2y * inverseInertia.z ) * areas.y ) + ( ( p2x * inverseInertia.z ) * areas.x );
}

void CPhysicsObject::UpdateInverseMass()
{
	for ( int i = 0; i < 3; ++i )
	{
		float &flInertia = m_Inertia[i];
		if ( !std::isfinite( flInertia ) || flInertia > MAX_INERTIA )
			flInertia = MAX_INERTIA;
		else if ( flInertia < -MAX_INERTIA )
			flInertia = -MAX_INERTIA;
		m_InvInertia[i] = 1.0f / flInertia;
	}
	m_flInvMass = 1.0f / m_flMass;
}

void CPhysicsObject::UpdateMassData()
{
	if ( m_bStatic || !m_bMotionEnabled )
	{
		b3Body_ApplyMassFromShapes( m_Body );
		return;
	}

	b3MassData massData;
	massData.mass = 1.0f / m_flInvMass;
	massData.center = ToBox3D( m_MassCenter );
	massData.inertia = b3Mat3_zero;
	massData.inertia.cx.x = INERTIA_SCALE / m_InvInertia.x;
	massData.inertia.cy.y = INERTIA_SCALE / m_InvInertia.y;
	massData.inertia.cz.z = INERTIA_SCALE / m_InvInertia.z;
	b3Body_SetMassData( m_Body, massData );
}

void CPhysicsObject::BecomeHinged( int localAxis )
{
	if ( IsMoveable() )
	{
		const float flMass = GetMass();
		const Vector savedInverseInertia = m_InvInertia;
		SetMass( VPHYSICS_MAX_MASS );
		const Vector heavyInverseInertia = m_InvInertia;
		const float flHeavyInverseMass = m_flInvMass;
		SetMass( flMass );

		const int nAxis = localAxis >= 0 && localAxis < 3 ? localAxis : 0;
		m_InvInertia = heavyInverseInertia;
		m_InvInertia[nAxis] = savedInverseInertia[nAxis];
		m_flInvMass = flHeavyInverseMass;
		UpdateMassData();
	}

	m_nHingedAxis = localAxis;
}

void CPhysicsObject::RemoveHinged()
{
	m_nHingedAxis = -1;
	UpdateInverseMass();
	UpdateMassData();
}

void CPhysicsObject::OutputDebugInfo() const
{
	Msg( "-----------------\nObject: %s\n", GetName() );
	Msg( "Mass: %.1f (inv %.3f)\n", GetMass(), GetInvMass() );
	const Vector inertia = GetInertia(), inverseInertia = GetInvInertia();
	Msg( "Inertia: %.2f, %.2f, %.2f (inv %.3f, %.3f, %.3f)\n", inertia.x, inertia.y, inertia.z, inverseInertia.x, inverseInertia.y, inverseInertia.z );
	Vector velocity;
	AngularImpulse angular;
	GetVelocity( &velocity, &angular );
	Msg( "Velocity: %.2f, %.2f, %.2f \n", velocity.x, velocity.y, velocity.z );
	Msg( "Ang Velocity: %.2f, %.2f, %.2f \n", angular.x, angular.y, angular.z );
	Msg( "Damping %.2f linear, %.2f angular\n", m_flSpeedDamping, m_flRotDamping );
	Msg( "Linear Drag: %.2f, %.2f, %.2f (factor %.2f)\n", m_LinearDragBasis[0], m_LinearDragBasis[1], m_LinearDragBasis[2], m_flDragCoefficient );
	Msg( "Angular Drag: %.2f, %.2f, %.2f (factor %.2f)\n", m_AngularDragBasis[0], m_AngularDragBasis[1], m_AngularDragBasis[2], m_flAngularDragCoefficient );
	if ( IsHinged() )
	{
		static const char *const s_AxisNames[] = { "x", "y", "z" };
		Msg( "Hinged on %s axis\n", m_nHingedAxis >= 0 && m_nHingedAxis < 3 ? s_AxisNames[m_nHingedAxis] : "" );
	}
	Msg( "attached to %d controllers\n", static_cast<int>( m_Controllers.size() ) );
	for ( int i = static_cast<int>( m_Controllers.size() ) - 1; i >= 0; --i )
		Msg( "%d) %s\n", i, m_Controllers[i]->GetName() );
	Msg( "State: %s, Collision %s, Motion %s, %sFlags %04X (game %04x, index %d)\n", IsAsleep() ? "Asleep" : "Awake", IsCollisionEnabled() ? "Enabled" : "Disabled", IsStatic() ? "Static" : ( IsMotionEnabled() ? "Enabled" : "Disabled" ), ( m_nCallbackFlags & CALLBACK_MARKED_FOR_TEST ) ? "Debug! " : "", m_nCallbackFlags, m_nGameFlags, m_nGameIndex );

	float flDensity, flThickness, flFriction, flElasticity;
	g_PhysicsSurfaceProps.GetPhysicsProperties( m_nMaterialIndex, &flDensity, &flThickness, &flFriction, &flElasticity );
	Msg( "Material: %s : density(%.1f), thickness(%.2f), friction(%.2f), elasticity(%.2f)\n", g_PhysicsSurfaceProps.GetPropName( m_nMaterialIndex ), flDensity, flThickness, flFriction, flElasticity );

	if ( m_pCollide )
		g_PhysicsCollision.OutputDebugInfo( m_pCollide );
}

void CPhysicsObject::SetShadow( float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation )
{
	if ( m_pShadow )
	{
		m_pShadow->MaxSpeed( maxSpeed, maxAngularSpeed );
		return;
	}

	m_bShadowTempGravityDisable = false;
	m_pShadow = m_pEnvironment->CreateShadowController( this, allowPhysicsMovement, allowPhysicsRotation );
	m_pShadow->MaxSpeed( maxSpeed, maxAngularSpeed );
	RecheckCollisionFilter();
}

void CPhysicsObject::UpdateShadow( const Vector &targetPosition, const QAngle &targetAngles, bool tempDisableGravity, float timeOffset )
{
	if ( tempDisableGravity != m_bShadowTempGravityDisable )
	{
		m_bShadowTempGravityDisable = tempDisableGravity;
		if ( !m_pShadow || m_pShadow->AllowsTranslation() )
			EnableGravity( !m_bShadowTempGravityDisable );
	}

	if ( m_pShadow )
		m_pShadow->Update( targetPosition, targetAngles, timeOffset );
}

int CPhysicsObject::GetShadowPosition( Vector *position, QAngle *angles ) const
{
	RealMatrix3_t rotation;
	RealVec3_t origin;
	GetPose( m_pEnvironment->GetNextStepTime(), rotation, origin );
	if ( position )
		*position = IVPToSourcePosition( origin );
	if ( angles )
		*angles = IVPToAngles( rotation );
	return 1;
}

void CPhysicsObject::RemoveShadowController()
{
	if ( !m_pShadow )
		return;

	m_pEnvironment->DestroyShadowController( m_pShadow );
	m_pShadow = nullptr;
}

RealVec3_t CPhysicsObject::GetCoreOrigin() const
{
	return m_bNoShift ? m_CorePosition : TransformShift( m_CoreRotation, m_Shift, m_CorePosition );
}

void CPhysicsObject::ApplyShadowControl( ShadowControl_t &control, float flSecondsToArrival, float flTimestep )
{
	float flFraction = 1.0f;
	if ( flSecondsToArrival > 0.0f )
		flFraction = std::min( flTimestep / flSecondsToArrival, 1.0f );
	if ( flFraction <= 0.0f )
		return;

	const RealVec3_t origin = GetCoreOrigin();
	b3Vec3 delta = ToFloat( control.m_TargetPosition - origin );

	if ( control.m_flTeleportDistance > 0.0f )
	{
		const bool bNoPrediction = control.m_LastPosition.x == 0 && control.m_LastPosition.y == 0 && control.m_LastPosition.z == 0;
		const float flDistanceSquared = bNoPrediction ? b3LengthSquared( delta ) : b3LengthSquared( ToFloat( origin - control.m_LastPosition ) );
		if ( flDistanceSquared > control.m_flTeleportDistance * control.m_flTeleportDistance )
		{
			MoveTo( control.m_TargetPosition, control.m_TargetRotation, true );
			delta = b3Vec3_zero;
		}
	}

	const float flScale = ( 1.0f / flTimestep ) * flFraction;
	const b3Vec3 velocity = ComputeControlVelocity( GetCoreVelocity(), delta, control.m_flMaxSpeed, control.m_flMaxDampSpeed, flScale, control.m_flDampFactor, &control.m_LastImpulse );
	SetCoreVelocity( velocity );
	control.m_LastPosition = { origin.x + real_t( flTimestep * velocity.x ), origin.y + real_t( flTimestep * velocity.y ), origin.z + real_t( flTimestep * velocity.z ) };

	const b3Vec3 rotationDelta = GetRotationDelta( m_NextRotation, control.m_TargetRotation );
	SetCoreAngularVelocity( ComputeControlVelocity( GetCoreAngularVelocity(), rotationDelta, control.m_flMaxAngular, control.m_flMaxDampAngular, flScale, control.m_flDampFactor, nullptr ) );
}

float CPhysicsObject::ComputeShadowControl( const hlshadowcontrol_params_t &params, float secondsToArrival, float dt )
{
	ShadowControl_t control;
	control.m_TargetPosition = ToReal( SourceToIVP( params.targetPosition ) );
	control.m_TargetRotation = AnglesToIVP( params.targetRotation );
	control.m_flMaxSpeed = params.maxSpeed * SOURCE_TO_IVP;
	control.m_flMaxDampSpeed = params.maxDampSpeed * SOURCE_TO_IVP;
	control.m_flMaxAngular = params.maxAngular * DEGREES_TO_RADIANS;
	control.m_flMaxDampAngular = params.maxDampAngular * DEGREES_TO_RADIANS;
	control.m_flDampFactor = params.dampFactor;
	control.m_flTeleportDistance = params.teleportDistance * SOURCE_TO_IVP;
	control.m_LastPosition = {};
	control.m_LastImpulse = b3Vec3_zero;
	ApplyShadowControl( control, secondsToArrival, dt );
	return std::max( secondsToArrival - dt, 0.0f );
}

void CPhysicsObject::RecheckCollisionFilter()
{
	m_pEnvironment->RecheckCollisionFilter( this );
}

void CPhysicsObject::RecheckContactPoints()
{
	m_pEnvironment->RecheckContactPoints( this );
}

bool CPhysicsObject::GetContactPoint( Vector *contactPoint, IPhysicsObject **contactObject ) const
{
	return m_pEnvironment->GetContactPoint( this, contactPoint, contactObject );
}

IPhysicsFrictionSnapshot *CPhysicsObject::CreateFrictionSnapshot()
{
	return m_pEnvironment->CreateFrictionSnapshot( this );
}

void CPhysicsObject::DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot )
{
	delete pSnapshot;
}

void CPhysicsObject::BecomeTrigger()
{
	if ( m_bTrigger )
		return;

	if ( m_pShadow )
		m_pShadow->UseShadowMaterial( false );

	EnableDrag( false );
	EnableGravity( false );
	const bool bCollisionEnabled = IsCollisionEnabled();
	EnableCollisions( false );

	DestroyShapes();
	m_bTrigger = true;
	CreateShapes( true );
	UpdateMassData();

	EnableCollisions( bCollisionEnabled );
}

void CPhysicsObject::RemoveTrigger()
{
	if ( !m_bTrigger )
		return;

	m_pEnvironment->RemoveTriggerOverlaps( this );
	DestroyShapes();
	m_bTrigger = false;
	CreateShapes( false );
	UpdateMassData();
}
