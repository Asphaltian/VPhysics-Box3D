#include "physics_environment.h"

#include "conversion.h"
#include "physics_collide.h"
#include "physics_constraint.h"
#include "physics_fluid_controller.h"
#include "physics_friction_snapshot.h"
#include "physics_motion_controller.h"
#include "physics_object.h"
#include "physics_player_controller.h"
#include "physics_shadow_controller.h"
#include "physics_spring.h"
#include "physics_vehicle_controller.h"
#include "physics_surface_props.h"

#include "tier0/dbg.h"
#include "vphysics/constraints.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <functional>
#include <limits>

static constexpr float DEFAULT_TIMESTEP = 1.0f / 66.0f;
static constexpr float IVP_DEFAULT_GRAVITY = 9.83f;
static constexpr float MAX_SIMULATE_DELTA = 1.0f;
static constexpr double MIN_SIMULATE_DELTA = 1e-4;
static constexpr double MAX_STEP_DELTA = 0.1;
static constexpr float ALIGNED_STEP_SCALE = 1.99999f;
static constexpr float COLLISION_TOLERANCE = ( 0.25f - 0.0001f ) * SOURCE_TO_IVP;
static constexpr float ISOTROPIC_INVERSE_INERTIA_FRACTION = 0.01f;
static constexpr float MIN_FRICTION_MASS = 1.0f;
static constexpr float MAX_FRICTION_MASS = 50000.0f;
static constexpr float PENETRATION_SPEED_SCALE = 4.0f;
static constexpr double UNKNOWN_IMPACT_TIME = -1000.0;
static constexpr float MAX_IMPACT_INTERVAL = 999.0f;
static constexpr float DEFAULT_IMPACT_INTERVAL = 1.0f;
static constexpr double PAIR_TIME_LIFETIME = 1.0;
static constexpr size_t MAX_CACHED_PAIR_TIMES = 16;
static constexpr float FRICTION_ENERGY_THRESHOLD = 0.05f;
static constexpr float IVP_ENERGY_TO_SOURCE = 1550.0032f;
static constexpr float SURFACE_NORMAL_BIAS = 1e-10f;
static constexpr float IMPACT_SEPARATION = COLLISION_TOLERANCE * IVP_TO_SOURCE;
static constexpr float PENETRATION_SEPARATION = -( COLLISION_TOLERANCE * IVP_TO_SOURCE );
static constexpr float FRICTION_RELEASE_SEPARATION = ( 4.5f - 2.0f ) * COLLISION_TOLERANCE * IVP_TO_SOURCE;
static constexpr uint32_t SLEEP_RANDOM_MULTIPLIER = 75;
static constexpr float SLEEP_RANDOM_SCALE = 1.5258789e-5f;
static constexpr short SLEEP_COUNTDOWN_BASE = 15;
static constexpr float SLEEP_COUNTDOWN_RANGE = 5.0f;

class CNullDebugOverlay final : public IVPhysicsDebugOverlay
{
public:
	void AddEntityTextOverlay( int, int, float, int, int, int, int, const char *, ... ) override {}

	void AddBoxOverlay( const Vector &, const Vector &, const Vector &, QAngle const &, int, int, int, int, float ) override {}

	void AddTriangleOverlay( const Vector &, const Vector &, const Vector &, int, int, int, int, bool, float ) override {}

	void AddLineOverlay( const Vector &, const Vector &, int, int, int, bool, float ) override {}

	void AddTextOverlay( const Vector &, float, const char *, ... ) override {}

	void AddTextOverlay( const Vector &, int, float, const char *, ... ) override {}

	void AddScreenTextOverlay( float, float, float, int, int, int, int, const char * ) override {}

	void AddSweptBoxOverlay( const Vector &, const Vector &, const Vector &, const Vector &, const QAngle &, int, int, int, int, float ) override {}

	void AddTextOverlayRGB( const Vector &, int, float, float, float, float, float, const char *, ... ) override {}
};

static CNullDebugOverlay s_NullDebugOverlay;

static uint64_t GetPairKey( uint32_t nId0, uint32_t nId1 )
{
	return nId0 < nId1 ? ( static_cast<uint64_t>( nId0 ) << 32 ) | nId1 : ( static_cast<uint64_t>( nId1 ) << 32 ) | nId0;
}

static uint64_t GetPairKey( const CPhysicsObject *pObject0, const CPhysicsObject *pObject1 )
{
	return GetPairKey( pObject0->GetId(), pObject1->GetId() );
}

static bool IsPairMember( uint64_t nPairKey, const CPhysicsObject *pObject )
{
	return static_cast<uint32_t>( nPairKey >> 32 ) == pObject->GetId() || static_cast<uint32_t>( nPairKey ) == pObject->GetId();
}

static uint64_t GetTriggerKey( const CPhysicsObject *pTrigger, const CPhysicsObject *pObject )
{
	return ( static_cast<uint64_t>( pTrigger->GetId() ) << 32 ) | pObject->GetId();
}

static uint64_t GetContactKey( b3ContactId contactId )
{
	return ( static_cast<uint64_t>( static_cast<uint32_t>( contactId.index1 ) ) << 32 ) | contactId.generation;
}

static uint64_t GetShapePairKey( b3ShapeId shape0, b3ShapeId shape1 )
{
	const uint32_t n0 = static_cast<uint32_t>( shape0.index1 ), n1 = static_cast<uint32_t>( shape1.index1 );
	return n0 < n1 ? ( static_cast<uint64_t>( n0 ) << 32 ) | n1 : ( static_cast<uint64_t>( n1 ) << 32 ) | n0;
}

static bool IsSameShape( b3ShapeId a, b3ShapeId b )
{
	return a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation;
}

static CPhysicsObject *GetShapeObject( b3ShapeId shape )
{
	return static_cast<CPhysicsObject *>( b3Body_GetUserData( b3Shape_GetBody( shape ) ) );
}

static bool IsBall( const CPhysicsObject *pObject )
{
	return !pObject->GetCollide();
}

static int GetFeatureVertexOwner( uint32_t nFeatureId )
{
	const uint32_t nOwner1 = ( nFeatureId >> 24 ) & 0xFF, nOwner2 = ( nFeatureId >> 8 ) & 0xFF;
	return nOwner1 == nOwner2 ? static_cast<int>( nOwner1 ) : -1;
}

static int GetInitialFirstSide( const CPhysicsObject *pObject0, const CPhysicsObject *pObject1 )
{
	const bool bBall0 = IsBall( pObject0 ), bBall1 = IsBall( pObject1 );
	if ( bBall0 != bBall1 )
		return bBall0 ? 0 : 1;
	if ( bBall0 )
		return std::greater<const CPhysicsObject *>()( pObject0, pObject1 ) ? 0 : 1;
	if ( pObject0->IsStatic() != pObject1->IsStatic() )
		return pObject0->IsStatic() ? 1 : 0;
	return 0;
}

static bool IsPendingDeletePair( const CPhysicsObject *pObject0, const CPhysicsObject *pObject1 )
{
	const unsigned short nFlags0 = pObject0->GetCallbackFlags(), nFlags1 = pObject1->GetCallbackFlags();
	return ( ( nFlags0 & CALLBACK_ENABLING_COLLISION ) && ( nFlags1 & CALLBACK_MARKED_FOR_DELETE ) ) || ( ( nFlags1 & CALLBACK_ENABLING_COLLISION ) && ( nFlags0 & CALLBACK_MARKED_FOR_DELETE ) );
}

static Vector NormalizeSurfaceNormal( const Vector &n )
{
	const float flLengthSquared = ( n.y * n.y + n.x * n.x ) + n.z * n.z + SURFACE_NORMAL_BIAS;
	return n * FastInverseSquareRoot( flLengthSquared );
}

static float MixFriction( float flFrictionA, uint64_t, float flFrictionB, uint64_t )
{
	return std::clamp( flFrictionA * flFrictionB, 0.0f, 1.0f );
}

static float MixRestitution( float flRestitutionA, uint64_t, float flRestitutionB, uint64_t )
{
	return std::sqrt( std::clamp( flRestitutionA * flRestitutionB, 0.0f, 1.0f ) );
}

static float GetMaterialFriction( int nMaterial )
{
	float flDensity, flThickness, flFriction, flElasticity;
	g_PhysicsSurfaceProps.GetPhysicsProperties( nMaterial, &flDensity, &flThickness, &flFriction, &flElasticity );
	return flFriction;
}

static bool FilterShapes( b3ShapeId shapeA, b3ShapeId shapeB, void *pContext )
{
	return static_cast<CPhysicsEnvironment *>( pContext )->IsCollisionAllowed( GetShapeObject( shapeA ), GetShapeObject( shapeB ) );
}

static bool PreSolveShapes( b3ShapeId shapeA, b3ShapeId shapeB, b3Pos, b3Vec3, void *pContext )
{
	return static_cast<CPhysicsEnvironment *>( pContext )->IsContactEnabled( GetShapeObject( shapeA ), GetShapeObject( shapeB ) );
}

static int GetPhysicalProcessorCount()
{
	return std::max( 1, static_cast<int>( GetCPUInformation()->m_nPhysicalProcessors ) );
}

CPhysicsEnvironment::CPhysicsEnvironment()
: m_IVPGravity{ 0.0f, IVP_DEFAULT_GRAVITY, 0.0f }
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ToBox3D( GetGravityVector() );
	worldDef.enableSleep = false;
	worldDef.workerCount = GetPhysicalProcessorCount();
	worldDef.isotropicInertiaTolerance = std::sqrt( 2.0f * ISOTROPIC_INVERSE_INERTIA_FRACTION / ( 3.0f - ISOTROPIC_INVERSE_INERTIA_FRACTION ) );
	worldDef.hitEventThreshold = FLT_MAX;
	worldDef.frictionCallback = MixFriction;
	worldDef.restitutionCallback = MixRestitution;
	m_World = b3CreateWorld( &worldDef );
	b3World_SetCustomFilterCallback( m_World, FilterShapes, this );
	b3World_SetPreSolveCallback( m_World, PreSolveShapes, this );

	SetSimulationTimestep( DEFAULT_TIMESTEP );
	m_flNextStepTime = m_flTimestep;

	physics_performanceparams_t settings;
	settings.Defaults();
	SetPerformanceSettings( &settings );
}

CPhysicsEnvironment::~CPhysicsEnvironment()
{
	m_pCollisionSolver = nullptr;
	m_pObjectEvents = nullptr;
	m_pCollisionEvents = nullptr;
	m_pConstraintEvents = nullptr;

	std::vector<IPhysicsObject *> objects;
	objects.swap( m_Objects );
	for ( int i = static_cast<int>( objects.size() ) - 1; i >= 0; --i )
		delete static_cast<CPhysicsObject *>( objects[i] );

	CleanupDeleteList();

	for ( CPhysicsFluidController *pController : m_FluidControllers )
		delete pController;
	m_FluidControllers.clear();

	b3DestroyWorld( m_World );
}

void CPhysicsEnvironment::SetDebugOverlay( CreateInterfaceFn debugOverlayFactory )
{
	m_pDebugOverlay = nullptr;
	if ( debugOverlayFactory )
		m_pDebugOverlay = static_cast<IVPhysicsDebugOverlay *>( debugOverlayFactory( VPHYSICS_DEBUG_OVERLAY_INTERFACE_VERSION, nullptr ) );

	if ( !m_pDebugOverlay )
		m_pDebugOverlay = &s_NullDebugOverlay;
}

Vector CPhysicsEnvironment::GetGravityVector() const
{
	return Vector( static_cast<float>( m_IVPGravity.x * IVP_TO_SOURCE ), static_cast<float>( m_IVPGravity.z * IVP_TO_SOURCE ), -static_cast<float>( m_IVPGravity.y * IVP_TO_SOURCE ) );
}

void CPhysicsEnvironment::SetGravity( const Vector &gravityVector )
{
	m_IVPGravity = ToReal( SourceToIVP( gravityVector ) );
	b3World_SetGravity( m_World, ToBox3D( GetGravityVector() ) );
	UpdateContactTuning();
	DevMsg( 1, "Set Gravity %.1f (%.3f tolerance)\n", gravityVector.Length(), COLLISION_TOLERANCE * IVP_TO_SOURCE );
}

void CPhysicsEnvironment::UpdateContactTuning()
{
	const b3WorldDef defaults = b3DefaultWorldDef();
	const float flPushSpeed = PENETRATION_SPEED_SCALE * GetGravityVector().Length() * m_flTimestep;
	b3World_SetContactTuning( m_World, defaults.contactHertz, defaults.contactDampingRatio, flPushSpeed );
}

void CPhysicsEnvironment::GetGravity( Vector *pGravityVector ) const
{
	*pGravityVector = GetGravityVector();
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObject( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams )
{
	CPhysicsObject *pObject = CPhysicsObject::CreatePolyObject( this, pCollisionModel, materialIndex, position, angles, pParams, false );
	if ( pObject )
		m_Objects.push_back( pObject );
	return pObject;
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObjectStatic( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams )
{
	CPhysicsObject *pObject = CPhysicsObject::CreatePolyObject( this, pCollisionModel, materialIndex, position, angles, pParams, true );
	if ( pObject )
		m_Objects.push_back( pObject );
	return pObject;
}

IPhysicsObject *CPhysicsEnvironment::CreateSphereObject( float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic )
{
	CPhysicsObject *pObject = CPhysicsObject::CreateSphereObject( this, radius, materialIndex, position, angles, pParams, isStatic );
	m_Objects.push_back( pObject );
	return pObject;
}

void CPhysicsEnvironment::DestroyObject( IPhysicsObject *pObject )
{
	if ( !pObject )
	{
		DevMsg( "Deleted NULL vphysics object\n" );
		return;
	}

	auto it = std::find( m_Objects.rbegin(), m_Objects.rend(), pObject );
	if ( it == m_Objects.rend() )
	{
		DevMsg( 1, "error deleting physics object\n" );
		return;
	}

	*it = m_Objects.back();
	m_Objects.pop_back();

	CPhysicsObject *pPhysicsObject = static_cast<CPhysicsObject *>( pObject );
	pPhysicsObject->SetCallbackFlags( pPhysicsObject->GetCallbackFlags() | CALLBACK_MARKED_FOR_DELETE );
	ForgetRestoredObject( pPhysicsObject );

	if ( m_bInSimulation || m_bDispatchingFriction || m_bDeleteQueue )
	{
		m_DeadObjects.push_back( pPhysicsObject );
		return;
	}

	RemoveActiveObject( pPhysicsObject );
	delete pPhysicsObject;
}

IPhysicsFluidController *CPhysicsEnvironment::CreateFluidController( IPhysicsObject *pFluidObject, fluidparams_t *pParams )
{
	CPhysicsObject *pObject = static_cast<CPhysicsObject *>( pFluidObject );
	pObject->BecomeTrigger();

	CPhysicsFluidController *pController = new CPhysicsFluidController( pObject, *pParams );
	pObject->SetFluidController( pController );
	m_FluidControllers.push_back( pController );

	for ( const auto &[nKey, overlap] : m_TriggerOverlaps )
	{
		if ( overlap.m_pTrigger == pObject && overlap.m_bReported )
			pController->ObjectEntered( overlap.m_pObject );
	}
	return pController;
}

void CPhysicsEnvironment::DestroyFluidController( IPhysicsFluidController *pController )
{
	CPhysicsFluidController *pFluidController = static_cast<CPhysicsFluidController *>( pController );
	auto it = std::find( m_FluidControllers.begin(), m_FluidControllers.end(), pFluidController );
	if ( it != m_FluidControllers.end() )
		m_FluidControllers.erase( it );
	delete pFluidController;
}

IPhysicsSpring *CPhysicsEnvironment::CreateSpring( IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams )
{
	if ( !pObjectStart || !pObjectEnd || !pParams )
		return nullptr;

	return new CPhysicsSpring( this, static_cast<CPhysicsObject *>( pObjectStart ), static_cast<CPhysicsObject *>( pObjectEnd ), *pParams );
}

void CPhysicsEnvironment::DestroySpring( IPhysicsSpring *pSpring )
{
	delete static_cast<CPhysicsSpring *>( pSpring );
}

IPhysicsConstraint *CPhysicsEnvironment::CreateRagdollConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	return CPhysicsConstraint::CreateRagdoll( this, static_cast<CPhysicsObject *>( pReferenceObject ), static_cast<CPhysicsObject *>( pAttachedObject ), static_cast<CPhysicsConstraintGroup *>( pGroup ), ragdoll );
}

IPhysicsConstraint *CPhysicsEnvironment::CreateHingeConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge )
{
	return CPhysicsConstraint::CreateHinge( this, static_cast<CPhysicsObject *>( pReferenceObject ), static_cast<CPhysicsObject *>( pAttachedObject ), static_cast<CPhysicsConstraintGroup *>( pGroup ), constraint_limitedhingeparams_t( hinge ) );
}

IPhysicsConstraint *CPhysicsEnvironment::CreateFixedConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed )
{
	return CPhysicsConstraint::CreateFixed( this, static_cast<CPhysicsObject *>( pReferenceObject ), static_cast<CPhysicsObject *>( pAttachedObject ), static_cast<CPhysicsConstraintGroup *>( pGroup ), fixed );
}

IPhysicsConstraint *CPhysicsEnvironment::CreateSlidingConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding )
{
	return CPhysicsConstraint::CreateSliding( this, static_cast<CPhysicsObject *>( pReferenceObject ), static_cast<CPhysicsObject *>( pAttachedObject ), static_cast<CPhysicsConstraintGroup *>( pGroup ), sliding );
}

IPhysicsConstraint *CPhysicsEnvironment::CreateBallsocketConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket )
{
	return CPhysicsConstraint::CreateBallsocket( this, static_cast<CPhysicsObject *>( pReferenceObject ), static_cast<CPhysicsObject *>( pAttachedObject ), static_cast<CPhysicsConstraintGroup *>( pGroup ), ballsocket );
}

IPhysicsConstraint *CPhysicsEnvironment::CreatePulleyConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley )
{
	return CPhysicsConstraint::CreatePulley( this, static_cast<CPhysicsObject *>( pReferenceObject ), static_cast<CPhysicsObject *>( pAttachedObject ), static_cast<CPhysicsConstraintGroup *>( pGroup ), pulley );
}

IPhysicsConstraint *CPhysicsEnvironment::CreateLengthConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length )
{
	return CPhysicsConstraint::CreateLength( this, static_cast<CPhysicsObject *>( pReferenceObject ), static_cast<CPhysicsObject *>( pAttachedObject ), static_cast<CPhysicsConstraintGroup *>( pGroup ), length );
}

void CPhysicsEnvironment::DestroyConstraint( IPhysicsConstraint *pConstraint )
{
	if ( !pConstraint )
		return;

	for ( IPhysicsObject *pObject : { pConstraint->GetReferenceObject(), pConstraint->GetAttachedObject() } )
	{
		if ( pObject && !m_bQuickDelete )
			pObject->Wake();
	}

	if ( m_bInSimulation )
	{
		pConstraint->Deactivate();
		m_DeadConstraints.push_back( pConstraint );
		return;
	}

	delete pConstraint;
}

IPhysicsConstraintGroup *CPhysicsEnvironment::CreateConstraintGroup( const constraint_groupparams_t &groupParams )
{
	return new CPhysicsConstraintGroup( this, groupParams.additionalIterations + CPhysicsConstraintGroup::BASE_ITERATIONS, groupParams.minErrorTicks, groupParams.errorTolerance * SOURCE_TO_IVP );
}

void CPhysicsEnvironment::DestroyConstraintGroup( IPhysicsConstraintGroup *pGroup )
{
	m_RestoredGroups.erase( std::remove( m_RestoredGroups.begin(), m_RestoredGroups.end(), pGroup ), m_RestoredGroups.end() );
	delete static_cast<CPhysicsConstraintGroup *>( pGroup );
}

IPhysicsShadowController *CPhysicsEnvironment::CreateShadowController( IPhysicsObject *pObject, bool allowTranslation, bool allowRotation )
{
	return new CPhysicsShadowController( static_cast<CPhysicsObject *>( pObject ), allowTranslation, allowRotation );
}

void CPhysicsEnvironment::DestroyShadowController( IPhysicsShadowController *pController )
{
	delete static_cast<CPhysicsShadowController *>( pController );
}

IPhysicsPlayerController *CPhysicsEnvironment::CreatePlayerController( IPhysicsObject *pObject )
{
	CPhysicsPlayerController *pController = new CPhysicsPlayerController( static_cast<CPhysicsObject *>( pObject ) );
	m_PlayerControllers.push_back( pController );
	return pController;
}

void CPhysicsEnvironment::DestroyPlayerController( IPhysicsPlayerController *pController )
{
	CPhysicsPlayerController *pPlayerController = static_cast<CPhysicsPlayerController *>( pController );
	auto it = std::find( m_PlayerControllers.begin(), m_PlayerControllers.end(), pPlayerController );
	if ( it != m_PlayerControllers.end() )
		m_PlayerControllers.erase( it );
	delete pPlayerController;
}

void CPhysicsEnvironment::RemovePlayerGround( const CPhysicsObject *pObject )
{
	for ( CPhysicsPlayerController *pController : m_PlayerControllers )
		pController->GroundDestroyed( pObject );
}

IPhysicsVehicleController *CPhysicsEnvironment::CreateVehicleController( IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace )
{
	return new CPhysicsVehicleController( this, static_cast<CPhysicsObject *>( pVehicleBodyObject ), params, nVehicleType, pGameTrace );
}

void CPhysicsEnvironment::DestroyVehicleController( IPhysicsVehicleController *pController )
{
	delete static_cast<CPhysicsVehicleController *>( pController );
}

IPhysicsMotionController *CPhysicsEnvironment::CreateMotionController( IMotionEvent *pHandler )
{
	return pHandler ? new CPhysicsMotionController( pHandler ) : nullptr;
}

void CPhysicsEnvironment::DestroyMotionController( IPhysicsMotionController *pController )
{
	delete static_cast<CPhysicsMotionController *>( pController );
}

void CPhysicsEnvironment::Simulate( float deltaTime )
{
	CleanupDeleteList();
	SweepSleepingObjects();

	if ( deltaTime <= MAX_SIMULATE_DELTA && static_cast<double>( deltaTime ) > MIN_SIMULATE_DELTA )
	{
		if ( static_cast<double>( deltaTime ) > MAX_STEP_DELTA )
			deltaTime = static_cast<float>( MAX_STEP_DELTA );

		m_PenetrationPairs.clear();
		if ( m_pCollisionEvents )
			m_pCollisionEvents->PostSimulationFrame();
		PurgePairTimes();

		m_bInSimulation = true;
		double flTarget;
		if ( m_bClockAligned && deltaTime == m_flTimestep )
		{
			flTarget = m_flLastStepTime + static_cast<double>( ALIGNED_STEP_SCALE * m_flTimestep );
		}
		else
		{
			m_bClockAligned = false;
			flTarget = m_flCurrentTime + static_cast<double>( deltaTime );
		}

		if ( m_bFirstStep )
		{
			m_flCurrentTime = m_flLastStepTime;
			Step( m_flLastStepTime );
		}

		while ( static_cast<float>( m_flNextStepTime - m_flLastStepTime ) < static_cast<float>( flTarget - m_flLastStepTime ) )
		{
			m_flCurrentTime = m_flNextStepTime;
			Step( m_flNextStepTime );
		}

		m_flCurrentTime = flTarget;
		m_bInSimulation = false;
	}

	if ( !m_bDeleteQueue )
		CleanupDeleteList();

	if ( m_pCollisionEvents )
		DispatchFriction();
}

void CPhysicsEnvironment::Step( double flTime )
{
	m_bFirstStep = false;
	for ( CPhysicsObject *pObject : m_PendingVelocityObjects )
		pObject->ApplyPendingVelocity();
	m_PendingVelocityObjects.clear();

	++m_nStepCount;
	m_SimulatedObjects.clear();
	for ( CPhysicsObject *pObject : m_ActiveObjects )
	{
		if ( pObject->GetSleepState() == CPhysicsObject::AWAKE )
			m_SimulatedObjects.push_back( pObject );
	}
	for ( CPhysicsObject *pObject : m_WokenObjects )
	{
		pObject->SetQueuedAsWoken( false );
		if ( pObject->GetActiveIndex() == NOT_LISTED && b3Body_IsAwake( pObject->GetBody() ) )
			m_SimulatedObjects.push_back( pObject );
	}
	m_WokenObjects.clear();

	for ( CPhysicsObject *pObject : m_SimulatedObjects )
	{
		if ( pObject->GetRestState() == CPhysicsObject::REST_FROZEN )
			pObject->Revive( m_flCurrentTime, ++m_nReviveSequence );
		pObject->RebuildCoreMatrix( m_flCurrentTime );
		pObject->ResetControllerCursor();
	}
	BuildSimulationUnits();
	UpdateRestTimers();

	for ( CPhysicsObject *pObject : m_SimulatedObjects )
		pObject->RunControllers( m_flTimestep, CPhysicsSpring::PRIORITY );
	for ( CPhysicsObject *pObject : m_SimulatedObjects )
		pObject->RunControllers( m_flTimestep, CPhysicsConstraintGroup::PRIORITY );
	for ( CPhysicsObject *pObject : m_SimulatedObjects )
	{
		pObject->RunControllers( m_flTimestep, std::numeric_limits<int>::min() );
		pObject->ClampStepVelocity();
	}
	CheckSimulationUnits();

	for ( CPhysicsObject *pObject : m_VelocityFlushObjects )
		pObject->FlushVelocity();
	m_VelocityFlushObjects.clear();

	for ( CPhysicsObject *pObject : m_SimulatedObjects )
	{
		pObject->CapturePreStepVelocity( m_nStepCount );
		ResolveFilterDecisions( pObject, false );
	}

	b3World_Step( m_World, m_flTimestep, SUBSTEP_COUNT );
	m_flLastStepTime = flTime;
	m_flNextStepTime = flTime + static_cast<double>( m_flTimestep );

	ResolvePendingFilterPairs();
	ProcessJointEvents();

	m_SteppedObjects.clear();
	std::vector<CPhysicsObject *> woken;
	const b3BodyEvents events = b3World_GetBodyEvents( m_World );
	for ( int i = 0; i < events.moveCount; ++i )
	{
		const b3BodyMoveEvent &event = events.moveEvents[i];
		CPhysicsObject *pObject = static_cast<CPhysicsObject *>( event.userData );
		pObject->UpdateTransform( event.transform );
		pObject->ClampStepVelocity();
		m_SteppedObjects.push_back( pObject );
		if ( pObject->GetSleepState() == CPhysicsObject::AWAKE )
			continue;

		if ( pObject->NotifyWake() == CPhysicsObject::FELL_ASLEEP || pObject->GetActiveIndex() != NOT_LISTED )
			continue;

		pObject->SetActiveIndex( static_cast<unsigned short>( m_ActiveObjects.size() ) );
		m_ActiveObjects.push_back( pObject );
		woken.push_back( pObject );
	}

	for ( CPhysicsObject *pObject : woken )
	{
		ReportPendingLeaves( pObject );
		if ( m_pObjectEvents )
			m_pObjectEvents->ObjectWake( pObject );
	}

	if ( m_pObjectEvents )
	{
		for ( CPhysicsObject *pObject : m_FrozenObjects )
		{
			if ( pObject->GetSleepState() != CPhysicsObject::AWAKE )
				m_pObjectEvents->ObjectSleep( pObject );
		}
	}
	m_FrozenObjects.clear();

	ProcessContactEvents();
	ReleaseDisabledContacts();
	DispatchContactEvents();
	ProcessSensorEvents();
	ProcessPenetration();
}

void CPhysicsEnvironment::ProcessJointEvents()
{
	const b3JointEvents events = b3World_GetJointEvents( m_World );
	if ( events.count == 0 || !m_pConstraintEvents )
		return;

	m_BrokenConstraints.clear();
	for ( int i = 0; i < events.count; ++i )
	{
		if ( events.jointEvents[i].userData )
			m_BrokenConstraints.push_back( static_cast<CPhysicsConstraint *>( events.jointEvents[i].userData ) );
	}

	for ( CPhysicsConstraint *pConstraint : m_BrokenConstraints )
	{
		if ( pConstraint->HasJoint() && m_pConstraintEvents )
			m_pConstraintEvents->ConstraintBroken( pConstraint );
	}
}

void CPhysicsEnvironment::NotifyWoken( CPhysicsObject *pObject )
{
	if ( pObject->GetActiveIndex() != NOT_LISTED || pObject->IsQueuedAsWoken() )
		return;

	pObject->SetQueuedAsWoken( true );
	m_WokenObjects.push_back( pObject );
}

void CPhysicsEnvironment::RemoveWoken( CPhysicsObject *pObject )
{
	if ( !pObject->IsQueuedAsWoken() )
		return;

	pObject->SetQueuedAsWoken( false );
	m_WokenObjects.erase( std::find( m_WokenObjects.begin(), m_WokenObjects.end(), pObject ) );
}

void CPhysicsEnvironment::SyncSleepingObjects()
{
	for ( CPhysicsObject *pObject : m_ActiveObjects )
	{
		if ( pObject->GetSleepState() != CPhysicsObject::AWAKE || b3Body_IsAwake( pObject->GetBody() ) )
			continue;

		pObject->NotifySleep( b3Body_GetTransform( pObject->GetBody() ) );
		if ( m_pObjectEvents )
			m_pObjectEvents->ObjectSleep( pObject );
	}
}

void CPhysicsEnvironment::RemovePendingVelocity( CPhysicsObject *pObject )
{
	auto it = std::find( m_PendingVelocityObjects.begin(), m_PendingVelocityObjects.end(), pObject );
	if ( it != m_PendingVelocityObjects.end() )
		m_PendingVelocityObjects.erase( it );
}

void CPhysicsEnvironment::RemoveVelocityFlush( CPhysicsObject *pObject )
{
	auto it = std::find( m_VelocityFlushObjects.begin(), m_VelocityFlushObjects.end(), pObject );
	if ( it != m_VelocityFlushObjects.end() )
		m_VelocityFlushObjects.erase( it );
}

static uint32_t s_nSleepSeed = 1;

static float GetSleepRandom()
{
	s_nSleepSeed *= SLEEP_RANDOM_MULTIPLIER;
	return static_cast<float>( s_nSleepSeed & 0xFFFF ) * SLEEP_RANDOM_SCALE;
}

static void CollectUnit( CPhysicsObject *pObject, uint32_t nStamp, std::vector<CPhysicsObject *> &objects )
{
	static thread_local std::vector<b3JointId> s_Joints;
	const size_t nFirst = objects.size();
	pObject->SetUnitStamp( nStamp );
	objects.push_back( pObject );
	for ( size_t i = nFirst; i < objects.size(); ++i )
	{
		const b3BodyId body = objects[i]->GetBody();
		s_Joints.resize( b3Body_GetJointCount( body ) );
		b3Body_GetJoints( body, s_Joints.data(), static_cast<int>( s_Joints.size() ) );
		for ( b3JointId joint : s_Joints )
		{
			for ( b3BodyId other : { b3Joint_GetBodyA( joint ), b3Joint_GetBodyB( joint ) } )
			{
				CPhysicsObject *pOther = static_cast<CPhysicsObject *>( b3Body_GetUserData( other ) );
				if ( pOther->IsStatic() || pOther->GetUnitStamp() == nStamp )
					continue;

				pOther->SetUnitStamp( nStamp );
				objects.push_back( pOther );
			}
		}
	}
}

void CPhysicsEnvironment::BuildSimulationUnits()
{
	m_UnitObjects.clear();
	m_SimulationUnits.clear();
	const uint32_t nStamp = ++m_nUnitStamp;
	for ( size_t nObject = 0; nObject < m_SimulatedObjects.size(); ++nObject )
	{
		CPhysicsObject *pObject = m_SimulatedObjects[nObject];
		if ( pObject->GetUnitStamp() == nStamp )
			continue;

		SimulationUnit_t &unit = m_SimulationUnits.emplace_back();
		unit.m_nFirst = static_cast<int>( m_UnitObjects.size() );
		CollectUnit( pObject, nStamp, m_UnitObjects );
		unit.m_nCount = static_cast<int>( m_UnitObjects.size() ) - unit.m_nFirst;
		unit.m_nSequence = 0;
		unit.m_bCheckRest = false;
		for ( int i = unit.m_nFirst; i < unit.m_nFirst + unit.m_nCount; ++i )
		{
			CPhysicsObject *pMember = m_UnitObjects[i];
			if ( pMember->GetLastStepTime() != m_flCurrentTime )
			{
				if ( pMember->GetRestState() == CPhysicsObject::REST_FROZEN )
					pMember->Revive( m_flCurrentTime, ++m_nReviveSequence );
				pMember->RebuildCoreMatrix( m_flCurrentTime );
				pMember->ResetControllerCursor();
				m_SimulatedObjects.push_back( pMember );
			}
			unit.m_nSequence = std::max( unit.m_nSequence, pMember->GetReviveSequence() );
		}
	}
	std::sort( m_SimulationUnits.begin(), m_SimulationUnits.end(), []( const SimulationUnit_t &a, const SimulationUnit_t &b ) { return a.m_nSequence != b.m_nSequence ? a.m_nSequence > b.m_nSequence : a.m_nFirst < b.m_nFirst; } );
}

void CPhysicsEnvironment::UpdateRestTimers()
{
	for ( SimulationUnit_t &unit : m_SimulationUnits )
	{
		CPhysicsObject **ppObjects = m_UnitObjects.data() + unit.m_nFirst;
		bool bFast = false;
		bool bWasFast = false;
		for ( int i = 0; i < unit.m_nCount; ++i )
		{
			bFast = bFast || 1.0f - b3LengthSquared( ppObjects[i]->GetCoreVelocity() ) < 0.0f;
			bWasFast = bWasFast || ppObjects[i]->IsRestFast();
		}

		for ( int i = 0; i < unit.m_nCount; ++i )
		{
			if ( !bFast && bWasFast )
				ppObjects[i]->ResetRestTimers( m_flCurrentTime );
			ppObjects[i]->SetRestFast( bFast );
		}

		if ( --m_nSleepCountdown == 0 )
		{
			m_nSleepCountdown = static_cast<short>( SLEEP_COUNTDOWN_BASE - static_cast<short>( static_cast<int>( GetSleepRandom() * -SLEEP_COUNTDOWN_RANGE ) ) );
			unit.m_bCheckRest = true;
		}
	}
}

void CPhysicsEnvironment::CheckSimulationUnits()
{
	for ( const SimulationUnit_t &unit : m_SimulationUnits )
	{
		if ( !unit.m_bCheckRest || m_UnitObjects[unit.m_nFirst]->GetRestState() == CPhysicsObject::REST_FROZEN )
			continue;

		int nRest = CPhysicsObject::REST_READY;
		for ( int i = unit.m_nFirst + unit.m_nCount - 1; i >= unit.m_nFirst; --i )
			nRest &= m_UnitObjects[i]->CheckRest( m_flCurrentTime, true );
		if ( nRest == CPhysicsObject::REST_READY )
			FreezeIsland( unit );
	}
}

void CPhysicsEnvironment::FreezeIsland( const SimulationUnit_t &unit )
{
	m_IslandObjects.assign( m_UnitObjects.begin() + unit.m_nFirst, m_UnitObjects.begin() + unit.m_nFirst + unit.m_nCount );
	const uint32_t nStamp = ++m_nUnitStamp;
	for ( CPhysicsObject *pObject : m_IslandObjects )
		pObject->SetUnitStamp( nStamp );

	for ( size_t i = 0; i < m_IslandObjects.size(); ++i )
	{
		for ( uint64_t nKey : m_IslandObjects[i]->GetContactKeys() )
		{
			const Contact_t &contact = m_Contacts.find( nKey )->second;
			CPhysicsObject *pPartner = contact.m_pObjects[contact.m_pObjects[0] == m_IslandObjects[i] ? 1 : 0];
			if ( pPartner->IsStatic() || pPartner->GetSleepState() != CPhysicsObject::AWAKE || pPartner->GetUnitStamp() == nStamp )
				continue;

			const size_t nFirst = m_IslandObjects.size();
			CollectUnit( pPartner, nStamp, m_IslandObjects );
			for ( size_t j = nFirst; j < m_IslandObjects.size(); ++j )
			{
				if ( m_IslandObjects[j]->IsMoveable() && !m_IslandObjects[j]->IsReadyToRest( m_flCurrentTime ) )
					return;
			}
		}
	}

	for ( CPhysicsObject *pObject : m_IslandObjects )
	{
		if ( !pObject->IsMoveable() )
			continue;

		pObject->Freeze( true );
		b3Body_SetAwake( pObject->GetBody(), false );
		m_FrozenObjects.push_back( pObject );
	}
	SyncSleepingObjects();
}

void CPhysicsEnvironment::ForceRest( CPhysicsObject *pObject )
{
	if ( pObject->IsAsleep() )
		return;

	m_IslandObjects.clear();
	CollectUnit( pObject, ++m_nUnitStamp, m_IslandObjects );
	for ( CPhysicsObject *pMember : m_IslandObjects )
	{
		pMember->PrepareForcedRest( m_flCurrentTime );
		if ( pMember->CheckRest( m_flCurrentTime, false ) == CPhysicsObject::REST_MOVING )
			return;
	}

	for ( CPhysicsObject *pMember : m_IslandObjects )
	{
		RemoveWoken( pMember );
		pMember->Freeze( false );
		b3Body_SetAwake( pMember->GetBody(), false );
		if ( m_pObjectEvents && pMember->GetSleepState() == CPhysicsObject::FELL_ASLEEP )
			m_pObjectEvents->ObjectSleep( pMember );
	}
	SyncSleepingObjects();
}

void CPhysicsEnvironment::SweepSleepingObjects()
{
	for ( int i = static_cast<int>( m_ActiveObjects.size() ) - 1; i >= 0; --i )
	{
		CPhysicsObject *pObject = m_ActiveObjects[i];
		if ( pObject->GetSleepState() == CPhysicsObject::AWAKE )
			continue;

		pObject->SetSleepState( CPhysicsObject::ASLEEP );
		RemoveActiveObject( pObject );
	}
}

void CPhysicsEnvironment::RemoveActiveObject( CPhysicsObject *pObject )
{
	const unsigned short nIndex = pObject->GetActiveIndex();
	if ( nIndex >= m_ActiveObjects.size() )
		return;

	m_ActiveObjects[nIndex] = m_ActiveObjects.back();
	m_ActiveObjects.pop_back();
	if ( nIndex < m_ActiveObjects.size() )
		m_ActiveObjects[nIndex]->SetActiveIndex( nIndex );

	pObject->SetActiveIndex( NOT_LISTED );
}

void CPhysicsEnvironment::SetSimulationTimestep( float timestep )
{
	m_flTimestep = timestep;
#if defined( VPHYSICS_GMOD )
	m_flInverseTimestep = static_cast<float>( 1.0 / static_cast<double>( timestep ) );
#else
	m_flInverseTimestep = 1.0f / timestep;
#endif
	UpdateContactTuning();
}

void CPhysicsEnvironment::ResetSimulationClock()
{
	for ( IPhysicsObject *pObject : m_Objects )
		static_cast<CPhysicsObject *>( pObject )->ResetClock( m_flLastStepTime );

	m_flCurrentTime = 0.0;
	m_flLastStepTime = 0.0;
	m_flNextStepTime = m_flTimestep;
	m_bClockAligned = true;
	s_nSleepSeed = 1;
}

void CPhysicsEnvironment::GetActiveObjects( IPhysicsObject **pOutputObjectList ) const
{
	std::copy( m_ActiveObjects.begin(), m_ActiveObjects.end(), pOutputObjectList );
}

const IPhysicsObject **CPhysicsEnvironment::GetObjectList( int *pOutputObjectCount ) const
{
	if ( pOutputObjectCount )
		*pOutputObjectCount = static_cast<int>( m_Objects.size() );

	if ( m_Objects.empty() )
		return nullptr;

	return const_cast<const IPhysicsObject **>( m_Objects.data() );
}

void CPhysicsEnvironment::CleanupDeleteList()
{
	for ( size_t i = 0; i < m_DeadObjects.size(); ++i )
	{
		RemoveActiveObject( m_DeadObjects[i] );
		delete m_DeadObjects[i];
	}
	m_DeadObjects.clear();

	for ( int i = static_cast<int>( m_DeadConstraints.size() ) - 1; i >= 0; --i )
		delete m_DeadConstraints[i];
	m_DeadConstraints.clear();
}

bool CPhysicsEnvironment::IsCollisionModelUsed( CPhysCollide *pCollide ) const
{
	for ( int i = static_cast<int>( m_DeadObjects.size() ) - 1; i >= 0; --i )
	{
		if ( m_DeadObjects[i]->GetCollide() == pCollide )
			return true;
	}

	for ( int i = static_cast<int>( m_Objects.size() ) - 1; i >= 0; --i )
	{
		if ( m_Objects[i]->GetCollide() == pCollide )
			return true;
	}

	return false;
}

void CPhysicsEnvironment::GetPerformanceSettings( physics_performanceparams_t *pOutput ) const
{
	if ( !pOutput )
		return;

	*pOutput = m_PerformanceSettings;
	pOutput->maxVelocity = m_flMaxSpeed * IVP_TO_SOURCE;
	pOutput->maxAngularVelocity = m_flMaxRotationPerStep * RADIANS_TO_DEGREES * ( 1.0f / m_flTimestep );
}

void CPhysicsEnvironment::SetPerformanceSettings( const physics_performanceparams_t *pSettings )
{
	if ( !pSettings )
		return;

	m_PerformanceSettings = *pSettings;
	m_flMaxSpeed = pSettings->maxVelocity * SOURCE_TO_IVP;
	m_flMaxRotationPerStep = pSettings->maxAngularVelocity * DEGREES_TO_RADIANS * m_flTimestep;
	m_PerformanceSettings.minFrictionMass = std::clamp( pSettings->minFrictionMass, MIN_FRICTION_MASS, MAX_FRICTION_MASS );
	m_PerformanceSettings.maxFrictionMass = std::clamp( pSettings->maxFrictionMass, MIN_FRICTION_MASS, MAX_FRICTION_MASS );
}

void CPhysicsEnvironment::ReadStats( physics_stats_t *pOutput )
{
	if ( pOutput )
		*pOutput = m_Stats;
}

void CPhysicsEnvironment::ClearStats()
{
	const float flTotalEnergyDestroyed = m_Stats.totalEnergyDestroyed;
	const int nCollisionPairsTotal = m_Stats.collisionPairsTotal;
	m_Stats = {};
	m_Stats.totalEnergyDestroyed = flTotalEnergyDestroyed;
	m_Stats.collisionPairsTotal = nCollisionPairsTotal;
}

bool CPhysicsEnvironment::EvaluateCollision( CPhysicsObject *pObject0, CPhysicsObject *pObject1 ) const
{
	if ( IsPendingDeletePair( pObject0, pObject1 ) )
		return false;

	return !m_pCollisionSolver || m_pCollisionSolver->ShouldCollide( pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData() ) != 0;
}

bool CPhysicsEnvironment::ShouldCollide( CPhysicsObject *pObject0, CPhysicsObject *pObject1 )
{
	if ( IsPendingDeletePair( pObject0, pObject1 ) )
		return false;

	if ( !m_pCollisionSolver )
		return true;

	bool bCollide;
	if ( pObject0->FindFilterDecision( pObject1, bCollide ) )
		return bCollide;

	bCollide = m_pCollisionSolver->ShouldCollide( pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData() ) != 0;
	pObject0->StoreFilterDecision( pObject1, bCollide );
	return bCollide;
}

bool CPhysicsEnvironment::IsCollisionAllowed( CPhysicsObject *pObject0, CPhysicsObject *pObject1 )
{
	if ( !pObject0 || !pObject1 )
		return true;

	if ( IsPendingDeletePair( pObject0, pObject1 ) )
		return false;

	if ( !m_pCollisionSolver )
		return true;

	bool bCollide;
	if ( pObject0->FindFilterDecision( pObject1, bCollide ) )
		return bCollide;

	std::lock_guard lock( m_PendingFilterMutex );
	m_PendingFilterPairs.emplace_back( pObject0, pObject1 );
	return true;
}

bool CPhysicsEnvironment::IsContactEnabled( const CPhysicsObject *pObject0, const CPhysicsObject *pObject1 ) const
{
	if ( !pObject0 || !pObject1 )
		return true;

	if ( IsPendingDeletePair( pObject0, pObject1 ) )
		return false;

	return m_DisabledPairs.empty() || !m_DisabledPairs.contains( GetPairKey( pObject0, pObject1 ) );
}

void CPhysicsEnvironment::ResolveFilterDecisions( CPhysicsObject *pObject, bool bForce )
{
	if ( !m_pCollisionSolver || !pObject->IsCollisionEnabled() )
		return;

	const b3AABB bounds = b3Body_ComputeAABB( pObject->GetBody() );
	if ( !bForce && pObject->HasFilterBounds() && b3AABB_Contains( pObject->GetFilterBounds(), bounds ) )
		return;

	const b3Vec3 extent = b3Sub( bounds.upperBound, bounds.lowerBound );
	const float flMargin = B3_AABB_MARGIN_FRACTION * std::max( std::max( extent.x, extent.y ), extent.z ) + b3Length( b3Body_GetLinearVelocity( pObject->GetBody() ) ) * m_flTimestep;
	const b3Vec3 margin = { flMargin, flMargin, flMargin };
	const b3AABB query = { b3Sub( bounds.lowerBound, margin ), b3Add( bounds.upperBound, margin ) };
	pObject->SetFilterBounds( query );

	static thread_local std::vector<CPhysicsObject *> s_Candidates;
	s_Candidates.clear();
	b3World_OverlapAABB(
		m_World, query, b3DefaultQueryFilter(),
		[]( b3ShapeId shape, void *pContext ) {
			static_cast<std::vector<CPhysicsObject *> *>( pContext )->push_back( GetShapeObject( shape ) );
			return true;
		},
		&s_Candidates );

	const std::vector<CPhysicsObject *> candidates = s_Candidates;
	for ( CPhysicsObject *pPartner : candidates )
	{
		bool bCollide;
		if ( pPartner && pPartner != pObject && pPartner->IsCollisionEnabled() && !pObject->FindFilterDecision( pPartner, bCollide ) )
			ShouldCollide( pObject, pPartner );
	}
}

void CPhysicsEnvironment::ResolvePendingFilterPairs()
{
	if ( m_PendingFilterPairs.empty() )
		return;

	std::vector<std::pair<CPhysicsObject *, CPhysicsObject *>> pairs;
	pairs.swap( m_PendingFilterPairs );
	for ( const auto &[pObject0, pObject1] : pairs )
	{
		bool bCollide;
		if ( pObject0->FindFilterDecision( pObject1, bCollide ) || ShouldCollide( pObject0, pObject1 ) )
			continue;

		CPhysicsObject *pReset = pObject0->IsStatic() ? pObject1 : pObject0;
		pReset->ResetShapeProxies();
	}
}

void CPhysicsEnvironment::DisablePair( uint64_t nPairKey )
{
	++m_DisabledPairs[nPairKey];
}

void CPhysicsEnvironment::EnablePair( uint64_t nPairKey )
{
	auto it = m_DisabledPairs.find( nPairKey );
	if ( it != m_DisabledPairs.end() && --it->second <= 0 )
		m_DisabledPairs.erase( it );
}

void CPhysicsEnvironment::RecheckCollisionFilter( CPhysicsObject *pObject )
{
	if ( pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE )
		return;

	pObject->SetCallbackFlags( pObject->GetCallbackFlags() | CALLBACK_ENABLING_COLLISION );

	std::vector<std::pair<CPhysicsObject *, bool>> previous;
	if ( m_pCollisionSolver && pObject->IsCollisionEnabled() )
	{
		b3AABB query = b3Body_ComputeAABB( pObject->GetBody() );
		if ( pObject->HasFilterBounds() )
			query = pObject->GetFilterBounds();

		static thread_local std::vector<CPhysicsObject *> s_Partners;
		s_Partners.clear();
		b3World_OverlapAABB(
			m_World, query, b3DefaultQueryFilter(),
			[]( b3ShapeId shape, void *pContext ) {
				static_cast<std::vector<CPhysicsObject *> *>( pContext )->push_back( GetShapeObject( shape ) );
				return true;
			},
			&s_Partners );

		for ( CPhysicsObject *pPartner : s_Partners )
		{
			if ( !pPartner || pPartner == pObject || !pPartner->IsCollisionEnabled() )
				continue;

			bool bCollide = true;
			pObject->FindFilterDecision( pPartner, bCollide );
			previous.emplace_back( pPartner, bCollide );
		}
	}

	pObject->ClearFilterDecisions();

	bool bChanged = false;
	for ( const auto &[pPartner, bPrevious] : previous )
		bChanged |= ShouldCollide( pObject, pPartner ) != bPrevious;

	if ( bChanged )
		pObject->ResetShapeProxies();

	pObject->SetCallbackFlags( pObject->GetCallbackFlags() & ~CALLBACK_ENABLING_COLLISION );
}

void CPhysicsEnvironment::ResetCollisionPairs( CPhysicsObject *pObject )
{
	const b3BodyId body = pObject->GetBody();
	std::vector<b3ContactData> contacts( b3Body_GetContactCapacity( body ) );
	contacts.resize( b3Body_GetContactData( body, contacts.data(), static_cast<int>( contacts.size() ) ) );
	for ( const b3ContactData &data : contacts )
	{
		auto it = m_Contacts.find( GetContactKey( data.contactId ) );
		if ( it == m_Contacts.end() )
			continue;

		m_ProxyResetContacts[GetShapePairKey( data.shapeIdA, data.shapeIdB )] = std::move( it->second );
		EraseContact( it );
	}
}

void CPhysicsEnvironment::PrepareShapeReset( CPhysicsObject *pObject )
{
	if ( pObject->GetContactKeys().empty() )
		return;

	pObject->Wake();
	for ( uint64_t nKey : pObject->GetContactKeys() )
	{
		const Contact_t &contact = m_Contacts.find( nKey )->second;
		contact.m_pObjects[contact.m_pObjects[0] == pObject ? 1 : 0]->Wake();
	}

	ResetCollisionPairs( pObject );
}

void CPhysicsEnvironment::RecheckContactPoints( CPhysicsObject *pObject )
{
	std::vector<CPhysicsObject *> partners;
	for ( uint64_t nKey : pObject->GetContactKeys() )
	{
		const Contact_t &contact = m_Contacts.find( nKey )->second;
		CPhysicsObject *pPartner = contact.m_pObjects[contact.m_pObjects[0] == pObject ? 1 : 0];
		const bool bActive = std::any_of( contact.m_Friction.begin(), contact.m_Friction.end(), []( const FrictionContact_t &friction ) { return !friction.m_bRemoved; } );
		if ( bActive && std::find( partners.begin(), partners.end(), pPartner ) == partners.end() )
			partners.push_back( pPartner );
	}

	for ( CPhysicsObject *pPartner : partners )
	{
		if ( EvaluateCollision( pObject, pPartner ) )
			continue;

		pPartner->Wake();
		RemoveContacts( pObject, pPartner );
	}
}

void CPhysicsEnvironment::RemoveContacts( CPhysicsObject *pObject, CPhysicsObject *pPartner )
{
	struct Removed_t
	{
		uint64_t m_nSequence;
		CPhysicsObject *m_pObjects[2];
		Vector m_Point;
		Vector m_Normal;
	};

	const uint64_t nPairKey = GetPairKey( pObject, pPartner );
	std::vector<Removed_t> removed;
	for ( uint64_t nKey : pObject->GetContactKeys() )
	{
		Contact_t &contact = m_Contacts.find( nKey )->second;
		if ( contact.m_pObjects[0] != pPartner && contact.m_pObjects[1] != pPartner )
			continue;

		for ( FrictionContact_t &friction : contact.m_Friction )
		{
			if ( friction.m_bRemoved )
				continue;

			friction.m_bRemoved = true;
			DisablePair( nPairKey );
			++contact.m_nDisableCount;
			removed.push_back( { friction.m_nSequence, { contact.m_pObjects[friction.m_nSide], contact.m_pObjects[1 - friction.m_nSide] }, friction.m_Point, friction.m_Normal } );
		}
	}

	std::sort( removed.begin(), removed.end(), []( const Removed_t &a, const Removed_t &b ) { return a.m_nSequence > b.m_nSequence; } );
	for ( const Removed_t &entry : removed )
		ReportTouch( entry.m_pObjects[0], entry.m_pObjects[1], entry.m_Point, entry.m_Normal, false );
}

void CPhysicsEnvironment::RemoveAllContacts( CPhysicsObject *pObject )
{
	std::vector<CPhysicsObject *> partners;
	for ( uint64_t nKey : pObject->GetContactKeys() )
	{
		const Contact_t &contact = m_Contacts.find( nKey )->second;
		CPhysicsObject *pPartner = contact.m_pObjects[contact.m_pObjects[0] == pObject ? 1 : 0];
		if ( std::find( partners.begin(), partners.end(), pPartner ) == partners.end() )
			partners.push_back( pPartner );
	}

	for ( CPhysicsObject *pPartner : partners )
	{
		pPartner->Wake();
		RemoveContacts( pObject, pPartner );
	}
}

void CPhysicsEnvironment::RemoveObjectContacts( CPhysicsObject *pObject )
{
	const auto release = [this]( const Contact_t &contact ) {
		for ( const FrictionContact_t &friction : contact.m_Friction )
			ReleaseFriction( contact, friction, false );
		const uint64_t nPairKey = GetPairKey( contact.m_pObjects[0], contact.m_pObjects[1] );
		for ( int i = 0; i < contact.m_nDisableCount; ++i )
			EnablePair( nPairKey );
	};

	while ( !pObject->GetContactKeys().empty() )
	{
		auto it = m_Contacts.find( pObject->GetContactKeys().back() );
		release( it->second );
		EraseContact( it );
	}

	for ( auto it = m_ProxyResetContacts.begin(); it != m_ProxyResetContacts.end(); )
	{
		if ( it->second.m_pObjects[0] != pObject && it->second.m_pObjects[1] != pObject )
		{
			++it;
			continue;
		}

		release( it->second );
		it = m_ProxyResetContacts.erase( it );
	}

	for ( auto it = m_DisabledContacts.begin(); it != m_DisabledContacts.end(); )
	{
		if ( !IsPairMember( it->m_nPairKey, pObject ) )
		{
			++it;
			continue;
		}

		for ( int i = 0; i < it->m_nCount; ++i )
			EnablePair( it->m_nPairKey );
		it = m_DisabledContacts.erase( it );
	}

	for ( auto it = m_TriggerOverlaps.begin(); it != m_TriggerOverlaps.end(); )
		it = it->second.m_pTrigger == pObject || it->second.m_pObject == pObject ? m_TriggerOverlaps.erase( it ) : std::next( it );

	std::lock_guard lock( m_PendingFilterMutex );
	std::erase_if( m_PendingFilterPairs, [pObject]( const auto &pair ) { return pair.first == pObject || pair.second == pObject; } );
}

void CPhysicsEnvironment::InsertContact( uint64_t nKey, Contact_t &&contact )
{
	Contact_t &stored = m_Contacts.insert_or_assign( nKey, std::move( contact ) ).first->second;
	for ( int i = 0; i < 2; ++i )
	{
		std::vector<uint64_t> &keys = stored.m_pObjects[i]->GetContactKeys();
		stored.m_nKeyIndex[i] = static_cast<uint32_t>( keys.size() );
		keys.push_back( nKey );
	}
}

void CPhysicsEnvironment::EraseContact( std::unordered_map<uint64_t, Contact_t>::iterator it )
{
	const Contact_t &contact = it->second;
	for ( int i = 0; i < 2; ++i )
	{
		CPhysicsObject *pObject = contact.m_pObjects[i];
		std::vector<uint64_t> &keys = pObject->GetContactKeys();
		const uint32_t nIndex = contact.m_nKeyIndex[i];
		keys[nIndex] = keys.back();
		keys.pop_back();
		if ( nIndex == keys.size() )
			continue;

		Contact_t &moved = m_Contacts.find( keys[nIndex] )->second;
		moved.m_nKeyIndex[moved.m_pObjects[0] == pObject ? 0 : 1] = nIndex;
	}

	m_Contacts.erase( it );
}

void CPhysicsEnvironment::RetireContact( const Contact_t &contact, bool bReport )
{
	for ( const FrictionContact_t &friction : contact.m_Friction )
		ReleaseFriction( contact, friction, bReport );

	if ( contact.m_nDisableCount == 0 )
		return;

	const uint64_t nPairKey = GetPairKey( contact.m_pObjects[0], contact.m_pObjects[1] );
	if ( b3Contact_IsValid( contact.m_ContactId ) )
	{
		m_DisabledContacts.push_back( { contact.m_ContactId, nPairKey, contact.m_nDisableCount } );
		return;
	}

	for ( int i = 0; i < contact.m_nDisableCount; ++i )
		EnablePair( nPairKey );
}

void CPhysicsEnvironment::ReleaseDisabledContacts()
{
	for ( auto it = m_DisabledContacts.begin(); it != m_DisabledContacts.end(); )
	{
		if ( b3Contact_IsValid( it->m_ContactId ) )
		{
			++it;
			continue;
		}

		for ( int i = 0; i < it->m_nCount; ++i )
			EnablePair( it->m_nPairKey );
		it = m_DisabledContacts.erase( it );
	}
}

void CPhysicsEnvironment::AddFrictionPair( uint64_t nPairKey )
{
	auto [pair, bCreated] = m_PairTimes.try_emplace( nPairKey, PairTime_t{ UNKNOWN_IMPACT_TIME, 0 } );
	++pair->second.m_nContactCount;
	if ( !bCreated )
		return;

	auto cached = m_PairTimeCache.find( nPairKey );
	if ( cached == m_PairTimeCache.end() )
	{
		if ( m_PairTimeCache.size() < MAX_CACHED_PAIR_TIMES )
			m_PairTimeCache.emplace( nPairKey, pair->second.m_flLastImpactTime );
	}
	else if ( cached->second > pair->second.m_flLastImpactTime )
	{
		pair->second.m_flLastImpactTime = cached->second;
	}
	else
	{
		cached->second = pair->second.m_flLastImpactTime;
	}
}

void CPhysicsEnvironment::RemoveFrictionPair( uint64_t nPairKey )
{
	auto pair = m_PairTimes.find( nPairKey );
	if ( pair == m_PairTimes.end() || --pair->second.m_nContactCount > 0 )
		return;

	auto cached = m_PairTimeCache.find( nPairKey );
	if ( cached != m_PairTimeCache.end() )
		cached->second = std::max( cached->second, pair->second.m_flLastImpactTime );
	else if ( m_PairTimeCache.size() < MAX_CACHED_PAIR_TIMES )
		m_PairTimeCache.emplace( nPairKey, pair->second.m_flLastImpactTime );
	m_PairTimes.erase( pair );
}

void CPhysicsEnvironment::PurgePairTimes()
{
	std::erase_if( m_PairTimeCache, [this]( const auto &entry ) { return m_flCurrentTime - entry.second > PAIR_TIME_LIFETIME; } );
}

void CPhysicsEnvironment::ReleaseFriction( const Contact_t &contact, const FrictionContact_t &friction, bool bReport )
{
	RemoveFrictionPair( GetPairKey( contact.m_pObjects[0], contact.m_pObjects[1] ) );
	if ( !friction.m_bRemoved && bReport )
	{
		QueueContactEvent( ContactEvent_t::END_TOUCH, contact, friction, 0.0f );
		Contact_t &ending = m_EndingContacts.emplace_back( contact );
		ending.m_Friction.assign( 1, friction );
	}
}

void CPhysicsEnvironment::QueueContactEvent( ContactEvent_t::Type_t nType, const Contact_t &contact, const FrictionContact_t &friction, float flApproachSpeed )
{
	ContactEvent_t &event = m_ContactEvents.emplace_back();
	event.m_nType = nType;
	event.m_pObjects[0] = contact.m_pObjects[friction.m_nSide];
	event.m_pObjects[1] = contact.m_pObjects[1 - friction.m_nSide];
	event.m_Materials[0] = friction.m_Materials[0];
	event.m_Materials[1] = friction.m_Materials[1];
	event.m_Point = friction.m_Point;
	event.m_Normal = friction.m_Normal;
	event.m_flApproachSpeed = flApproachSpeed;
}

void CPhysicsEnvironment::ReportTouch( CPhysicsObject *pObject0, CPhysicsObject *pObject1, const Vector &point, const Vector &normal, bool bStart )
{
	if ( !m_pCollisionEvents )
		return;

	const unsigned short nFlags = pObject0->GetCallbackFlags() | pObject1->GetCallbackFlags();
	if ( !( nFlags & CALLBACK_GLOBAL_TOUCH ) )
		return;

	if ( ( pObject0->IsStatic() || pObject1->IsStatic() ) && !( nFlags & CALLBACK_GLOBAL_TOUCH_STATIC ) )
		return;

	CCollisionData data( normal, point, Vector( 0.0f, 0.0f, 0.0f ) );
	if ( bStart )
		m_pCollisionEvents->StartTouch( pObject0, pObject1, &data );
	else
		m_pCollisionEvents->EndTouch( pObject0, pObject1, &data );
}

void CPhysicsEnvironment::ReportImpact( const ContactEvent_t &event )
{
	CPhysicsObject *pObject0 = event.m_pObjects[0], *pObject1 = event.m_pObjects[1];
	const unsigned short nFlags0 = pObject0->GetCallbackFlags(), nFlags1 = pObject1->GetCallbackFlags();
	bool bCollision = ( nFlags0 & CALLBACK_GLOBAL_COLLISION ) && ( nFlags1 & CALLBACK_GLOBAL_COLLISION );
	const bool bShadowCollision = ( ( nFlags0 ^ nFlags1 ) & CALLBACK_SHADOW_COLLISION ) != 0;
	if ( ( pObject0->IsStatic() && !( nFlags1 & CALLBACK_GLOBAL_COLLIDE_STATIC ) ) || ( pObject1->IsStatic() && !( nFlags0 & CALLBACK_GLOBAL_COLLIDE_STATIC ) ) )
		bCollision = false;
	if ( !bCollision && !bShadowCollision )
		return;

	auto pair = m_PairTimes.try_emplace( GetPairKey( pObject0, pObject1 ), PairTime_t{ UNKNOWN_IMPACT_TIME, 0 } ).first;
	float flInterval = static_cast<float>( m_flLastStepTime - pair->second.m_flLastImpactTime );
	pair->second.m_flLastImpactTime = m_flLastStepTime;
	if ( flInterval > MAX_IMPACT_INTERVAL )
		flInterval = DEFAULT_IMPACT_INTERVAL;

	const Vector speed = pObject1->GetPreStepVelocityAtPoint( event.m_Point ) - pObject0->GetPreStepVelocityAtPoint( event.m_Point );
	CCollisionData data( event.m_Normal, event.m_Point, speed );

	m_CollisionEvent.pObjects[0] = pObject0;
	m_CollisionEvent.pObjects[1] = pObject1;
	m_CollisionEvent.surfaceProps[0] = event.m_Materials[0];
	m_CollisionEvent.surfaceProps[1] = event.m_Materials[1];
	m_CollisionEvent.isCollision = bCollision;
	m_CollisionEvent.isShadowCollision = bShadowCollision;
	m_CollisionEvent.deltaCollisionTime = flInterval;
	m_CollisionEvent.pInternalData = &data;

	pObject0->SetReportPreStepVelocity( true );
	pObject1->SetReportPreStepVelocity( true );
	m_pCollisionEvents->PreCollision( &m_CollisionEvent );
	pObject0->SetReportPreStepVelocity( false );
	pObject1->SetReportPreStepVelocity( false );

	m_CollisionEvent.collisionSpeed = event.m_flApproachSpeed;
	m_pCollisionEvents->PostCollision( &m_CollisionEvent );
}

int CPhysicsEnvironment::GetContactMaterial( const CPhysicsObject *pObject, const CPhysConvex *pConvex, const Vector &point, const Vector &outwardNormal, bool bFaceFeature ) const
{
	if ( !bFaceFeature || !pConvex )
		return pObject->GetMaterialIndex();

	const b3WorldTransform transform = b3Body_GetTransform( pObject->GetBody() );
	const b3Vec3 localNormal = b3InvRotateVector( transform.q, ToBox3D( outwardNormal ) );
	const b3Vec3 localPoint = b3InvRotateVector( transform.q, b3Sub( ToBox3D( point ), b3ToVec3( transform.p ) ) );
	const int nMaterial = pConvex->FindTriangleMaterial( localNormal, localPoint );
	return nMaterial ? g_PhysicsSurfaceProps.GetWorldMaterialIndex( nMaterial ) : pObject->GetMaterialIndex();
}

void CPhysicsEnvironment::UpdateFrictionContacts( Contact_t &contact )
{
	const b3ContactData data = b3Contact_GetData( contact.m_ContactId );
	const bool bBall = IsBall( contact.m_pObjects[0] ) || IsBall( contact.m_pObjects[1] );
	const b3BodyId bodies[2] = { contact.m_pObjects[0]->GetBody(), contact.m_pObjects[1]->GetBody() };
	const b3Vec3 center = b3ToVec3( b3Body_GetWorldCenter( bodies[0] ) );
	ptrdiff_t nImpact = -1;
	float flImpactArrival = 0.0f, flImpactSpeed = 0.0f;
	for ( int m = 0; m < data.manifoldCount; ++m )
	{
		const b3Manifold &manifold = data.manifolds[m];
		const Vector normal = ToSource( manifold.normal );
		for ( int p = 0; p < manifold.pointCount; ++p )
		{
			const b3ManifoldPoint &point = manifold.points[p];
			const int nVertexOwner = GetFeatureVertexOwner( point.featureId );
			if ( !bBall && nVertexOwner >= 0 )
				contact.m_nFirstSide = nVertexOwner;

			auto it = std::find_if( contact.m_Friction.begin(), contact.m_Friction.end(), [&]( const FrictionContact_t &friction ) { return friction.m_nFeatureId == point.featureId && friction.m_nTriangle == point.triangleIndex; } );
			const bool bTouching = point.totalNormalImpulse > 0.0f;
			const bool bCreated = it == contact.m_Friction.end();
			const b3Vec3 worldPoint = b3Add( center, point.anchorA );
			if ( bCreated )
			{
				if ( !bTouching )
					continue;

				FrictionContact_t &created = contact.m_Friction.emplace_back();
				created.m_nSequence = ++m_nContactSequence;
				created.m_nFeatureId = point.featureId;
				created.m_nTriangle = point.triangleIndex;
				created.m_nSide = bBall || nVertexOwner < 0 ? contact.m_nFirstSide : nVertexOwner;
				created.m_flEnergy = 0.0f;
				created.m_bSeparated = point.separation - point.normalVelocity * m_flTimestep >= PENETRATION_SEPARATION;
				created.m_bRemoved = false;

				const int nFaceShape = bBall ? ( IsBall( contact.m_pObjects[0] ) ? 1 : 0 ) : ( nVertexOwner >= 0 ? 1 - nVertexOwner : -1 );
				const Vector point0 = ToSource( worldPoint );
				int materials[2];
				for ( int j = 0; j < 2; ++j )
					materials[j] = GetContactMaterial( contact.m_pObjects[j], contact.m_pConvexes[j], point0, j == 0 ? normal : -normal, j == nFaceShape );
				created.m_Materials[0] = materials[created.m_nSide];
				created.m_Materials[1] = materials[1 - created.m_nSide];
				created.m_flFriction = MixFriction( GetMaterialFriction( created.m_Materials[0] ), 0, GetMaterialFriction( created.m_Materials[1] ), 0 );

				it = contact.m_Friction.end() - 1;
				AddFrictionPair( GetPairKey( contact.m_pObjects[0], contact.m_pObjects[1] ) );
			}

			FrictionContact_t &friction = *it;
			friction.m_Point = ToSource( worldPoint );
			friction.m_Normal = friction.m_nSide == 0 ? normal : -normal;
			friction.m_flSeparation = point.separation;
			friction.m_flNormalImpulse = point.totalNormalImpulse;
			if ( point.separation > IMPACT_SEPARATION )
				friction.m_bSeparated = true;

			if ( friction.m_bRemoved )
				continue;

			if ( bTouching )
			{
				const b3Pos position = b3ToPos( worldPoint );
				const b3Vec3 slip = b3Sub( b3Body_GetWorldPointVelocity( bodies[1], position ), b3Body_GetWorldPointVelocity( bodies[0], position ) );
				const b3Vec3 tangentSlip = b3MulSub( slip, b3Dot( slip, manifold.normal ), manifold.normal );
				friction.m_flEnergy += friction.m_flFriction * point.totalNormalImpulse * b3Length( tangentSlip );
			}

			if ( bCreated )
			{
				if ( contact.m_pObjects[1]->IsMoveable() )
					contact.m_pObjects[0]->SetTouchedDynamic();
				if ( contact.m_pObjects[0]->IsMoveable() )
					contact.m_pObjects[1]->SetTouchedDynamic();
				QueueContactEvent( ContactEvent_t::START_TOUCH, contact, friction, 0.0f );
			}

			if ( bTouching && friction.m_bSeparated && point.normalVelocity < 0.0f )
			{
				friction.m_bSeparated = false;
				const float flArrival = point.separation / -point.normalVelocity;
				if ( nImpact < 0 || flArrival < flImpactArrival )
				{
					nImpact = it - contact.m_Friction.begin();
					flImpactArrival = flArrival;
					flImpactSpeed = -point.normalVelocity;
				}
			}
		}
	}

	if ( nImpact >= 0 )
		QueueContactEvent( ContactEvent_t::IMPACT, contact, contact.m_Friction[nImpact], flImpactSpeed );

	for ( auto it = contact.m_Friction.begin(); it != contact.m_Friction.end(); )
	{
		if ( it->m_flSeparation < FRICTION_RELEASE_SEPARATION )
		{
			++it;
			continue;
		}

		ReleaseFriction( contact, *it, true );
		it = contact.m_Friction.erase( it );
	}
}

void CPhysicsEnvironment::ProcessContactEvents()
{
	const b3ContactEvents events = b3World_GetContactEvents( m_World );
	for ( int i = 0; i < events.beginCount; ++i )
	{
		const b3ContactBeginTouchEvent &event = events.beginEvents[i];
		Contact_t contact;
		contact.m_pObjects[0] = GetShapeObject( event.shapeIdA );
		contact.m_pObjects[1] = GetShapeObject( event.shapeIdB );
		contact.m_pConvexes[0] = static_cast<const CPhysConvex *>( b3Shape_GetUserData( event.shapeIdA ) );
		contact.m_pConvexes[1] = static_cast<const CPhysConvex *>( b3Shape_GetUserData( event.shapeIdB ) );
		contact.m_Shapes[0] = event.shapeIdA;
		contact.m_Shapes[1] = event.shapeIdB;
		contact.m_ContactId = event.contactId;
		contact.m_nScanStep = 0;
		contact.m_nFirstSide = GetInitialFirstSide( contact.m_pObjects[0], contact.m_pObjects[1] );
		contact.m_nDisableCount = 0;
		contact.m_bPenetrationRefused = false;

		auto previous = m_ProxyResetContacts.find( GetShapePairKey( event.shapeIdA, event.shapeIdB ) );
		if ( previous != m_ProxyResetContacts.end() )
		{
			Contact_t &reset = previous->second;
			const bool bSameOrder = IsSameShape( reset.m_Shapes[0], event.shapeIdA ) && IsSameShape( reset.m_Shapes[1], event.shapeIdB );
			const bool bSwapped = IsSameShape( reset.m_Shapes[0], event.shapeIdB ) && IsSameShape( reset.m_Shapes[1], event.shapeIdA );
			if ( bSameOrder || bSwapped )
			{
				const int nFlip = bSameOrder ? 0 : 1;
				contact.m_nFirstSide = reset.m_nFirstSide ^ nFlip;
				contact.m_nDisableCount = reset.m_nDisableCount;
				contact.m_bPenetrationRefused = reset.m_bPenetrationRefused;
				contact.m_Friction = std::move( reset.m_Friction );
				for ( FrictionContact_t &friction : contact.m_Friction )
					friction.m_nSide ^= nFlip;
				m_ProxyResetContacts.erase( previous );
			}
		}

		InsertContact( GetContactKey( event.contactId ), std::move( contact ) );
	}

	for ( int i = 0; i < events.endCount; ++i )
	{
		auto it = m_Contacts.find( GetContactKey( events.endEvents[i].contactId ) );
		if ( it == m_Contacts.end() )
			continue;

		RetireContact( it->second, true );
		EraseContact( it );
	}

	for ( const auto &[nKey, contact] : m_ProxyResetContacts )
		RetireContact( contact, true );
	m_ProxyResetContacts.clear();

	m_ScannedContacts.clear();
	for ( CPhysicsObject *pObject : m_SteppedObjects )
	{
		for ( uint64_t nKey : pObject->GetContactKeys() )
		{
			Contact_t &contact = m_Contacts.find( nKey )->second;
			if ( contact.m_nScanStep == m_nStepCount )
				continue;

			contact.m_nScanStep = m_nStepCount;
			m_ScannedContacts.push_back( nKey );
			UpdateFrictionContacts( contact );
		}
	}
}

void CPhysicsEnvironment::DispatchContactEvents()
{
	std::vector<ContactEvent_t> events;
	events.swap( m_ContactEvents );
	for ( const ContactEvent_t &event : events )
	{
		if ( event.m_nType == ContactEvent_t::IMPACT )
		{
			++m_Stats.impactCounter;
			if ( m_pCollisionEvents )
				ReportImpact( event );
		}
		else
		{
			ReportTouch( event.m_pObjects[0], event.m_pObjects[1], event.m_Point, event.m_Normal, event.m_nType == ContactEvent_t::START_TOUCH );
			if ( event.m_nType == ContactEvent_t::END_TOUCH )
				m_EndingContacts.pop_front();
		}
	}
	events.clear();
	m_ContactEvents.swap( events );
}

void CPhysicsEnvironment::ReportTrigger( CPhysicsObject *pTrigger, CPhysicsObject *pObject, bool bEnter )
{
	if ( !m_pCollisionEvents )
		return;

	if ( IPhysicsFluidController *pFluid = pTrigger->GetFluidController() )
	{
		if ( !( pObject->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
			return;

		if ( bEnter )
			m_pCollisionEvents->FluidStartTouch( pObject, pFluid );
		else
			m_pCollisionEvents->FluidEndTouch( pObject, pFluid );
		return;
	}

	if ( bEnter )
		m_pCollisionEvents->ObjectEnterTrigger( pTrigger, pObject );
	else
		m_pCollisionEvents->ObjectLeaveTrigger( pTrigger, pObject );
}

void CPhysicsEnvironment::ReportOverlap( TriggerOverlap_t &overlap, bool bEnter )
{
	if ( CPhysicsFluidController *pFluid = static_cast<CPhysicsFluidController *>( overlap.m_pTrigger->GetFluidController() ) )
	{
		if ( bEnter )
			pFluid->ObjectEntered( overlap.m_pObject );
		else
			pFluid->ObjectLeft( overlap.m_pObject );
	}
	ReportTrigger( overlap.m_pTrigger, overlap.m_pObject, bEnter );
}

void CPhysicsEnvironment::ReportPendingLeaves( CPhysicsObject *pObject )
{
	std::vector<TriggerOverlap_t> leaves;
	for ( auto it = m_TriggerOverlaps.begin(); it != m_TriggerOverlaps.end(); )
	{
		if ( it->second.m_pObject != pObject || !it->second.m_bLeavePending )
		{
			++it;
			continue;
		}

		leaves.push_back( it->second );
		it = m_TriggerOverlaps.erase( it );
	}

	for ( TriggerOverlap_t &overlap : leaves )
		ReportOverlap( overlap, false );
}

void CPhysicsEnvironment::ProcessSensorEvents()
{
	std::vector<uint64_t> pending;
	pending.swap( m_PendingTriggerEnters );
	for ( uint64_t nKey : pending )
	{
		auto it = m_TriggerOverlaps.find( nKey );
		if ( it == m_TriggerOverlaps.end() || it->second.m_bReported )
			continue;

		it->second.m_bReported = true;
		TriggerOverlap_t overlap = it->second;
		ReportOverlap( overlap, true );
	}

	const b3SensorEvents events = b3World_GetSensorEvents( m_World );
	for ( int i = 0; i < events.beginCount; ++i )
	{
		const b3SensorBeginTouchEvent &event = events.beginEvents[i];
		CPhysicsObject *pTrigger = GetShapeObject( event.sensorShapeId );
		CPhysicsObject *pObject = GetShapeObject( event.visitorShapeId );
		if ( !pTrigger || !pObject || pTrigger == pObject || ( pTrigger->IsStatic() && pObject->IsStatic() ) )
			continue;

		const uint64_t nKey = GetTriggerKey( pTrigger, pObject );
		TriggerOverlap_t &overlap = m_TriggerOverlaps.try_emplace( nKey, TriggerOverlap_t{ pTrigger, pObject, 0, false, false } ).first->second;
		if ( ++overlap.m_nCount != 1 )
			continue;

		if ( overlap.m_bLeavePending )
			overlap.m_bLeavePending = false;
		else
			m_PendingTriggerEnters.push_back( nKey );
	}

	std::vector<TriggerOverlap_t> leaves;
	for ( int i = 0; i < events.endCount; ++i )
	{
		const b3SensorEndTouchEvent &event = events.endEvents[i];
		if ( !b3Shape_IsValid( event.sensorShapeId ) || !b3Shape_IsValid( event.visitorShapeId ) )
			continue;

		auto it = m_TriggerOverlaps.find( GetTriggerKey( GetShapeObject( event.sensorShapeId ), GetShapeObject( event.visitorShapeId ) ) );
		if ( it == m_TriggerOverlaps.end() || --it->second.m_nCount > 0 )
			continue;

		if ( it->second.m_bReported && it->second.m_pObject->IsAsleep() )
		{
			it->second.m_bLeavePending = true;
			continue;
		}

		if ( it->second.m_bReported )
			leaves.push_back( it->second );
		m_TriggerOverlaps.erase( it );
	}

	for ( TriggerOverlap_t &overlap : leaves )
		ReportOverlap( overlap, false );
}

void CPhysicsEnvironment::RemoveTriggerOverlaps( CPhysicsObject *pTrigger )
{
	if ( CPhysicsFluidController *pFluid = static_cast<CPhysicsFluidController *>( pTrigger->GetFluidController() ) )
		pFluid->RemoveAllObjects();

	std::vector<CPhysicsObject *> visitors;
	for ( auto it = m_TriggerOverlaps.begin(); it != m_TriggerOverlaps.end(); )
	{
		if ( it->second.m_pTrigger != pTrigger )
		{
			++it;
			continue;
		}

		if ( it->second.m_bReported )
			visitors.push_back( it->second.m_pObject );
		it = m_TriggerOverlaps.erase( it );
	}

	for ( CPhysicsObject *pObject : visitors )
		ReportTrigger( pTrigger, pObject, false );
}

void CPhysicsEnvironment::RemoveVisitorOverlaps( CPhysicsObject *pObject )
{
	std::vector<TriggerOverlap_t> leaves;
	for ( auto it = m_TriggerOverlaps.begin(); it != m_TriggerOverlaps.end(); )
	{
		if ( it->second.m_pObject != pObject )
		{
			++it;
			continue;
		}

		if ( it->second.m_bReported )
			leaves.push_back( it->second );
		it = m_TriggerOverlaps.erase( it );
	}

	for ( TriggerOverlap_t &overlap : leaves )
		ReportOverlap( overlap, false );
}

void CPhysicsEnvironment::ProcessPenetration()
{
	if ( !m_pCollisionSolver )
		return;

	for ( uint64_t nKey : m_ScannedContacts )
	{
		auto it = m_Contacts.find( nKey );
		if ( it == m_Contacts.end() )
			continue;

		const Contact_t &contact = it->second;
		CPhysicsObject *pObject0 = contact.m_pObjects[0], *pObject1 = contact.m_pObjects[1];
		if ( contact.m_bPenetrationRefused || ( pObject0->GetSleepState() != CPhysicsObject::AWAKE && pObject1->GetSleepState() != CPhysicsObject::AWAKE ) )
			continue;

		if ( ( pObject0->GetCallbackFlags() | pObject1->GetCallbackFlags() ) & CALLBACK_MARKED_FOR_DELETE )
			continue;

		if ( std::none_of( contact.m_Friction.begin(), contact.m_Friction.end(), []( const FrictionContact_t &friction ) { return !friction.m_bRemoved && friction.m_flSeparation < PENETRATION_SEPARATION; } ) )
			continue;

		const uint64_t nPairKey = GetPairKey( pObject0, pObject1 );
		if ( pObject0->IsMoveable() && pObject1->IsMoveable() )
		{
			if ( std::find( m_PenetrationPairs.begin(), m_PenetrationPairs.end(), nPairKey ) != m_PenetrationPairs.end() )
				continue;
			m_PenetrationPairs.push_back( nPairKey );
		}

		if ( m_pCollisionSolver->ShouldSolvePenetration( pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData(), m_flTimestep ) != 0 )
			continue;

		it = m_Contacts.find( nKey );
		if ( it == m_Contacts.end() )
			continue;

		it->second.m_bPenetrationRefused = true;
		++it->second.m_nDisableCount;
		DisablePair( nPairKey );
	}
}

const std::vector<CPhysicsEnvironment::FrictionRef_t> &CPhysicsEnvironment::GetSortedContacts( const CPhysicsObject *pObject ) const
{
	std::vector<FrictionRef_t> &contacts = m_SortedContacts;
	contacts.clear();
	for ( uint64_t nKey : pObject->GetContactKeys() )
	{
		const Contact_t &contact = m_Contacts.find( nKey )->second;
		for ( const FrictionContact_t &friction : contact.m_Friction )
		{
			if ( !friction.m_bRemoved )
				contacts.push_back( { &contact, &friction } );
		}
	}
	for ( const Contact_t &contact : m_EndingContacts )
	{
		if ( contact.m_pObjects[0] == pObject || contact.m_pObjects[1] == pObject )
			contacts.push_back( { &contact, &contact.m_Friction.front() } );
	}
	std::sort( contacts.begin(), contacts.end(), []( const FrictionRef_t &a, const FrictionRef_t &b ) { return a.m_pFriction->m_nSequence > b.m_pFriction->m_nSequence; } );
	return contacts;
}

const std::vector<CPhysicsEnvironment::ContactNormal_t> &CPhysicsEnvironment::GetContactNormals( const CPhysicsObject *pObject )
{
	m_ContactNormals.clear();
	for ( const FrictionRef_t &ref : GetSortedContacts( pObject ) )
	{
		const bool bFirst = ref.m_pContact->m_pObjects[ref.m_pFriction->m_nSide] == pObject;
		ContactNormal_t &entry = m_ContactNormals.emplace_back();
		entry.m_pOther = ref.m_pContact->m_pObjects[ref.m_pContact->m_pObjects[0] == pObject ? 1 : 0];
		entry.m_Normal = NormalizeSurfaceNormal( bFirst ? ref.m_pFriction->m_Normal : -ref.m_pFriction->m_Normal );
		entry.m_flNormalForce = ref.m_pFriction->m_flNormalImpulse * m_flInverseTimestep;
	}
	return m_ContactNormals;
}

IPhysicsFrictionSnapshot *CPhysicsEnvironment::CreateFrictionSnapshot( CPhysicsObject *pObject )
{
	const std::vector<FrictionRef_t> &touching = GetSortedContacts( pObject );

	std::vector<CPhysicsFrictionSnapshot::Contact_t> entries;
	entries.reserve( touching.size() );
	for ( const FrictionRef_t &ref : touching )
	{
		const Contact_t &contact = *ref.m_pContact;
		const FrictionContact_t &friction = *ref.m_pFriction;
		const int nSide = contact.m_pObjects[friction.m_nSide] == pObject ? 0 : 1;

		CPhysicsFrictionSnapshot::Contact_t &entry = entries.emplace_back();
		entry.m_nKey = friction.m_nSequence;
		entry.m_pOther = contact.m_pObjects[contact.m_pObjects[0] == pObject ? 1 : 0];
		entry.m_Point = friction.m_Point;
		entry.m_Normal = NormalizeSurfaceNormal( nSide == 0 ? friction.m_Normal : -friction.m_Normal );
		entry.m_Materials[0] = friction.m_Materials[nSide];
		entry.m_Materials[1] = friction.m_Materials[1 - nSide];
		entry.m_flFriction = friction.m_flFriction;
		entry.m_flNormalForce = friction.m_flNormalImpulse * m_flInverseTimestep;
		entry.m_flEnergy = friction.m_flEnergy;
	}

	return new CPhysicsFrictionSnapshot( pObject, std::move( entries ) );
}

bool CPhysicsEnvironment::RefreshFrictionContact( const CPhysicsObject *pObject, uint64_t nSequence, Vector &point, Vector &normal ) const
{
	for ( uint64_t nKey : pObject->GetContactKeys() )
	{
		const Contact_t &contact = m_Contacts.find( nKey )->second;
		for ( const FrictionContact_t &friction : contact.m_Friction )
		{
			if ( friction.m_nSequence != nSequence )
				continue;

			point = friction.m_Point;
			normal = NormalizeSurfaceNormal( contact.m_pObjects[friction.m_nSide] == pObject ? friction.m_Normal : -friction.m_Normal );
			return true;
		}
	}
	return false;
}

bool CPhysicsEnvironment::GetContactPoint( const CPhysicsObject *pObject, Vector *pContactPoint, IPhysicsObject **ppContactObject ) const
{
	const std::vector<FrictionRef_t> &touching = GetSortedContacts( pObject );
	if ( touching.empty() )
		return false;

	const Contact_t &contact = *touching.front().m_pContact;
	if ( pContactPoint )
		*pContactPoint = touching.front().m_pFriction->m_Point;
	if ( ppContactObject )
		*ppContactObject = contact.m_pObjects[contact.m_pObjects[0] == pObject ? 1 : 0];
	return true;
}

bool CPhysicsEnvironment::GetStrongestContact( const CPhysicsObject *pObject, Vector &point, int &nSurface ) const
{
	float flBestForce = -1.0f;
	bool bContact = false;
	for ( const FrictionRef_t &ref : GetSortedContacts( pObject ) )
	{
		const float flForce = ref.m_pFriction->m_flNormalImpulse * m_flInverseTimestep;
		if ( !( flForce > flBestForce ) )
			continue;

		const int nSide = ref.m_pContact->m_pObjects[ref.m_pFriction->m_nSide] == pObject ? 0 : 1;
		flBestForce = flForce;
		point = ref.m_pFriction->m_Point;
		nSurface = ref.m_pFriction->m_Materials[1 - nSide];
		bContact = true;
	}
	return bContact;
}

bool CPhysicsEnvironment::HasFrictionContact( const CPhysicsObject *pObject ) const
{
	for ( uint64_t nKey : pObject->GetContactKeys() )
	{
		for ( const FrictionContact_t &friction : m_Contacts.find( nKey )->second.m_Friction )
		{
			if ( !friction.m_bRemoved )
				return true;
		}
	}
	return false;
}

void CPhysicsEnvironment::DispatchFriction()
{
	const float flLastStepTime = static_cast<float>( m_flLastStepTime );
	const float flElapsed = flLastStepTime - m_flLastFrictionTime;
	if ( flElapsed < m_flTimestep )
		return;

	const float flInverse = flElapsed != 0.0f ? 1.0f / flElapsed : 0.0f;
	m_flLastFrictionTime = flLastStepTime;

	m_FrictionDispatches.clear();
	std::vector<std::pair<FrictionContact_t *, const Contact_t *>> frictions;
	for ( CPhysicsObject *pObject : m_ActiveObjects )
	{
		if ( !( pObject->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) )
			continue;

		frictions.clear();
		for ( uint64_t nKey : pObject->GetContactKeys() )
		{
			Contact_t &contact = m_Contacts.find( nKey )->second;
			if ( !( contact.m_pObjects[contact.m_pObjects[0] == pObject ? 1 : 0]->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) )
				continue;

			for ( FrictionContact_t &friction : contact.m_Friction )
			{
				if ( !friction.m_bRemoved )
					frictions.emplace_back( &friction, &contact );
			}
		}
		std::sort( frictions.begin(), frictions.end(), []( const auto &a, const auto &b ) { return a.first->m_nSequence > b.first->m_nSequence; } );

		for ( const auto &[pFriction, pContact] : frictions )
		{
			if ( pFriction->m_flEnergy == 0.0f )
				continue;

			const float flEnergy = pFriction->m_flEnergy * flInverse * pObject->GetInvMass();
			pFriction->m_flEnergy = 0.0f;
			if ( !( flEnergy > FRICTION_ENERGY_THRESHOLD * IVP_ENERGY_TO_SOURCE ) )
				continue;

			const int nSide = pContact->m_pObjects[pFriction->m_nSide] == pObject ? 0 : 1;
			const Vector normal = NormalizeSurfaceNormal( nSide == 0 ? pFriction->m_Normal : -pFriction->m_Normal );
			m_FrictionDispatches.push_back( { pObject, flEnergy, pFriction->m_Materials[1 - nSide], pFriction->m_Point, normal } );
		}
	}

	m_bDispatchingFriction = true;
	for ( const FrictionDispatch_t &dispatch : m_FrictionDispatches )
	{
		if ( dispatch.m_pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE )
			continue;

		CCollisionData data( dispatch.m_Normal, dispatch.m_Point, Vector( 0.0f, 0.0f, 0.0f ) );
		m_pCollisionEvents->Friction( dispatch.m_pObject, dispatch.m_flEnergy, dispatch.m_pObject->GetMaterialIndex(), dispatch.m_nSurfaceHit, &data );
	}
	m_bDispatchingFriction = false;

	if ( !m_bDeleteQueue )
		CleanupDeleteList();
}

void CPhysicsEnvironment::DebugCheckContacts()
{
	for ( const auto &[nKey, contact] : m_Contacts )
	{
		CPhysicsObject *pObject0 = contact.m_pObjects[contact.m_nFirstSide], *pObject1 = contact.m_pObjects[1 - contact.m_nFirstSide];
		if ( !EvaluateCollision( pObject0, pObject1 ) )
			Msg( "Changed collision rules for %s vs. %s without calling recheck!\n", pObject0->GetName(), pObject1->GetName() );
	}
}
