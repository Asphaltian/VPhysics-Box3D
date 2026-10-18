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
#include <tuple>

static constexpr float DEFAULT_TIMESTEP = 1.0f / 66.0f;
static constexpr int PARALLEL_OBJECT_COUNT = 48;
static constexpr float IVP_DEFAULT_GRAVITY = 9.83f;
static constexpr float MAX_SIMULATE_DELTA = 1.0f;
static constexpr double MIN_SIMULATE_DELTA = 1e-4;
static constexpr double MAX_STEP_DELTA = 0.1;
static constexpr float ALIGNED_STEP_SCALE = 1.99999f;
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
static constexpr float PENETRATION_SEPARATION = -2.0f * SHAPE_INFLATION;
static constexpr float MESH_COPLANAR_COSINE = 0.996f;
static constexpr float FRICTION_RELEASE_SEPARATION = 4.5f * COLLISION_TOLERANCE * IVP_TO_SOURCE - 2.0f * SHAPE_INFLATION;
static constexpr float FRICTION_SPAN_TOLERANCE = 1e-6f * IVP_TO_SOURCE * IVP_TO_SOURCE;
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
	CPhysicsEnvironment *pEnvironment = static_cast<CPhysicsEnvironment *>( pContext );
	return !pEnvironment->IsShapePairHeld( shapeA, shapeB ) && pEnvironment->IsCollisionAllowed( GetShapeObject( shapeA ), GetShapeObject( shapeB ) );
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
, m_TaskPool( GetPhysicalProcessorCount() )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ToBox3D( GetGravityVector() );
	worldDef.enableSleep = false;
	worldDef.workerCount = m_TaskPool.GetThreadCount();
	worldDef.enqueueTask = CTaskPool::EnqueueTask;
	worldDef.finishTask = CTaskPool::FinishTask;
	worldDef.userTaskContext = &m_TaskPool;
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
	m_TaskPool.SetSerial( static_cast<int>( m_SimulatedObjects.size() ) < PARALLEL_OBJECT_COUNT );

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
		pObject->UpdateFastRotation();
		pObject->CapturePreStepVelocity( m_nStepCount );
	}
	ResolveSteppedFilterDecisions();
	DetectPendingPenetrationHolds();
	ApplyPenetrationResets();

	m_TaskPool.Reset();
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
		pObject->ClampVelocity( false );
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

void CPhysicsEnvironment::UpdateWorldMaterials()
{
	for ( IPhysicsObject *pObject : m_Objects )
		static_cast<CPhysicsObject *>( pObject )->UpdateWorldMaterials();
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

b3AABB CPhysicsEnvironment::GetFilterQueryBounds( const CPhysicsObject *pObject, const b3AABB &bounds ) const
{
	const b3Vec3 extent = b3Sub( bounds.upperBound, bounds.lowerBound );
	const float flMargin = B3_AABB_MARGIN_FRACTION * std::max( std::max( extent.x, extent.y ), extent.z ) + b3Length( b3Body_GetLinearVelocity( pObject->GetBody() ) ) * m_flTimestep;
	const b3Vec3 margin = { flMargin, flMargin, flMargin };
	return { b3Sub( bounds.lowerBound, margin ), b3Add( bounds.upperBound, margin ) };
}

void CPhysicsEnvironment::QueryFilterPartners( CPhysicsObject *pObject, const b3AABB &query )
{
	static thread_local std::vector<CPhysicsObject *> s_Candidates;
	std::vector<CPhysicsObject *> candidates;
	candidates.swap( s_Candidates );
	candidates.clear();
	b3World_OverlapAABB(
		m_World, query, b3DefaultQueryFilter(),
		[]( b3ShapeId shape, void *pContext ) {
			static_cast<std::vector<CPhysicsObject *> *>( pContext )->push_back( GetShapeObject( shape ) );
			return true;
		},
		&candidates );

	for ( CPhysicsObject *pPartner : candidates )
	{
		if ( !pPartner || pPartner == pObject || !pPartner->IsCollisionEnabled() )
			continue;

		if ( pObject->GetSleepState() != CPhysicsObject::AWAKE && pPartner->GetSleepState() != CPhysicsObject::AWAKE )
			continue;

		bool bCollide;
		if ( !pObject->FindFilterDecision( pPartner, bCollide ) )
			ShouldCollide( pObject, pPartner );
	}
	candidates.swap( s_Candidates );
}

void CPhysicsEnvironment::ResolveFilterDecisions( CPhysicsObject *pObject, bool bForce )
{
	if ( !m_pCollisionSolver || !pObject->IsCollisionEnabled() )
		return;

	const b3AABB bounds = b3Body_ComputeAABB( pObject->GetBody() );
	if ( !bForce && pObject->HasFilterBounds() && b3AABB_Contains( pObject->GetFilterBounds(), bounds ) )
		return;

	const b3AABB query = GetFilterQueryBounds( pObject, bounds );
	if ( pObject->GetSleepState() == CPhysicsObject::AWAKE )
		pObject->SetFilterBounds( query );
	QueryFilterPartners( pObject, query );
}

void CPhysicsEnvironment::ResolveSteppedFilterDecisions()
{
	if ( !m_pCollisionSolver )
		return;

	m_FilterQueryObjects.clear();
	for ( CPhysicsObject *pObject : m_SimulatedObjects )
	{
		if ( !pObject->IsCollisionEnabled() )
			continue;

		const b3AABB bounds = b3Body_ComputeAABB( pObject->GetBody() );
		if ( pObject->HasFilterBounds() && b3AABB_Contains( pObject->GetFilterBounds(), bounds ) )
			continue;

		pObject->SetFilterBounds( GetFilterQueryBounds( pObject, bounds ) );
		m_FilterQueryObjects.push_back( pObject );
	}

	const int nThreads = m_TaskPool.GetThreadCount();
	m_ThreadCandidates.resize( nThreads );
	m_ThreadUndecided.resize( nThreads );
	for ( std::vector<CPhysicsObject *> &undecided : m_ThreadUndecided )
		undecided.clear();
	m_FilterQueryResults.resize( m_FilterQueryObjects.size() );
	m_TaskPool.ParallelFor( static_cast<int>( m_FilterQueryObjects.size() ), [this]( int nIndex, int nThread ) {
		CPhysicsObject *pObject = m_FilterQueryObjects[nIndex];
		std::vector<CPhysicsObject *> &candidates = m_ThreadCandidates[nThread];
		std::vector<CPhysicsObject *> &undecided = m_ThreadUndecided[nThread];
		candidates.clear();
		b3World_OverlapAABB(
			m_World, pObject->GetFilterBounds(), b3DefaultQueryFilter(),
			[]( b3ShapeId shape, void *pContext ) {
				static_cast<std::vector<CPhysicsObject *> *>( pContext )->push_back( GetShapeObject( shape ) );
				return true;
			},
			&candidates );

		const size_t nBegin = undecided.size();
		for ( CPhysicsObject *pPartner : candidates )
		{
			if ( !pPartner || pPartner == pObject || !pPartner->IsCollisionEnabled() )
				continue;

			bool bCollide;
			if ( !pObject->FindFilterDecision( pPartner, bCollide ) )
				undecided.push_back( pPartner );
		}
		m_FilterQueryResults[nIndex] = { nThread, nBegin, undecided.size() };
	} );

	for ( size_t i = 0; i < m_FilterQueryObjects.size(); ++i )
	{
		CPhysicsObject *pObject = m_FilterQueryObjects[i];
		const FilterQueryResult_t &result = m_FilterQueryResults[i];
		for ( size_t j = result.m_nBegin; j < result.m_nEnd; ++j )
		{
			CPhysicsObject *pPartner = m_ThreadUndecided[result.m_nThread][j];
			bool bCollide;
			if ( !pObject->FindFilterDecision( pPartner, bCollide ) )
				ShouldCollide( pObject, pPartner );
		}
	}
}

void CPhysicsEnvironment::ResolvePendingFilterPairs()
{
	if ( m_PendingFilterPairs.empty() )
		return;

	std::vector<std::pair<CPhysicsObject *, CPhysicsObject *>> pairs;
	pairs.swap( m_PendingFilterPairs );
	std::sort( pairs.begin(), pairs.end(), []( const auto &a, const auto &b ) {
		const uint32_t a0 = a.first->GetId(), a1 = a.second->GetId(), b0 = b.first->GetId(), b1 = b.second->GetId();
		return std::make_tuple( std::min( a0, a1 ), std::max( a0, a1 ), a0 ) < std::make_tuple( std::min( b0, b1 ), std::max( b0, b1 ), b0 );
	} );
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

		for ( FrictionContact_t *pFriction : it->second.m_Friction )
			UnlinkFriction( pFriction );
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
		const bool bActive = std::any_of( contact.m_Friction.begin(), contact.m_Friction.end(), []( const FrictionContact_t *pFriction ) { return !pFriction->m_bRemoved; } );
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

		const bool bMesh = contact.m_pMeshes[0] || contact.m_pMeshes[1];
		for ( FrictionContact_t *pFriction : contact.m_Friction )
		{
			if ( pFriction->m_bRemoved )
				continue;

			pFriction->m_bRemoved = true;
			UnlinkFriction( pFriction );
			if ( !bMesh )
			{
				DisablePair( nPairKey );
				++contact.m_nDisableCount;
			}
			removed.push_back( { pFriction->m_nSequence, { pFriction->m_pObjects[0], pFriction->m_pObjects[1] }, pFriction->m_Point, pFriction->m_Normal } );
		}

		if ( bMesh )
		{
			std::erase_if( contact.m_Friction, [&]( FrictionContact_t *pFriction ) {
				if ( !pFriction->m_bRemoved )
					return false;
				RemoveFrictionPair( nPairKey );
				delete pFriction;
				return true;
			} );
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
	std::erase( m_PendingHoldChecks, pObject );
	std::erase( m_PenetrationResets, pObject );
	std::erase_if( m_PenetrationHolds, [&]( const PenetrationHold_t &hold ) {
		const bool bMember = hold.m_pObject == pObject || ( b3Shape_IsValid( hold.m_MeshShape ) && GetShapeObject( hold.m_MeshShape ) == pObject );
		if ( bMember )
			m_HeldShapePairs.erase( GetShapePairKey( hold.m_Shape, hold.m_MeshShape ) );
		return bMember;
	} );

	const auto release = [this]( const Contact_t &contact ) {
		for ( FrictionContact_t *pFriction : contact.m_Friction )
			ReleaseFriction( contact, pFriction, false );
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
	if ( contact.m_bDetached )
		m_DetachedContacts.erase( GetShapePairKey( contact.m_Shapes[0], contact.m_Shapes[1] ) );
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
	for ( FrictionContact_t *pFriction : contact.m_Friction )
		ReleaseFriction( contact, pFriction, bReport );

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

void CPhysicsEnvironment::LinkFriction( FrictionContact_t *pFriction )
{
	if ( pFriction->m_bLinked )
		return;

	pFriction->m_bLinked = true;
	for ( int i = 0; i < 2; ++i )
	{
		CPhysicsObject *pObject = pFriction->m_pObjects[i];
		FrictionContact_t *pPrev = nullptr, *pNext = pObject->GetFrictionHead();
		while ( pNext && pNext->m_nSequence > pFriction->m_nSequence )
		{
			pPrev = pNext;
			pNext = pNext->m_pNext[pNext->m_pObjects[0] == pObject ? 0 : 1];
		}

		pFriction->m_pPrev[i] = pPrev;
		pFriction->m_pNext[i] = pNext;
		if ( pPrev )
			pPrev->m_pNext[pPrev->m_pObjects[0] == pObject ? 0 : 1] = pFriction;
		else
			pObject->SetFrictionHead( pFriction );
		if ( pNext )
			pNext->m_pPrev[pNext->m_pObjects[0] == pObject ? 0 : 1] = pFriction;
	}
}

void CPhysicsEnvironment::UnlinkFriction( FrictionContact_t *pFriction )
{
	if ( !pFriction->m_bLinked )
		return;

	pFriction->m_bLinked = false;
	for ( int i = 0; i < 2; ++i )
	{
		CPhysicsObject *pObject = pFriction->m_pObjects[i];
		FrictionContact_t *pPrev = pFriction->m_pPrev[i], *pNext = pFriction->m_pNext[i];
		if ( pPrev )
			pPrev->m_pNext[pPrev->m_pObjects[0] == pObject ? 0 : 1] = pNext;
		else
			pObject->SetFrictionHead( pNext );
		if ( pNext )
			pNext->m_pPrev[pNext->m_pObjects[0] == pObject ? 0 : 1] = pPrev;
	}
}

void CPhysicsEnvironment::ReleaseFriction( const Contact_t &contact, FrictionContact_t *pFriction, bool bReport )
{
	RemoveFrictionPair( GetPairKey( contact.m_pObjects[0], contact.m_pObjects[1] ) );
	if ( !pFriction->m_bRemoved && bReport )
	{
		QueueContactEvent( ContactEvent_t::END_TOUCH, contact, *pFriction, 0.0f );
		m_ContactEvents.back().m_pEnding = pFriction;
		pFriction->m_bEnding = true;
		LinkFriction( pFriction );
		return;
	}

	UnlinkFriction( pFriction );
	delete pFriction;
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
	event.m_pEnding = nullptr;
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

int CPhysicsEnvironment::GetContactMaterial( const Contact_t &contact, int nSide, int nTriangle, const Vector &point, const Vector &outwardNormal, bool bFaceFeature ) const
{
	const CPhysicsObject *pObject = contact.m_pObjects[nSide];
	if ( !bFaceFeature )
		return pObject->GetMaterialIndex();

	int nMaterial = 0;
	if ( const CollideMesh_t *pMesh = contact.m_pMeshes[nSide] )
	{
		const CollideMesh_t::Source_t &source = pMesh->m_Sources[nTriangle];
		nMaterial = pObject->GetCollide()->GetConvexes()[source.m_nConvex]->GetTriangles()[source.m_nTriangle].m_nMaterialIndex;
	}
	else if ( const CPhysConvex *pConvex = contact.m_pConvexes[nSide] )
	{
		const b3WorldTransform transform = b3Body_GetTransform( pObject->GetBody() );
		const b3Vec3 localNormal = b3InvRotateVector( transform.q, ToBox3D( outwardNormal ) );
		const b3Vec3 localPoint = b3InvRotateVector( transform.q, b3Sub( ToBox3D( point ), b3ToVec3( transform.p ) ) );
		nMaterial = pConvex->FindTriangleMaterial( localNormal, localPoint );
	}
	return nMaterial ? g_PhysicsSurfaceProps.GetWorldMaterialIndex( nMaterial ) : pObject->GetMaterialIndex();
}

int CPhysicsEnvironment::GetPointVertexOwner( const Contact_t &contact, const b3Manifold &manifold, const b3ManifoldPoint &point )
{
	static constexpr uint8_t FLAT_EDGE_FLAGS[3] = { b3_flatEdge1, b3_flatEdge2, b3_flatEdge3 };

	const int nOwner = GetFeatureVertexOwner( point.featureId );
	const int nMesh = contact.m_pMeshes[0] ? 0 : ( contact.m_pMeshes[1] ? 1 : -1 );
	if ( nMesh < 0 || nOwner == 1 - nMesh )
		return nOwner;

	const b3MeshData *pData = contact.m_pMeshes[nMesh]->m_pData;
	const uint8_t nFlags = b3GetMeshFlags( pData )[point.triangleIndex];
	const uint32_t features[2][2] = { { ( point.featureId >> 24 ) & 0xFF, ( point.featureId >> 16 ) & 0xFF }, { ( point.featureId >> 8 ) & 0xFF, point.featureId & 0xFF } };
	bool bFlat = true;
	for ( const auto &feature : features )
	{
		if ( feature[0] == static_cast<uint32_t>( nMesh ) && ( feature[1] > 2 || ( nFlags & FLAT_EDGE_FLAGS[feature[1]] ) != FLAT_EDGE_FLAGS[feature[1]] ) )
			bFlat = false;
	}
	if ( bFlat )
		return 1 - nMesh;

	if ( nOwner == nMesh )
		return nOwner;

	const b3MeshTriangle &triangle = b3GetMeshTriangles( pData )[point.triangleIndex];
	const b3Vec3 *pVertices = b3GetMeshVertices( pData );
	const b3Vec3 a = pVertices[triangle.index1];
	const b3Vec3 localNormal = b3Normalize( b3Cross( b3Sub( pVertices[triangle.index2], a ), b3Sub( pVertices[triangle.index3], a ) ) );
	const b3Vec3 normal = b3RotateVector( b3Body_GetRotation( contact.m_pObjects[nMesh]->GetBody() ), localNormal );
	const float flFacing = b3Dot( nMesh == 0 ? manifold.normal : b3Neg( manifold.normal ), normal );
	return flFacing > MESH_COPLANAR_COSINE ? 1 - nMesh : nOwner;
}

int CPhysicsEnvironment::GetContactLedge( const Contact_t &contact, int nTriangle )
{
	for ( const CollideMesh_t *pMesh : contact.m_pMeshes )
	{
		if ( pMesh )
			return pMesh->m_Sources[nTriangle].m_nConvex;
	}
	return -1;
}

int CPhysicsEnvironment::GetPointMaterials( const Contact_t &contact, const b3Manifold &manifold, const b3ManifoldPoint &point, b3Vec3 worldPoint, bool bBall, int materials[2] ) const
{
	const int nVertexOwner = GetPointVertexOwner( contact, manifold, point );
	const int nSide = bBall || nVertexOwner < 0 ? contact.m_nFirstSide : nVertexOwner;
	const int nFaceShape = bBall ? ( IsBall( contact.m_pObjects[0] ) ? 1 : 0 ) : ( nVertexOwner >= 0 ? 1 - nVertexOwner : -1 );
	const Vector normal = ToSource( manifold.normal );
	const Vector point0 = ToSource( worldPoint );
	int sideMaterials[2];
	for ( int j = 0; j < 2; ++j )
		sideMaterials[j] = GetContactMaterial( contact, j, point.triangleIndex, point0, j == 0 ? normal : -normal, j == nFaceShape );
	materials[0] = sideMaterials[nSide];
	materials[1] = sideMaterials[1 - nSide];
	return nSide;
}

FrictionContact_t *CPhysicsEnvironment::CreateFrictionContact( Contact_t &contact, const b3Manifold &manifold, const b3ManifoldPoint &point, b3Vec3 worldPoint, bool bBall )
{
	const Vector normal = ToSource( manifold.normal );
	const Vector point0 = ToSource( worldPoint );
	FrictionContact_t &created = *contact.m_Friction.emplace_back( new FrictionContact_t{} );
	created.m_nSequence = ++m_nContactSequence;
	created.m_nFeatureId = point.featureId;
	created.m_nTriangle = point.triangleIndex;
	created.m_nSide = GetPointMaterials( contact, manifold, point, worldPoint, bBall, created.m_Materials );
	created.m_flFriction = MixFriction( GetMaterialFriction( created.m_Materials[0] ), 0, GetMaterialFriction( created.m_Materials[1] ), 0 );
	created.m_pObjects[0] = contact.m_pObjects[created.m_nSide];
	created.m_pObjects[1] = contact.m_pObjects[1 - created.m_nSide];
	created.m_Point = point0;
	created.m_Normal = created.m_nSide == 0 ? normal : -normal;
	created.m_flSeparation = point.separation;
	LinkFriction( &created );
	AddFrictionPair( GetPairKey( contact.m_pObjects[0], contact.m_pObjects[1] ) );

	if ( contact.m_pObjects[1]->IsMoveable() )
		contact.m_pObjects[0]->SetTouchedDynamic();
	if ( contact.m_pObjects[0]->IsMoveable() )
		contact.m_pObjects[1]->SetTouchedDynamic();
	QueueContactEvent( ContactEvent_t::START_TOUCH, contact, created, 0.0f );
	return &created;
}

void CPhysicsEnvironment::SelectLedgeImpacts()
{
	std::vector<ContactImpact_t> &impacts = m_ContactImpacts;
	std::vector<ContactImpact_t> selected;
	for ( const ContactImpact_t &impact : impacts )
	{
		const ManifoldPoint_t &entry = m_ManifoldPoints[impact.m_nPoint];
		auto earliest = std::find_if( selected.begin(), selected.end(), [&]( const ContactImpact_t &other ) { return m_ManifoldPoints[other.m_nPoint].m_nLedge == entry.m_nLedge; } );
		if ( earliest == selected.end() )
		{
			selected.push_back( impact );
			continue;
		}

		const b3ManifoldPoint &point = *entry.m_pPoint;
		const b3ManifoldPoint &other = *m_ManifoldPoints[earliest->m_nPoint].m_pPoint;
		if ( point.separation / -point.normalVelocity < other.separation / -other.normalVelocity )
			*earliest = impact;
	}
	impacts.swap( selected );
}

void CPhysicsEnvironment::UpdateFrictionContacts( Contact_t &contact )
{
	const b3ContactData data = b3Contact_GetData( contact.m_ContactId );
	const bool bBall = IsBall( contact.m_pObjects[0] ) || IsBall( contact.m_pObjects[1] );
	b3Vec3 centers[2], velocities[2], angularVelocities[2];
	b3Quat rotations[2];
	for ( int i = 0; i < 2; ++i )
	{
		const b3BodyId body = contact.m_pObjects[i]->GetBody();
		centers[i] = b3ToVec3( b3Body_GetWorldCenter( body ) );
		rotations[i] = b3Body_GetRotation( body );
		velocities[i] = b3Body_GetLinearVelocity( body );
		angularVelocities[i] = b3Body_GetAngularVelocity( body );
	}

	contact.m_flSpanCapacity = 0.0f;
	contact.m_flMinSeparation = FLT_MAX;
	std::vector<ManifoldPoint_t> &points = m_ManifoldPoints;
	std::vector<LedgeContact_t> &ledges = m_LedgeContacts;
	std::vector<ContactImpact_t> &impacts = m_ContactImpacts;
	points.clear();
	ledges.clear();
	impacts.clear();
	for ( int m = 0; m < data.manifoldCount; ++m )
	{
		const b3Manifold &manifold = data.manifolds[m];
		for ( int p = 0; p < manifold.pointCount; ++p )
		{
			const b3ManifoldPoint &point = manifold.points[p];
			const int nVertexOwner = GetPointVertexOwner( contact, manifold, point );
			if ( !bBall && nVertexOwner >= 0 )
				contact.m_nFirstSide = nVertexOwner;
			contact.m_flMinSeparation = std::min( contact.m_flMinSeparation, point.separation );

			auto state = std::find_if( contact.m_Points.begin(), contact.m_Points.end(), [&]( const ContactPoint_t &tracked ) { return tracked.m_nFeatureId == point.featureId && tracked.m_nTriangle == point.triangleIndex; } );
			if ( state == contact.m_Points.end() )
				state = contact.m_Points.insert( contact.m_Points.end(), { point.featureId, point.triangleIndex, 0, point.separation - point.normalVelocity * m_flTimestep > IMPACT_SEPARATION } );
			state->m_nStep = m_nStepCount;
			if ( point.separation > IMPACT_SEPARATION )
				state->m_bSeparated = true;

			const int nLedge = GetContactLedge( contact, point.triangleIndex );
			auto ledge = std::find_if( ledges.begin(), ledges.end(), [nLedge]( const LedgeContact_t &entry ) { return entry.m_nLedge == nLedge; } );
			if ( ledge == ledges.end() )
				ledge = ledges.insert( ledges.end(), { nLedge, -1 } );

			const ptrdiff_t nIndex = static_cast<ptrdiff_t>( points.size() );
			points.push_back( { &manifold, &point, b3Add( centers[0], point.anchorA ), nullptr, nLedge } );
			if ( point.totalNormalImpulse <= 0.0f )
				continue;

			if ( point.separation >= PENETRATION_SEPARATION && ( ledge->m_nDeepest < 0 || point.separation < points[ledge->m_nDeepest].m_pPoint->separation ) )
				ledge->m_nDeepest = nIndex;

			if ( state->m_bSeparated && point.normalVelocity < 0.0f )
			{
				state->m_bSeparated = false;
				impacts.push_back( { nIndex, -point.normalVelocity } );
			}
		}
	}
	std::erase_if( contact.m_Points, [this]( const ContactPoint_t &tracked ) { return tracked.m_nStep != m_nStepCount; } );

	std::vector<FrictionContact_t *> unbound;
	for ( FrictionContact_t *pFriction : contact.m_Friction )
	{
		auto bound = std::find_if( points.begin(), points.end(), [&]( const ManifoldPoint_t &entry ) { return !entry.m_pFriction && entry.m_pPoint->featureId == pFriction->m_nFeatureId && entry.m_pPoint->triangleIndex == pFriction->m_nTriangle; } );
		if ( bound != points.end() )
			bound->m_pFriction = pFriction;
		else
			unbound.push_back( pFriction );
	}

	SelectLedgeImpacts();
	for ( const ContactImpact_t &impact : impacts )
	{
		ManifoldPoint_t &entry = points[impact.m_nPoint];
		if ( !entry.m_pFriction )
			entry.m_pFriction = CreateFrictionContact( contact, *entry.m_pManifold, *entry.m_pPoint, entry.m_WorldPoint, bBall );
	}

	for ( const LedgeContact_t &ledge : ledges )
	{
		if ( ledge.m_nDeepest < 0 || points[ledge.m_nDeepest].m_pFriction )
			continue;

		const bool bHasFriction = std::any_of( contact.m_Friction.begin(), contact.m_Friction.end(), [&]( const FrictionContact_t *pFriction ) { return GetContactLedge( contact, pFriction->m_nTriangle ) == ledge.m_nLedge; } );
		if ( !bHasFriction )
			points[ledge.m_nDeepest].m_pFriction = CreateFrictionContact( contact, *points[ledge.m_nDeepest].m_pManifold, *points[ledge.m_nDeepest].m_pPoint, points[ledge.m_nDeepest].m_WorldPoint, bBall );
	}

	for ( FrictionContact_t *pFriction : unbound )
	{
		const int nLedge = GetContactLedge( contact, pFriction->m_nTriangle );
		ManifoldPoint_t *pNearest = nullptr;
		float flNearest = FLT_MAX;
		for ( ManifoldPoint_t &entry : points )
		{
			const float flDistance = b3DistanceSquared( entry.m_WorldPoint, ToBox3D( pFriction->m_Point ) );
			if ( !entry.m_pFriction && entry.m_nLedge == nLedge && flDistance < flNearest )
			{
				flNearest = flDistance;
				pNearest = &entry;
			}
		}

		if ( !pNearest )
		{
			const bool bLedgeTouching = std::any_of( points.begin(), points.end(), [nLedge]( const ManifoldPoint_t &entry ) { return entry.m_nLedge == nLedge; } );
			pFriction->m_flSeparation = bLedgeTouching ? FRICTION_RELEASE_SEPARATION : GetMeshFrictionGap( contact, *pFriction );
			continue;
		}

		pNearest->m_pFriction = pFriction;
		pFriction->m_nFeatureId = pNearest->m_pPoint->featureId;
		pFriction->m_nTriangle = pNearest->m_pPoint->triangleIndex;
	}

	for ( FrictionContact_t *pFriction : contact.m_Friction )
		pFriction->m_flNormalImpulse = 0.0f;

	for ( ManifoldPoint_t &entry : points )
	{
		const b3ManifoldPoint &point = *entry.m_pPoint;
		if ( point.totalNormalImpulse <= 0.0f )
			continue;

		FrictionContact_t *pNearest = entry.m_pFriction;
		if ( !pNearest )
		{
			float flNearest = FLT_MAX;
			for ( const ManifoldPoint_t &other : points )
			{
				const float flDistance = b3DistanceSquared( entry.m_WorldPoint, other.m_WorldPoint );
				if ( other.m_pFriction && other.m_nLedge == entry.m_nLedge && flDistance < flNearest )
				{
					flNearest = flDistance;
					pNearest = other.m_pFriction;
				}
			}
		}
		if ( !pNearest )
			continue;

		pNearest->m_flNormalImpulse += point.totalNormalImpulse;
		const b3Vec3 offsets[2] = { point.anchorA, b3Sub( entry.m_WorldPoint, centers[1] ) };
		float flInvMass = 0.0f;
		for ( int i = 0; i < 2; ++i )
		{
			if ( contact.m_pObjects[i]->IsMoveable() )
				flInvMass += contact.m_pObjects[i]->GetFrictionInvMass( b3InvRotateVector( rotations[i], offsets[i] ) );
		}
		contact.m_flSpanCapacity += point.totalNormalImpulse * pNearest->m_flFriction * flInvMass;
	}

	for ( ManifoldPoint_t &entry : points )
	{
		if ( !entry.m_pFriction )
			continue;

		FrictionContact_t &friction = *entry.m_pFriction;
		const b3Manifold &manifold = *entry.m_pManifold;
		const b3ManifoldPoint &point = *entry.m_pPoint;
		const Vector normal = ToSource( manifold.normal );
		friction.m_Point = ToSource( entry.m_WorldPoint );
		friction.m_Normal = friction.m_nSide == 0 ? normal : -normal;
		friction.m_flSeparation = point.separation;
		if ( friction.m_bRemoved )
			continue;

		const b3Vec3 offsets[2] = { point.anchorA, b3Sub( entry.m_WorldPoint, centers[1] ) };
		const b3Vec3 slip = b3Sub( b3Add( velocities[1], b3Cross( angularVelocities[1], offsets[1] ) ), b3Add( velocities[0], b3Cross( angularVelocities[0], offsets[0] ) ) );
		const b3Vec3 tangentSlip = b3MulSub( slip, b3Dot( slip, manifold.normal ), manifold.normal );
		const b3Vec3 span = b3MulSub( friction.m_Span, b3Dot( friction.m_Span, manifold.normal ), manifold.normal );
		friction.m_Span = b3MulAdd( span, m_flTimestep, tangentSlip );
		friction.m_nSpanStep = m_nStepCount;
	}

	for ( const ContactImpact_t &impact : impacts )
	{
		const FrictionContact_t *pFriction = points[impact.m_nPoint].m_pFriction;
		if ( !pFriction->m_bRemoved )
			QueueContactEvent( ContactEvent_t::IMPACT, contact, *pFriction, impact.m_flSpeed );
	}

	if ( contact.m_bCompound )
		m_CompoundContacts.push_back( &contact );
	else
		AccumulateFrictionEnergy( contact, contact.m_flSpanCapacity );

	for ( auto it = contact.m_Friction.begin(); it != contact.m_Friction.end(); )
	{
		if ( ( *it )->m_flSeparation < FRICTION_RELEASE_SEPARATION )
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
		const b3ShapeId shapes[2] = { event.shapeIdA, event.shapeIdB };
		for ( int j = 0; j < 2; ++j )
		{
			const bool bMesh = b3Shape_GetType( shapes[j] ) == b3_meshShape;
			contact.m_pConvexes[j] = bMesh ? nullptr : static_cast<const CPhysConvex *>( b3Shape_GetUserData( shapes[j] ) );
			contact.m_pMeshes[j] = bMesh ? static_cast<const CollideMesh_t *>( b3Shape_GetUserData( shapes[j] ) ) : nullptr;
		}
		contact.m_Shapes[0] = event.shapeIdA;
		contact.m_Shapes[1] = event.shapeIdB;
		contact.m_ContactId = event.contactId;
		contact.m_nScanStep = 0;
		contact.m_nBeginStep = m_nStepCount;
		contact.m_nFirstSide = GetInitialFirstSide( contact.m_pObjects[0], contact.m_pObjects[1] );
		contact.m_nDisableCount = 0;
		contact.m_flSpanCapacity = 0.0f;
		contact.m_flMinSeparation = FLT_MAX;
		contact.m_bCompound = b3Body_GetShapeCount( contact.m_pObjects[0]->GetBody() ) > 1 || b3Body_GetShapeCount( contact.m_pObjects[1]->GetBody() ) > 1;
		contact.m_bPenetrationRefused = false;
		contact.m_bPenetrating = false;
		contact.m_bDetached = false;

		auto detached = m_DetachedContacts.find( GetShapePairKey( event.shapeIdA, event.shapeIdB ) );
		if ( detached != m_DetachedContacts.end() )
		{
			auto stored = m_Contacts.find( detached->second );
			Contact_t &previous = stored->second;
			const bool bSameOrder = IsSameShape( previous.m_Shapes[0], event.shapeIdA ) && IsSameShape( previous.m_Shapes[1], event.shapeIdB );
			const bool bSwapped = IsSameShape( previous.m_Shapes[0], event.shapeIdB ) && IsSameShape( previous.m_Shapes[1], event.shapeIdA );
			if ( bSameOrder || bSwapped )
			{
				const int nFlip = bSameOrder ? 0 : 1;
				contact.m_nFirstSide = previous.m_nFirstSide ^ nFlip;
				contact.m_nBeginStep = previous.m_nBeginStep;
				contact.m_bPenetrationRefused = previous.m_bPenetrationRefused;
				contact.m_bPenetrating = previous.m_bPenetrating;
				contact.m_Friction = std::move( previous.m_Friction );
				for ( FrictionContact_t *pFriction : contact.m_Friction )
					pFriction->m_nSide ^= nFlip;
			}
			else
			{
				RetireContact( previous, true );
			}
			EraseContact( stored );
		}

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
				contact.m_nBeginStep = reset.m_nBeginStep;
				contact.m_bPenetrationRefused = reset.m_bPenetrationRefused;
				contact.m_bPenetrating = reset.m_bPenetrating;
				contact.m_Friction = std::move( reset.m_Friction );
				if ( bSameOrder )
					contact.m_Points = std::move( reset.m_Points );
				for ( FrictionContact_t *pFriction : contact.m_Friction )
				{
					pFriction->m_nSide ^= nFlip;
					if ( !pFriction->m_bRemoved )
						LinkFriction( pFriction );
				}
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

		Contact_t &contact = it->second;
		if ( ShouldDetachContact( contact ) )
		{
			contact.m_bDetached = true;
			contact.m_flMinSeparation = FLT_MAX;
			contact.m_Points.clear();
			m_DetachedContacts[GetShapePairKey( contact.m_Shapes[0], contact.m_Shapes[1] )] = it->first;
			continue;
		}

		RetireContact( contact, true );
		EraseContact( it );
	}

	for ( const auto &[nKey, contact] : m_ProxyResetContacts )
		RetireContact( contact, true );
	m_ProxyResetContacts.clear();

	m_ScannedContacts.clear();
	m_CompoundContacts.clear();
	for ( CPhysicsObject *pObject : m_SteppedObjects )
	{
		for ( uint64_t nKey : pObject->GetContactKeys() )
		{
			Contact_t &contact = m_Contacts.find( nKey )->second;
			if ( contact.m_nScanStep == m_nStepCount )
				continue;

			contact.m_nScanStep = m_nStepCount;
			if ( contact.m_bDetached )
			{
				if ( !UpdateDetachedContact( contact ) )
					m_ExpiredContacts.push_back( nKey );
				continue;
			}

			m_ScannedContacts.push_back( nKey );
			UpdateFrictionContacts( contact );
		}
	}

	for ( uint64_t nKey : m_ExpiredContacts )
	{
		auto it = m_Contacts.find( nKey );
		RetireContact( it->second, true );
		EraseContact( it );
	}
	m_ExpiredContacts.clear();

	m_PairSpanCapacities.clear();
	for ( const Contact_t *pContact : m_CompoundContacts )
		m_PairSpanCapacities[GetPairKey( pContact->m_pObjects[0], pContact->m_pObjects[1] )] += pContact->m_flSpanCapacity;
	for ( Contact_t *pContact : m_CompoundContacts )
		AccumulateFrictionEnergy( *pContact, m_PairSpanCapacities[GetPairKey( pContact->m_pObjects[0], pContact->m_pObjects[1] )] );
}

void CPhysicsEnvironment::AccumulateFrictionEnergy( Contact_t &contact, float flCapacity )
{
	const float flMaxSpan = m_flTimestep * flCapacity;
	for ( FrictionContact_t *pFriction : contact.m_Friction )
	{
		if ( pFriction->m_bRemoved || pFriction->m_nSpanStep != m_nStepCount )
			continue;

		const float flSpanSqr = b3LengthSquared( pFriction->m_Span );
		if ( !( flSpanSqr > flMaxSpan * flMaxSpan + FRICTION_SPAN_TOLERANCE ) )
			continue;

		const float flSpan = std::sqrt( flSpanSqr );
		pFriction->m_flEnergy += ( flSpan - flMaxSpan ) * pFriction->m_flFriction * pFriction->m_flNormalImpulse * m_flInverseTimestep;
		pFriction->m_Span = b3MulSV( flMaxSpan / flSpan, pFriction->m_Span );
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
			if ( event.m_pEnding )
			{
				UnlinkFriction( event.m_pEnding );
				delete event.m_pEnding;
			}
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
	UpdatePenetrationHolds();

	for ( uint64_t nKey : m_ScannedContacts )
	{
		auto it = m_Contacts.find( nKey );
		if ( it == m_Contacts.end() )
			continue;

		Contact_t &contact = it->second;
		CPhysicsObject *pObject0 = contact.m_pObjects[0], *pObject1 = contact.m_pObjects[1];
		if ( !( contact.m_flMinSeparation < PENETRATION_SEPARATION ) )
			contact.m_bPenetrating = false;
		else if ( contact.m_nBeginStep == m_nStepCount && ( pObject0->GetPlacedStep() == m_nStepCount || pObject1->GetPlacedStep() == m_nStepCount ) )
			contact.m_bPenetrating = true;

		if ( contact.m_bPenetrationRefused && !contact.m_bPenetrating )
		{
			contact.m_bPenetrationRefused = false;
			--contact.m_nDisableCount;
			EnablePair( GetPairKey( pObject0, pObject1 ) );
		}

		if ( contact.m_bPenetrationRefused || ( pObject0->GetSleepState() != CPhysicsObject::AWAKE && pObject1->GetSleepState() != CPhysicsObject::AWAKE ) )
			continue;

		if ( contact.m_nDisableCount > 0 || !contact.m_bPenetrating )
			continue;

		if ( !m_pCollisionSolver )
		{
			ResolveStaticPenetration( contact );
			HoldStaticPenetration( contact );
			continue;
		}

		if ( ( pObject0->GetCallbackFlags() | pObject1->GetCallbackFlags() ) & CALLBACK_MARKED_FOR_DELETE )
			continue;

		const uint64_t nPairKey = GetPairKey( pObject0, pObject1 );
		if ( pObject0->IsMoveable() && pObject1->IsMoveable() )
		{
			if ( std::find( m_PenetrationPairs.begin(), m_PenetrationPairs.end(), nPairKey ) != m_PenetrationPairs.end() )
				continue;
			m_PenetrationPairs.push_back( nPairKey );
		}

		const bool bSolve = m_pCollisionSolver->ShouldSolvePenetration( pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData(), m_flTimestep ) != 0;
		it = m_Contacts.find( nKey );
		if ( it == m_Contacts.end() )
			continue;

		if ( bSolve )
		{
			ResolveStaticPenetration( it->second );
			HoldStaticPenetration( it->second );
			continue;
		}

		if ( it->second.m_pMeshes[0] || it->second.m_pMeshes[1] )
		{
			HoldStaticPenetration( it->second );
			continue;
		}

		it->second.m_bPenetrationRefused = true;
		++it->second.m_nDisableCount;
		DisablePair( nPairKey );
	}

	ApplyPenetrationResets();
}

void CPhysicsEnvironment::QueuePenetrationCheck( CPhysicsObject *pObject )
{
	if ( !pObject->IsStatic() )
	{
		pObject->SetPlacedStep( m_nStepCount + 1 );
		if ( std::find( m_PendingHoldChecks.begin(), m_PendingHoldChecks.end(), pObject ) == m_PendingHoldChecks.end() )
			m_PendingHoldChecks.push_back( pObject );
		return;
	}

	if ( !pObject->IsCollisionEnabled() )
		return;

	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = MOVING_SHAPE_BIT;
	b3World_OverlapAABB(
		m_World, b3Body_ComputeAABB( pObject->GetBody() ), filter,
		[]( b3ShapeId shape, void *pContext ) {
			static_cast<CPhysicsEnvironment *>( pContext )->QueuePenetrationCheck( GetShapeObject( shape ) );
			return true;
		},
		this );
}

bool CPhysicsEnvironment::IsShapePairHeld( b3ShapeId shape0, b3ShapeId shape1 ) const
{
	return !m_HeldShapePairs.empty() && m_HeldShapePairs.contains( GetShapePairKey( shape0, shape1 ) );
}

void CPhysicsEnvironment::DetectPendingPenetrationHolds()
{
	std::vector<b3ShapeId> shapes, meshShapes;
	for ( CPhysicsObject *pObject : m_PendingHoldChecks )
	{
		if ( !pObject->IsMoveable() || !pObject->IsCollisionEnabled() )
			continue;

		const b3BodyId body = pObject->GetBody();
		shapes.resize( b3Body_GetShapeCount( body ) );
		shapes.resize( b3Body_GetShapes( body, shapes.data(), static_cast<int>( shapes.size() ) ) );
		for ( b3ShapeId shape : shapes )
		{
			if ( b3Shape_IsSensor( shape ) )
				continue;

			meshShapes.clear();
			b3QueryFilter filter = b3DefaultQueryFilter();
			filter.maskBits = STATIC_SHAPE_BIT;
			b3World_OverlapAABB(
				m_World, b3Shape_GetAABB( shape ), filter,
				[]( b3ShapeId candidate, void *pContext ) {
					if ( b3Shape_GetType( candidate ) == b3_meshShape )
						static_cast<std::vector<b3ShapeId> *>( pContext )->push_back( candidate );
					return true;
				},
				&meshShapes );
			for ( b3ShapeId meshShape : meshShapes )
				TryHoldPenetration( shape, meshShape );
		}
	}
	m_PendingHoldChecks.clear();
}

bool CPhysicsEnvironment::HasShapeFriction( const CPhysicsObject *pObject, b3ShapeId shape, b3ShapeId meshShape ) const
{
	for ( uint64_t nKey : pObject->GetContactKeys() )
	{
		const Contact_t &contact = m_Contacts.find( nKey )->second;
		const bool bPair = ( IsSameShape( contact.m_Shapes[0], shape ) && IsSameShape( contact.m_Shapes[1], meshShape ) ) || ( IsSameShape( contact.m_Shapes[0], meshShape ) && IsSameShape( contact.m_Shapes[1], shape ) );
		if ( bPair && !contact.m_Friction.empty() )
			return true;
	}
	return false;
}

void CPhysicsEnvironment::TryHoldPenetration( b3ShapeId shape, b3ShapeId meshShape )
{
	const uint64_t nKey = GetShapePairKey( shape, meshShape );
	if ( m_HeldShapePairs.contains( nKey ) || m_ProxyResetContacts.contains( nKey ) )
		return;

	CPhysicsObject *pObject = GetShapeObject( shape );
	CPhysicsObject *pStatic = GetShapeObject( meshShape );
	if ( !IsContactEnabled( pObject, pStatic ) || HasShapeFriction( pObject, shape, meshShape ) )
		return;

	bool bCollide = true;
	if ( pObject->FindFilterDecision( pStatic, bCollide ) && !bCollide )
		return;

	if ( !FindHeldLedges( shape, meshShape, m_HeldLedges ) )
		return;

	m_PenetrationHolds.push_back( { pObject, shape, meshShape } );
	m_HeldShapePairs.insert( nKey );
	QueuePenetrationReset( pObject );
}

void CPhysicsEnvironment::QueuePenetrationReset( CPhysicsObject *pObject )
{
	if ( std::find( m_PenetrationResets.begin(), m_PenetrationResets.end(), pObject ) == m_PenetrationResets.end() )
		m_PenetrationResets.push_back( pObject );
}

void CPhysicsEnvironment::ApplyPenetrationResets()
{
	std::vector<CPhysicsObject *> resets;
	resets.swap( m_PenetrationResets );
	for ( CPhysicsObject *pObject : resets )
		pObject->ResetShapeProxies();
}

static float GetProxyDistance( const b3ShapeProxy &proxyA, b3WorldTransform transformA, const b3ShapeProxy &proxyB, b3WorldTransform transformB )
{
	b3DistanceInput input = {};
	input.proxyA = proxyA;
	input.proxyB = proxyB;
	input.transform = b3InvMulWorldTransforms( transformA, transformB );
	input.useRadii = true;
	b3SimplexCache cache = {};
	return b3ShapeDistance( &input, &cache, nullptr, 0 ).distance;
}

bool CPhysicsEnvironment::FindHeldLedges( b3ShapeId shape, b3ShapeId meshShape, std::vector<int> &ledges )
{
	ledges.clear();
	const CPhysicsObject *pStatic = GetShapeObject( meshShape );
	const CollideMesh_t &mesh = *static_cast<const CollideMesh_t *>( b3Shape_GetUserData( meshShape ) );
	const std::vector<std::unique_ptr<CPhysConvex>> &convexes = pStatic->GetCollide()->GetConvexes();

	b3Sphere sphere = {};
	b3ShapeProxy grown = {}, exact = {};
	if ( b3Shape_GetType( shape ) == b3_sphereShape )
	{
		sphere = b3Shape_GetSphere( shape );
		grown = { &sphere.center, 1, sphere.radius };
		exact = { &sphere.center, 1, sphere.radius - SHAPE_INFLATION };
	}
	else
	{
		const b3HullData *pHull = b3Shape_GetHull( shape );
		const CPhysConvex &convex = *static_cast<const CPhysConvex *>( b3Shape_GetUserData( shape ) );
		grown = { b3GetHullPoints( pHull ), pHull->vertexCount, 0.0f };
		exact = { convex.GetPoints().data(), static_cast<int>( convex.GetPoints().size() ), 0.0f };
	}

	const b3WorldTransform transform = b3Body_GetTransform( GetShapeObject( shape )->GetBody() );
	const b3WorldTransform staticTransform = b3Body_GetTransform( pStatic->GetBody() );
	const b3Transform relative = b3InvMulWorldTransforms( staticTransform, transform );
	b3AABB bounds = { b3TransformPoint( relative, grown.points[0] ), b3TransformPoint( relative, grown.points[0] ) };
	for ( int i = 1; i < grown.count; ++i )
	{
		const b3Vec3 point = b3TransformPoint( relative, grown.points[i] );
		bounds.lowerBound = b3Min( bounds.lowerBound, point );
		bounds.upperBound = b3Max( bounds.upperBound, point );
	}
	const float flMargin = grown.radius + B3_SPECULATIVE_DISTANCE;
	bounds.lowerBound = b3Sub( bounds.lowerBound, { flMargin, flMargin, flMargin } );
	bounds.upperBound = b3Add( bounds.upperBound, { flMargin, flMargin, flMargin } );

	m_MeshTriangles.clear();
	const b3Mesh query = { mesh.m_pData, b3Vec3_one };
	b3QueryMesh(
		&query, bounds,
		[]( b3Vec3, b3Vec3, b3Vec3, int nTriangle, void *pContext ) {
			static_cast<std::vector<int> *>( pContext )->push_back( nTriangle );
			return true;
		},
		&m_MeshTriangles );

	const b3MeshTriangle *pTriangles = b3GetMeshTriangles( mesh.m_pData );
	const b3Vec3 *pVertices = b3GetMeshVertices( mesh.m_pData );
	m_LedgeStates.clear();
	for ( int nTriangle : m_MeshTriangles )
	{
		const b3MeshTriangle &triangle = pTriangles[nTriangle];
		const b3Vec3 corners[3] = { pVertices[triangle.index1], pVertices[triangle.index2], pVertices[triangle.index3] };
		if ( GetProxyDistance( grown, transform, { corners, 3, 0.0f }, staticTransform ) >= B3_SPECULATIVE_DISTANCE )
			continue;

		const int nLedge = mesh.m_Sources[nTriangle].m_nConvex;
		auto state = std::find_if( m_LedgeStates.begin(), m_LedgeStates.end(), [nLedge]( const std::pair<int, bool> &entry ) { return entry.first == nLedge; } );
		if ( state == m_LedgeStates.end() )
		{
			const std::vector<b3Vec3> &points = convexes[nLedge]->GetPoints();
			state = m_LedgeStates.insert( m_LedgeStates.end(), { nLedge, GetProxyDistance( exact, transform, { points.data(), static_cast<int>( points.size() ), 0.0f }, staticTransform ) <= 0.0f } );
			if ( state->second )
				ledges.push_back( nLedge );
		}

		if ( !state->second )
		{
			return false;
		}
	}
	return !ledges.empty();
}

bool CPhysicsEnvironment::ShouldDetachContact( const Contact_t &contact ) const
{
	if ( ( !contact.m_pMeshes[0] && !contact.m_pMeshes[1] ) || !b3Shape_IsValid( contact.m_Shapes[0] ) || !b3Shape_IsValid( contact.m_Shapes[1] ) )
		return false;

	if ( !contact.m_pObjects[0]->IsCollisionEnabled() || !contact.m_pObjects[1]->IsCollisionEnabled() )
		return false;

	return std::any_of( contact.m_Friction.begin(), contact.m_Friction.end(), []( const FrictionContact_t *pFriction ) { return !pFriction->m_bRemoved && pFriction->m_flSeparation < FRICTION_RELEASE_SEPARATION; } );
}

float CPhysicsEnvironment::GetMeshFrictionGap( const Contact_t &contact, const FrictionContact_t &friction ) const
{
	const int nMesh = contact.m_pMeshes[0] ? 0 : ( contact.m_pMeshes[1] ? 1 : -1 );
	if ( nMesh < 0 || !b3Shape_IsValid( contact.m_Shapes[0] ) || !b3Shape_IsValid( contact.m_Shapes[1] ) )
		return FRICTION_RELEASE_SEPARATION;

	const b3ShapeId shape = contact.m_Shapes[1 - nMesh];
	b3Sphere sphere = {};
	b3ShapeProxy proxy = {};
	if ( b3Shape_GetType( shape ) == b3_sphereShape )
	{
		sphere = b3Shape_GetSphere( shape );
		proxy = { &sphere.center, 1, sphere.radius };
	}
	else
	{
		const b3HullData *pHull = b3Shape_GetHull( shape );
		proxy = { b3GetHullPoints( pHull ), pHull->vertexCount, 0.0f };
	}

	const b3MeshData *pData = contact.m_pMeshes[nMesh]->m_pData;
	const b3MeshTriangle &triangle = b3GetMeshTriangles( pData )[friction.m_nTriangle];
	const b3Vec3 *pVertices = b3GetMeshVertices( pData );
	const b3Vec3 corners[3] = { pVertices[triangle.index1], pVertices[triangle.index2], pVertices[triangle.index3] };
	const float flDistance = GetProxyDistance( proxy, b3Body_GetTransform( contact.m_pObjects[1 - nMesh]->GetBody() ), { corners, 3, 0.0f }, b3Body_GetTransform( contact.m_pObjects[nMesh]->GetBody() ) );
	return flDistance - B3_MESH_REST_OFFSET;
}

bool CPhysicsEnvironment::UpdateDetachedContact( Contact_t &contact )
{
	if ( !ShouldDetachContact( contact ) )
		return false;

	for ( auto it = contact.m_Friction.begin(); it != contact.m_Friction.end(); )
	{
		FrictionContact_t *pFriction = *it;
		if ( !pFriction->m_bRemoved )
			pFriction->m_flSeparation = GetMeshFrictionGap( contact, *pFriction );
		if ( pFriction->m_flSeparation < FRICTION_RELEASE_SEPARATION )
		{
			++it;
			continue;
		}

		ReleaseFriction( contact, pFriction, true );
		it = contact.m_Friction.erase( it );
	}
	return !contact.m_Friction.empty();
}

void CPhysicsEnvironment::ResolveHeldLedges( CPhysicsObject *pObject, CPhysicsObject *pStatic, const std::vector<int> &ledges )
{
	if ( pObject->GetSleepState() != CPhysicsObject::AWAKE )
		return;

	if ( m_pCollisionSolver )
	{
		if ( ( pStatic->GetCallbackFlags() | pObject->GetCallbackFlags() ) & CALLBACK_MARKED_FOR_DELETE )
			return;
		if ( !m_pCollisionSolver->ShouldSolvePenetration( pStatic, pObject, pStatic->GetGameData(), pObject->GetGameData(), m_flTimestep ) )
			return;
	}

	const std::vector<std::unique_ptr<CPhysConvex>> &convexes = pStatic->GetCollide()->GetConvexes();
	for ( int nLedge : ledges )
		ResolveLedgePenetration( *convexes[nLedge], pStatic, pObject );
}

void CPhysicsEnvironment::UpdatePenetrationHolds()
{
	for ( size_t i = 0; i < m_PenetrationHolds.size(); )
	{
		const PenetrationHold_t hold = m_PenetrationHolds[i];
		bool bHeld = b3Shape_IsValid( hold.m_Shape ) && b3Shape_IsValid( hold.m_MeshShape ) && hold.m_pObject->IsMoveable() && hold.m_pObject->IsCollisionEnabled();
		CPhysicsObject *pStatic = bHeld ? GetShapeObject( hold.m_MeshShape ) : nullptr;
		bHeld = bHeld && pStatic->IsCollisionEnabled() && IsContactEnabled( hold.m_pObject, pStatic ) && FindHeldLedges( hold.m_Shape, hold.m_MeshShape, m_HeldLedges );
		if ( bHeld )
		{
			ResolveHeldLedges( hold.m_pObject, pStatic, m_HeldLedges );
			++i;
			continue;
		}

		m_HeldShapePairs.erase( GetShapePairKey( hold.m_Shape, hold.m_MeshShape ) );
		m_PenetrationHolds.erase( m_PenetrationHolds.begin() + static_cast<ptrdiff_t>( i ) );
		if ( b3Shape_IsValid( hold.m_Shape ) && hold.m_pObject->IsCollisionEnabled() )
			QueuePenetrationReset( hold.m_pObject );
	}
}

void CPhysicsEnvironment::ResolveStaticPenetration( const Contact_t &contact )
{
	const int nStatic = contact.m_pObjects[0]->IsStatic() ? 0 : ( contact.m_pObjects[1]->IsStatic() ? 1 : -1 );
	if ( nStatic < 0 )
		return;

	CPhysicsObject *pObject = contact.m_pObjects[1 - nStatic];
	if ( !pObject->IsMoveable() || pObject->GetSleepState() != CPhysicsObject::AWAKE )
		return;

	const CPhysicsObject *pStatic = contact.m_pObjects[nStatic];
	if ( const CollideMesh_t *pMesh = contact.m_pMeshes[nStatic] )
	{
		std::vector<int> &ledges = m_PenetrationLedges;
		ledges.clear();
		const b3ContactData data = b3Contact_GetData( contact.m_ContactId );
		for ( int m = 0; m < data.manifoldCount; ++m )
		{
			for ( int p = 0; p < data.manifolds[m].pointCount; ++p )
			{
				const b3ManifoldPoint &point = data.manifolds[m].points[p];
				const int nConvex = pMesh->m_Sources[point.triangleIndex].m_nConvex;
				if ( point.separation < PENETRATION_SEPARATION && std::find( ledges.begin(), ledges.end(), nConvex ) == ledges.end() )
					ledges.push_back( nConvex );
			}
		}

		const std::vector<std::unique_ptr<CPhysConvex>> &convexes = pStatic->GetCollide()->GetConvexes();
		for ( int nConvex : ledges )
			ResolveLedgePenetration( *convexes[nConvex], pStatic, pObject );
		return;
	}

	if ( contact.m_pConvexes[nStatic] )
		ResolveLedgePenetration( *contact.m_pConvexes[nStatic], pStatic, pObject );
}

void CPhysicsEnvironment::HoldStaticPenetration( const Contact_t &contact )
{
	if ( !contact.m_Friction.empty() )
		return;

	for ( int i = 0; i < 2; ++i )
	{
		if ( contact.m_pMeshes[i] && contact.m_pObjects[i]->IsStatic() && contact.m_pObjects[1 - i]->IsMoveable() )
			TryHoldPenetration( contact.m_Shapes[1 - i], contact.m_Shapes[i] );
	}
}

void CPhysicsEnvironment::ResolveLedgePenetration( const CPhysConvex &convex, const CPhysicsObject *pStatic, CPhysicsObject *pObject )
{
	const b3WorldTransform transform = b3Body_GetTransform( pStatic->GetBody() );
	const b3Vec3 center = b3InvRotateVector( transform.q, b3Sub( b3ToVec3( b3Body_GetWorldCenter( pObject->GetBody() ) ), b3ToVec3( transform.p ) ) );
	const std::vector<b3Vec3> &points = convex.GetPoints();
	float flBestDistance = -FLT_MAX;
	b3Vec3 bestNormal = b3Vec3_zero;
	for ( const CPhysConvex::Triangle_t &triangle : convex.GetTriangles() )
	{
		const b3Vec3 a = points[triangle.m_nVertices[0]];
		float flLength = 0.0f;
		const b3Vec3 normal = b3GetLengthAndNormalize( &flLength, b3Cross( b3Sub( points[triangle.m_nVertices[2]], a ), b3Sub( points[triangle.m_nVertices[1]], a ) ) );
		if ( flLength <= 0.0f )
			continue;

		const float flDistance = b3Dot( normal, b3Sub( center, a ) );
		if ( flDistance > flBestDistance )
		{
			flBestDistance = flDistance;
			bestNormal = normal;
		}
	}
	if ( flBestDistance == -FLT_MAX )
		return;

	const float flSpeed = PENETRATION_SPEED_SCALE * GetGravityVector().Length() * m_flTimestep * SOURCE_TO_IVP;
	const b3Vec3 normal = SourceToIVPDirection( b3RotateVector( transform.q, bestNormal ) );
	if ( b3Dot( b3Add( pObject->GetCoreVelocity(), pObject->GetPendingVelocity() ), normal ) < 0.1f * flSpeed )
		pObject->SetPendingVelocity( b3MulAdd( pObject->GetPendingVelocity(), flSpeed, normal ) );
}

const std::vector<CPhysicsEnvironment::ContactNormal_t> &CPhysicsEnvironment::GetContactNormals( const CPhysicsObject *pObject )
{
	m_ContactNormals.clear();
	for ( const FrictionContact_t *pFriction = pObject->GetFrictionHead(); pFriction; pFriction = pFriction->GetNext( pObject ) )
	{
		ContactNormal_t &entry = m_ContactNormals.emplace_back();
		entry.m_pOther = pFriction->m_pObjects[pFriction->m_pObjects[0] == pObject ? 1 : 0];
		entry.m_Normal = GetFrictionNormal( *pFriction, pObject );
		entry.m_flNormalForce = pFriction->m_flNormalImpulse * m_flInverseTimestep;
	}
	return m_ContactNormals;
}

IPhysicsFrictionSnapshot *CPhysicsEnvironment::CreateFrictionSnapshot( CPhysicsObject *pObject )
{
	return new CPhysicsFrictionSnapshot( pObject );
}

Vector CPhysicsEnvironment::GetFrictionNormal( const FrictionContact_t &friction, const CPhysicsObject *pObject ) const
{
	return NormalizeSurfaceNormal( friction.m_pObjects[0] == pObject ? friction.m_Normal : -friction.m_Normal );
}

bool CPhysicsEnvironment::GetContactPoint( const CPhysicsObject *pObject, Vector *pContactPoint, IPhysicsObject **ppContactObject ) const
{
	const FrictionContact_t *pFriction = pObject->GetFrictionHead();
	if ( !pFriction )
		return false;

	if ( pContactPoint )
		*pContactPoint = pFriction->m_Point;
	if ( ppContactObject )
		*ppContactObject = pFriction->m_pObjects[pFriction->m_pObjects[0] == pObject ? 1 : 0];
	return true;
}

bool CPhysicsEnvironment::GetStrongestContact( const CPhysicsObject *pObject, Vector &point, int &nSurface ) const
{
	float flBestForce = -1.0f;
	bool bContact = false;
	for ( const FrictionContact_t *pFriction = pObject->GetFrictionHead(); pFriction; pFriction = pFriction->GetNext( pObject ) )
	{
		const float flForce = pFriction->m_flNormalImpulse * m_flInverseTimestep;
		if ( !( flForce > flBestForce ) )
			continue;

		flBestForce = flForce;
		point = pFriction->m_Point;
		nSurface = pFriction->m_Materials[pFriction->m_pObjects[0] == pObject ? 1 : 0];
		bContact = true;
	}
	return bContact;
}

bool CPhysicsEnvironment::HasFrictionContact( const CPhysicsObject *pObject ) const
{
	for ( const FrictionContact_t *pFriction = pObject->GetFrictionHead(); pFriction; pFriction = pFriction->GetNext( pObject ) )
	{
		if ( !pFriction->m_bEnding )
			return true;
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
	for ( CPhysicsObject *pObject : m_ActiveObjects )
	{
		if ( !( pObject->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) )
			continue;

		for ( FrictionContact_t *pFriction = pObject->GetFrictionHead(); pFriction; pFriction = pFriction->GetNext( pObject ) )
		{
			const int nSide = pFriction->m_pObjects[0] == pObject ? 0 : 1;
			if ( pFriction->m_bEnding || pFriction->m_flEnergy == 0.0f || !( pFriction->m_pObjects[1 - nSide]->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) )
				continue;

			const float flEnergy = pFriction->m_flEnergy * flInverse * pObject->GetInvMass();
			pFriction->m_flEnergy = 0.0f;
			if ( !( flEnergy > FRICTION_ENERGY_THRESHOLD * IVP_ENERGY_TO_SOURCE ) )
				continue;

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
