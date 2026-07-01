//=================================================================================================
//
// Interface to a physics scene
//
//=================================================================================================

#include "cbase.h"

#include "vbox_environment.h"
#include "vbox_object.h"
#include "vbox_collide.h"
#include "vbox_controllers.h"
#include "vbox_surfaceprops.h"

#include "tier0/memdbgon.h"

namespace
{
	// Apply a Source surface's friction/bounce/density to a shape. Box3D combines both shapes on contact.
	void ApplyMaterialToShape( b3ShapeDef &shapeDef, int materialIndex )
	{
		surfacedata_t *pSurface = Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData( materialIndex );
		if ( !pSurface )
			return;

		shapeDef.baseMaterial.friction = Max( pSurface->physics.friction, 0.0f );
		// Restitution >= 1 adds energy every bounce and blows stacks up.
		shapeDef.baseMaterial.restitution = clamp( pSurface->physics.elasticity, 0.0f, 1.0f );
		if ( pSurface->physics.density > 0.0f )
			shapeDef.density = pSurface->physics.density;	// kg/m^3 in both, geometry is in metres
	}
}

Box3DPhysicsEnvironment::Box3DPhysicsEnvironment()
{
	b3WorldDef def = b3DefaultWorldDef();
	// Only hit events for impacts >= 70 in/s (Source's collision-sound threshold).
	def.hitEventThreshold = SourceToBox::Distance( 70.0f );
	// Cap body speed at sv_maxvelocity so a solver blow-up can't fling objects to an invalid coordinate,
	// which the engine treats as deleted.
	def.maximumLinearSpeed = SourceToBox::Distance( 3500.0f );
	def.enableContinuous = true;
	m_WorldId = b3CreateWorld( &def );
}

Box3DPhysicsEnvironment::~Box3DPhysicsEnvironment()
{
	b3DestroyWorld( m_WorldId );
}

void Box3DPhysicsEnvironment::SetDebugOverlay( CreateInterfaceFn debugOverlayFactory )
{
	Log_Stub( LOG_VBox3D );
}

IVPhysicsDebugOverlay* Box3DPhysicsEnvironment::GetDebugOverlay( void )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

void Box3DPhysicsEnvironment::SetGravity( const Vector& gravityVector )
{
	m_vecGravity = gravityVector;
	b3World_SetGravity( m_WorldId, SourceToBox::Distance( gravityVector ) );
}

void Box3DPhysicsEnvironment::GetGravity( Vector* pGravityVector ) const
{
	*pGravityVector = m_vecGravity;
}

void Box3DPhysicsEnvironment::SetAirDensity( float density )
{
	m_flAirDensity = density;
}

float Box3DPhysicsEnvironment::GetAirDensity() const
{
	return m_flAirDensity;
}

IPhysicsObject *Box3DPhysicsEnvironment::CreateObject( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool bStatic )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = bStatic ? b3_staticBody : b3_dynamicBody;
	bodyDef.position = SourceToBox::Distance( position );
	bodyDef.rotation = SourceToBox::Angle( angles );

	const b3BodyId bodyId = b3CreateBody( m_WorldId, &bodyDef );

	if ( pCollisionModel )
	{
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		shapeDef.enableHitEvents = true;
		ApplyMaterialToShape( shapeDef, materialIndex );
		for ( int i = 0; i < pCollisionModel->m_Convexes.Count(); i++ )
			if ( pCollisionModel->m_Convexes[ i ]->m_pHull )
				b3CreateHullShape( bodyId, &shapeDef, pCollisionModel->m_Convexes[ i ]->m_pHull );

		if ( pCollisionModel->m_pMesh )
			b3CreateMeshShape( bodyId, &shapeDef, pCollisionModel->m_pMesh, b3Vec3{ 1.0f, 1.0f, 1.0f } );
	}

	Box3DPhysicsObject *pObject = new Box3DPhysicsObject( bodyId, this, bStatic, materialIndex, pCollisionModel, pParams );
	m_Objects.AddToTail( pObject );
	return pObject;
}

IPhysicsObject* Box3DPhysicsEnvironment::CreatePolyObject( const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams )
{
	return CreateObject( pCollisionModel, materialIndex, position, angles, pParams, false );
}

IPhysicsObject* Box3DPhysicsEnvironment::CreatePolyObjectStatic( const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams )
{
	return CreateObject( pCollisionModel, materialIndex, position, angles, pParams, true );
}

IPhysicsObject* Box3DPhysicsEnvironment::CreateSphereObject( float radius, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams, bool isStatic )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = isStatic ? b3_staticBody : b3_dynamicBody;
	bodyDef.position = SourceToBox::Distance( position );
	bodyDef.rotation = SourceToBox::Angle( angles );

	const b3BodyId bodyId = b3CreateBody( m_WorldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	shapeDef.enableHitEvents = true;
	ApplyMaterialToShape( shapeDef, materialIndex );
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, SourceToBox::Distance( radius ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	Box3DPhysicsObject *pObject = new Box3DPhysicsObject( bodyId, this, isStatic, materialIndex, nullptr, pParams );
	m_Objects.AddToTail( pObject );
	return pObject;
}

void Box3DPhysicsEnvironment::DestroyObject( IPhysicsObject* pObject )
{
	if ( !pObject )
		return;

	Box3DPhysicsObject *pBoxObject = static_cast< Box3DPhysicsObject * >( pObject );

	// Drop any controllers referencing this object while its body is still valid.
	pBoxObject->RemoveShadowController();
	for ( int i = 0; i < m_MotionControllers.Count(); i++ )
		m_MotionControllers[ i ]->DetachObject( pBoxObject );

	m_Objects.FindAndRemove( pBoxObject );
	m_ActiveObjects.FindAndRemove( pBoxObject );
	b3DestroyBody( pBoxObject->GetBodyID() );
	delete pBoxObject;
}

IPhysicsFluidController* Box3DPhysicsEnvironment::CreateFluidController( IPhysicsObject* pFluidObject, fluidparams_t* pParams )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

void Box3DPhysicsEnvironment::DestroyFluidController( IPhysicsFluidController* )
{
	Log_Stub( LOG_VBox3D );
}

IPhysicsSpring* Box3DPhysicsEnvironment::CreateSpring( IPhysicsObject* pObjectStart, IPhysicsObject* pObjectEnd, springparams_t* pParams )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

void Box3DPhysicsEnvironment::DestroySpring( IPhysicsSpring* )
{
	Log_Stub( LOG_VBox3D );
}

// No-op constraints. The game virtual-calls whatever these Create* methods return without a null
// check, so they must return a valid object rather than null.
namespace
{
	class Box3DDummyConstraint final : public IPhysicsConstraint
	{
	public:
		void Activate() override {}
		void Deactivate() override {}
		void SetGameData( void* gameData ) override { m_pGameData = gameData; }
		void* GetGameData() const override { return m_pGameData; }
		IPhysicsObject* GetReferenceObject() const override { return m_pReference; }
		IPhysicsObject* GetAttachedObject() const override { return m_pAttached; }
		void SetLinearMotor( float, float ) override {}
		void SetAngularMotor( float, float ) override {}
		void UpdateRagdollTransforms( const matrix3x4_t&, const matrix3x4_t& ) override {}
		bool GetConstraintTransform( matrix3x4_t* pConstraintToReference, matrix3x4_t* pConstraintToAttached ) const override
		{
			if ( pConstraintToReference ) SetIdentityMatrix( *pConstraintToReference );
			if ( pConstraintToAttached ) SetIdentityMatrix( *pConstraintToAttached );
			return false;
		}
		bool GetConstraintParams( constraint_breakableparams_t* pParams ) const override
		{
			if ( pParams ) memset( pParams, 0, sizeof( *pParams ) );
			return false;
		}
		void OutputDebugInfo() override {}

		void* m_pGameData = nullptr;
		IPhysicsObject* m_pReference = nullptr;
		IPhysicsObject* m_pAttached = nullptr;
	};

	class Box3DDummyConstraintGroup final : public IPhysicsConstraintGroup
	{
	public:
		void Activate() override {}
		bool IsInErrorState() override { return false; }
		void ClearErrorState() override {}
		void GetErrorParams( constraint_groupparams_t* pParams ) override { if ( pParams ) memset( pParams, 0, sizeof( *pParams ) ); }
		void SetErrorParams( const constraint_groupparams_t& ) override {}
		void SolvePenetration( IPhysicsObject*, IPhysicsObject* ) override {}
	};

	class Box3DDummyPlayerController final : public IPhysicsPlayerController
	{
	public:
		void Update( const Vector&, const Vector&, float, bool, IPhysicsObject* ) override {}
		void SetEventHandler( IPhysicsPlayerControllerEvent* ) override {}
		bool IsInContact() override { return false; }
		void MaxSpeed( const Vector& ) override {}
		void SetObject( IPhysicsObject* pObject ) override { m_pObject = pObject; }
		int GetShadowPosition( Vector* position, QAngle* angles ) override
		{
			if ( position ) *position = vec3_origin;
			if ( angles ) *angles = vec3_angle;
			return 0;
		}
		void StepUp( float ) override {}
		void Jump() override {}
		void GetShadowVelocity( Vector* velocity ) override { if ( velocity ) *velocity = vec3_origin; }
		IPhysicsObject* GetObject() override { return m_pObject; }
		void GetLastImpulse( Vector* pOut ) override { if ( pOut ) *pOut = vec3_origin; }
		void SetPushMassLimit( float ) override {}
		void SetPushSpeedLimit( float ) override {}
		float GetPushMassLimit() override { return 0.0f; }
		float GetPushSpeedLimit() override { return 0.0f; }
		bool WasFrozen() override { return false; }

		IPhysicsObject* m_pObject = nullptr;
	};

	IPhysicsConstraint* MakeDummyConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject )
	{
		Box3DDummyConstraint* pConstraint = new Box3DDummyConstraint;
		pConstraint->m_pReference = pReferenceObject;
		pConstraint->m_pAttached = pAttachedObject;
		return pConstraint;
	}
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateRagdollConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_ragdollparams_t& ragdoll )
{
	return MakeDummyConstraint( pReferenceObject, pAttachedObject );
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateHingeConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_hingeparams_t& hinge )
{
	return MakeDummyConstraint( pReferenceObject, pAttachedObject );
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateFixedConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_fixedparams_t& fixed )
{
	return MakeDummyConstraint( pReferenceObject, pAttachedObject );
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateSlidingConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_slidingparams_t& sliding )
{
	return MakeDummyConstraint( pReferenceObject, pAttachedObject );
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateBallsocketConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_ballsocketparams_t& ballsocket )
{
	return MakeDummyConstraint( pReferenceObject, pAttachedObject );
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreatePulleyConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_pulleyparams_t& pulley )
{
	return MakeDummyConstraint( pReferenceObject, pAttachedObject );
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateLengthConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_lengthparams_t& length )
{
	return MakeDummyConstraint( pReferenceObject, pAttachedObject );
}

void Box3DPhysicsEnvironment::DestroyConstraint( IPhysicsConstraint* pConstraint )
{
	delete static_cast< Box3DDummyConstraint* >( pConstraint );
}

IPhysicsConstraintGroup* Box3DPhysicsEnvironment::CreateConstraintGroup( const constraint_groupparams_t& groupParams )
{
	return new Box3DDummyConstraintGroup;
}

void Box3DPhysicsEnvironment::DestroyConstraintGroup( IPhysicsConstraintGroup* pGroup )
{
	delete static_cast< Box3DDummyConstraintGroup* >( pGroup );
}

IPhysicsShadowController* Box3DPhysicsEnvironment::CreateShadowController( IPhysicsObject* pObject, bool allowTranslation, bool allowRotation )
{
	Box3DPhysicsShadowController* pController = new Box3DPhysicsShadowController( static_cast< Box3DPhysicsObject* >( pObject ), allowTranslation, allowRotation );
	m_ShadowControllers.AddToTail( pController );
	return pController;
}

void Box3DPhysicsEnvironment::DestroyShadowController( IPhysicsShadowController* pController )
{
	Box3DPhysicsShadowController* pShadow = static_cast< Box3DPhysicsShadowController* >( pController );
	m_ShadowControllers.FindAndRemove( pShadow );
	delete pShadow;
}

IPhysicsPlayerController* Box3DPhysicsEnvironment::CreatePlayerController( IPhysicsObject* pObject )
{
	Box3DDummyPlayerController* pController = new Box3DDummyPlayerController;
	pController->m_pObject = pObject;
	return pController;
}

void Box3DPhysicsEnvironment::DestroyPlayerController( IPhysicsPlayerController* pController )
{
	delete static_cast< Box3DDummyPlayerController* >( pController );
}

IPhysicsMotionController* Box3DPhysicsEnvironment::CreateMotionController( IMotionEvent* pHandler )
{
	Box3DPhysicsMotionController* pController = new Box3DPhysicsMotionController( pHandler );
	m_MotionControllers.AddToTail( pController );
	return pController;
}

void Box3DPhysicsEnvironment::DestroyMotionController( IPhysicsMotionController* pController )
{
	Box3DPhysicsMotionController* pMotion = static_cast< Box3DPhysicsMotionController* >( pController );
	m_MotionControllers.FindAndRemove( pMotion );
	delete pMotion;
}

IPhysicsVehicleController* Box3DPhysicsEnvironment::CreateVehicleController( IPhysicsObject* pVehicleBodyObject, const vehicleparams_t& params, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

void Box3DPhysicsEnvironment::DestroyVehicleController( IPhysicsVehicleController* )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::SetCollisionSolver( IPhysicsCollisionSolver* pSolver )
{
	m_pCollisionSolver = pSolver;
}

void Box3DPhysicsEnvironment::Simulate( float deltaTime )
{
	if ( deltaTime <= 0.0f )
		return;

	// Drive game-controlled objects before stepping: pickup/physgun/doors (shadow) and the
	// gravity-gun grab (motion) each nudge their objects' velocity/force toward the game's target.
	for ( int i = 0; i < m_ShadowControllers.Count(); i++ )
		m_ShadowControllers[ i ]->OnPreSimulate( deltaTime );
	for ( int i = 0; i < m_MotionControllers.Count(); i++ )
		m_MotionControllers[ i ]->OnPreSimulate( deltaTime );

	m_bInSimulation = true;
	b3World_Step( m_WorldId, deltaTime, 4 );
	m_bInSimulation = false;

	// Collect the objects that moved so the game can read back their transforms.
	const b3BodyEvents events = b3World_GetBodyEvents( m_WorldId );
	m_ActiveObjects.RemoveAll();
	for ( int i = 0; i < events.moveCount; i++ )
	{
		Box3DPhysicsObject *pObject = static_cast< Box3DPhysicsObject * >( events.moveEvents[ i ].userData );
		if ( pObject )
			m_ActiveObjects.AddToTail( pObject );
	}

	DrainContactEvents();
}

namespace
{
	// A single contact's collision data, handed to the game during a Pre/PostCollision or touch callback.
	class Box3DCollisionData final : public IPhysicsCollisionData
	{
	public:
		Box3DCollisionData( const Vector &vecNormal, const Vector &vecPoint )
			: m_vecNormal( vecNormal ), m_vecPoint( vecPoint ) {}
		void GetSurfaceNormal( Vector &out ) override	{ out = m_vecNormal; }
		void GetContactPoint( Vector &out ) override	{ out = m_vecPoint; }
		void GetContactSpeed( Vector &out ) override	{ out = vec3_origin; }
	private:
		Vector m_vecNormal;
		Vector m_vecPoint;
	};

	Box3DPhysicsObject *ObjectFromShape( b3ShapeId shapeId )
	{
		if ( !b3Shape_IsValid( shapeId ) )
			return nullptr;
		const b3BodyId bodyId = b3Shape_GetBody( shapeId );
		if ( !b3Body_IsValid( bodyId ) )
			return nullptr;
		return static_cast< Box3DPhysicsObject * >( b3Body_GetUserData( bodyId ) );
	}

	// Should a Pre/PostCollision (sound/damage) callback fire for this pair? Mirrors the game's rules.
	bool IsCollisionCallback( Box3DPhysicsObject *p1, Box3DPhysicsObject *p2 )
	{
		bool bIsCollision = ( p1->GetCallbackFlags() & p2->GetCallbackFlags() ) & CALLBACK_GLOBAL_COLLISION;
		if ( p1->IsStatic() && !( p2->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC ) )
			bIsCollision = false;
		if ( p2->IsStatic() && !( p1->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC ) )
			bIsCollision = false;
		return bIsCollision;
	}

	// Should a StartTouch/EndTouch callback fire for this pair?
	bool ShouldTouchCallback( Box3DPhysicsObject *p1, Box3DPhysicsObject *p2 )
	{
		const uint32 uFlags = (uint32)p1->GetCallbackFlags() | (uint32)p2->GetCallbackFlags();
		if ( !( uFlags & CALLBACK_GLOBAL_TOUCH ) )
			return false;
		if ( !( uFlags & CALLBACK_GLOBAL_TOUCH_STATIC ) && ( p1->IsStatic() || p2->IsStatic() ) )
			return false;
		return true;
	}
}

void Box3DPhysicsEnvironment::DrainContactEvents()
{
	if ( !m_pCollisionEvent )
		return;

	const b3ContactEvents events = b3World_GetContactEvents( m_WorldId );

	// Begin-touch -> StartTouch
	for ( int i = 0; i < events.beginCount; i++ )
	{
		Box3DPhysicsObject *p1 = ObjectFromShape( events.beginEvents[ i ].shapeIdA );
		Box3DPhysicsObject *p2 = ObjectFromShape( events.beginEvents[ i ].shapeIdB );
		if ( !p1 || !p2 || !ShouldTouchCallback( p1, p2 ) )
			continue;

		Box3DCollisionData data( vec3_origin, vec3_origin );
		m_pCollisionEvent->StartTouch( p1, p2, &data );
	}

	// Hit events -> Pre/PostCollision (drives impact sounds and damage). Pre/Post must be a matched pair.
	for ( int i = 0; i < events.hitCount; i++ )
	{
		const b3ContactHitEvent &hit = events.hitEvents[ i ];
		Box3DPhysicsObject *p1 = ObjectFromShape( hit.shapeIdA );
		Box3DPhysicsObject *p2 = ObjectFromShape( hit.shapeIdB );
		if ( !p1 || !p2 || !IsCollisionCallback( p1, p2 ) )
			continue;

		// Box3D's normal points from A to B; the game wants it pointing toward object[1], so negate.
		Box3DCollisionData data( -BoxToSource::Unitless( hit.normal ), BoxToSource::Distance( hit.point ) );

		vcollisionevent_t event = {};
		event.pObjects[ 0 ]			= p1;
		event.pObjects[ 1 ]			= p2;
		event.surfaceProps[ 0 ]		= p1->GetMaterialIndex();
		event.surfaceProps[ 1 ]		= p2->GetMaterialIndex();
		event.isCollision			= true;
		event.isShadowCollision		= false;
		event.deltaCollisionTime	= 100.0f;
		event.collisionSpeed		= BoxToSource::Distance( hit.approachSpeed );
		event.pInternalData			= &data;

		m_pCollisionEvent->PreCollision( &event );
		m_pCollisionEvent->PostCollision( &event );
	}

	// End-touch -> EndTouch. Shapes/bodies here may already be destroyed; ObjectFromShape guards that.
	for ( int i = 0; i < events.endCount; i++ )
	{
		Box3DPhysicsObject *p1 = ObjectFromShape( events.endEvents[ i ].shapeIdA );
		Box3DPhysicsObject *p2 = ObjectFromShape( events.endEvents[ i ].shapeIdB );
		if ( !p1 || !p2 || !ShouldTouchCallback( p1, p2 ) )
			continue;

		Box3DCollisionData data( vec3_origin, vec3_origin );
		m_pCollisionEvent->EndTouch( p1, p2, &data );
	}

	m_pCollisionEvent->PostSimulationFrame();
}

bool Box3DPhysicsEnvironment::IsInSimulation() const
{
	return m_bInSimulation;
}

float Box3DPhysicsEnvironment::GetSimulationTimestep() const
{
	return m_flSimulationTimestep;
}

void Box3DPhysicsEnvironment::SetSimulationTimestep( float timestep )
{
	m_flSimulationTimestep = timestep;
}

float Box3DPhysicsEnvironment::GetSimulationTime() const
{
	Log_Stub( LOG_VBox3D );
	return 0.0f;
}

void Box3DPhysicsEnvironment::ResetSimulationClock()
{
	Log_Stub( LOG_VBox3D );
}

float Box3DPhysicsEnvironment::GetNextFrameTime() const
{
	Log_Stub( LOG_VBox3D );
	return 0.0f;
}

void Box3DPhysicsEnvironment::SetCollisionEventHandler( IPhysicsCollisionEvent* pCollisionEvents )
{
	m_pCollisionEvent = pCollisionEvents;
}

void Box3DPhysicsEnvironment::SetObjectEventHandler( IPhysicsObjectEvent* pObjectEvents )
{
	m_pObjectEvent = pObjectEvents;
}

void Box3DPhysicsEnvironment::SetConstraintEventHandler( IPhysicsConstraintEvent* pConstraintEvents )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::SetQuickDelete( bool bQuick )
{
	Log_Stub( LOG_VBox3D );
}

int Box3DPhysicsEnvironment::GetActiveObjectCount() const
{
	return m_ActiveObjects.Count();
}

void Box3DPhysicsEnvironment::GetActiveObjects( IPhysicsObject** pOutputObjectList ) const
{
	for ( int i = 0; i < m_ActiveObjects.Count(); i++ )
		pOutputObjectList[ i ] = m_ActiveObjects[ i ];
}

const IPhysicsObject** Box3DPhysicsEnvironment::GetObjectList( int* pOutputObjectCount ) const
{
	if ( pOutputObjectCount )
		*pOutputObjectCount = m_Objects.Count();
	return (const IPhysicsObject **)m_Objects.Base();
}

bool Box3DPhysicsEnvironment::TransferObject( IPhysicsObject* pObject, IPhysicsEnvironment* pDestinationEnvironment )
{
	Log_Stub( LOG_VBox3D );
	return false;
}

void Box3DPhysicsEnvironment::CleanupDeleteList()
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::EnableDeleteQueue( bool enable )
{
	Log_Stub( LOG_VBox3D );
}

bool Box3DPhysicsEnvironment::Save( const physsaveparams_t& params )
{
	Log_Stub( LOG_VBox3D );
	return false;
}

void Box3DPhysicsEnvironment::PreRestore( const physprerestoreparams_t& params )
{
	Log_Stub( LOG_VBox3D );
}

bool Box3DPhysicsEnvironment::Restore( const physrestoreparams_t& params )
{
	Log_Stub( LOG_VBox3D );
	return false;
}

void Box3DPhysicsEnvironment::PostRestore()
{
	Log_Stub( LOG_VBox3D );
}

bool Box3DPhysicsEnvironment::IsCollisionModelUsed( CPhysCollide* pCollide ) const
{
	Log_Stub( LOG_VBox3D );
	return false;
}

void Box3DPhysicsEnvironment::TraceRay( const Ray_t& ray, unsigned int fMask, IPhysicsTraceFilter* pTraceFilter, trace_t* pTrace )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::SweepCollideable( const CPhysCollide* pCollide, const Vector& vecAbsStart, const Vector& vecAbsEnd,
	const QAngle& vecAngles, unsigned int fMask, IPhysicsTraceFilter* pTraceFilter, trace_t* pTrace )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::GetPerformanceSettings( physics_performanceparams_t* pOutput ) const
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::SetPerformanceSettings( const physics_performanceparams_t* pSettings )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::ReadStats( physics_stats_t* pOutput )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::ClearStats()
{
	Log_Stub( LOG_VBox3D );
}

unsigned int Box3DPhysicsEnvironment::GetObjectSerializeSize( IPhysicsObject* pObject ) const
{
	Log_Stub( LOG_VBox3D );
	return 0;
}

void Box3DPhysicsEnvironment::SerializeObjectToBuffer( IPhysicsObject* pObject, unsigned char* pBuffer, unsigned int bufferSize )
{
	Log_Stub( LOG_VBox3D );
}

IPhysicsObject* Box3DPhysicsEnvironment::UnserializeObjectFromBuffer( void* pGameData, unsigned char* pBuffer, unsigned int bufferSize, bool enableCollisions )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

void Box3DPhysicsEnvironment::EnableConstraintNotify( bool bEnable )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::DebugCheckContacts()
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::SetAlternateGravity( const Vector& gravityVector )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::GetAlternateGravity( Vector* pGravityVector ) const
{
	Log_Stub( LOG_VBox3D );
}

float Box3DPhysicsEnvironment::GetDeltaFrameTime( int maxTicks ) const
{
	Log_Stub( LOG_VBox3D );
	return 0.0f;
}

void Box3DPhysicsEnvironment::ForceObjectsToSleep( IPhysicsObject** pList, int listCount )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::SetPredicted( bool bPredicted )
{
	Log_Stub( LOG_VBox3D );
}

bool Box3DPhysicsEnvironment::IsPredicted()
{
	Log_Stub( LOG_VBox3D );
	return false;
}

void Box3DPhysicsEnvironment::SetPredictionCommandNum( int iCommandNum )
{
	Log_Stub( LOG_VBox3D );
}

int Box3DPhysicsEnvironment::GetPredictionCommandNum()
{
	Log_Stub( LOG_VBox3D );
	return 0;
}

void Box3DPhysicsEnvironment::DoneReferencingPreviousCommands( int iCommandNum )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::RestorePredictedSimulation()
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::DestroyCollideOnDeadObjectFlush( CPhysCollide* )
{
	Log_Stub( LOG_VBox3D );
}

#if defined( GAME_GMOD_64X )
void Box3DPhysicsEnvironment::PreSave( const physprerestoreparams_t &params )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsEnvironment::PostSave()
{
	Log_Stub( LOG_VBox3D );
}
#endif
