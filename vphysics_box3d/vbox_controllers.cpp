//=================================================================================================
//
// Shadow + motion controllers
//
//=================================================================================================

#include "cbase.h"

#include "vbox_controllers.h"
#include "vbox_object.h"
#include "vbox_environment.h"
#include "vbox_surfaceprops.h"
#include "vbox_collide.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------
// Shadow controller. The object is driven kinematically toward the game's target, so dynamic
// bodies can never push it off course (doors, lifts, NPC-driven objects).
//-------------------------------------------------------------------------------------------------

// A contact normal.z below this (pointing down out of the surface the body rests on) counts as ground.
static constexpr float kGroundNormalZ = -0.7f;

// The other object in a body's contact; also reports whether pSelf is side A (manifold normals point A -> B).
static Box3DPhysicsObject *ContactOther( const b3ContactData &contact, Box3DPhysicsObject *pSelf, bool *pSelfIsA = nullptr )
{
	Box3DPhysicsObject *pA = static_cast< Box3DPhysicsObject * >( b3Body_GetUserData( b3Shape_GetBody( contact.shapeIdA ) ) );
	const bool bSelfIsA = pA == pSelf;
	if ( pSelfIsA )
		*pSelfIsA = bSelfIsA;
	return bSelfIsA
		? static_cast< Box3DPhysicsObject * >( b3Body_GetUserData( b3Shape_GetBody( contact.shapeIdB ) ) )
		: pA;
}

Box3DPhysicsShadowController::Box3DPhysicsShadowController( Box3DPhysicsObject *pObject, bool allowTranslation, bool allowRotation )
	: m_pObject( pObject )
	, m_allowTranslation( allowTranslation )
	, m_allowRotation( allowRotation )
{
	const b3BodyId bodyId = m_pObject->GetBodyID();

	m_savedBodyType = b3Body_GetType( bodyId );
	b3Body_SetType( bodyId, b3_kinematicBody );

	m_savedMaterialIndex = m_pObject->GetMaterialIndex();
	UseShadowMaterial( true );

	m_savedCallbackFlags = m_pObject->GetCallbackFlags();
	unsigned short uFlags = m_savedCallbackFlags | CALLBACK_SHADOW_COLLISION;
	uFlags &= ~CALLBACK_GLOBAL_FRICTION;
	uFlags &= ~CALLBACK_GLOBAL_COLLIDE_STATIC;
	m_pObject->SetCallbackFlags( uFlags );
	m_pObject->EnableDrag( false );

	m_pObject->GetPosition( &m_targetPosition, &m_targetAngles );
}

Box3DPhysicsShadowController::~Box3DPhysicsShadowController()
{
	const b3BodyId bodyId = m_pObject->GetBodyID();
	const bool bMarkedForDelete = ( m_pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) != 0;

	if ( !bMarkedForDelete )
	{
		m_pObject->SetCallbackFlags( m_savedCallbackFlags );
		m_pObject->EnableDrag( true );
		UseShadowMaterial( false );

		if ( b3Body_IsValid( bodyId ) )
		{
			b3Body_SetType( bodyId, m_savedBodyType );
			b3Body_SetAwake( bodyId, true );
		}
	}
}

void Box3DPhysicsShadowController::Update( const Vector &position, const QAngle &angles, float timeOffset )
{
	const Vector vecOldTarget = m_targetPosition;
	const QAngle angOldTarget = m_targetAngles;

	m_targetPosition = position;
	m_targetAngles = angles;
	m_secondsToArrival = Max( timeOffset, 0.0f );
	m_enabled = true;

	if ( ( position - vecOldTarget ).LengthSqr() < 1e-8f &&
		 ( Vector( angles.x, angles.y, angles.z ) - Vector( angOldTarget.x, angOldTarget.y, angOldTarget.z ) ).LengthSqr() < 1e-8f )
		return;

	m_pObject->Wake();
}

void Box3DPhysicsShadowController::MaxSpeed( float maxSpeed, float maxAngularSpeed )
{
	m_maxSpeed = maxSpeed;
	m_maxDampSpeed = maxSpeed;
	m_maxAngular = maxAngularSpeed;
	m_maxDampAngular = maxAngularSpeed;
}

void Box3DPhysicsShadowController::StepUp( float height )
{
	if ( height == 0.0f )
		return;

	Vector vecPos;
	QAngle angPos;
	m_pObject->GetPosition( &vecPos, &angPos );
	vecPos.z += height;
	m_pObject->SetPosition( vecPos, angPos, true );
}

void Box3DPhysicsShadowController::SetTeleportDistance( float teleportDistance )
{
	m_teleportDistance = teleportDistance;
}

bool Box3DPhysicsShadowController::AllowsTranslation()
{
	return m_allowTranslation;
}

bool Box3DPhysicsShadowController::AllowsRotation()
{
	return m_allowRotation;
}

void Box3DPhysicsShadowController::SetPhysicallyControlled( bool isPhysicallyControlled )
{
	m_isPhysicallyControlled = isPhysicallyControlled;
}

bool Box3DPhysicsShadowController::IsPhysicallyControlled()
{
	return m_isPhysicallyControlled;
}

void Box3DPhysicsShadowController::GetLastImpulse( Vector *pOut )
{
	if ( pOut )
		*pOut = m_lastImpulse;
}

void Box3DPhysicsShadowController::UseShadowMaterial( bool bUseShadowMaterial )
{
	const int nCurrent = m_pObject->GetMaterialIndex();
	const int nTarget = bUseShadowMaterial ? MATERIAL_INDEX_SHADOW : m_savedMaterialIndex;
	if ( nTarget != nCurrent )
		m_pObject->SetMaterialIndex( nTarget );
}

void Box3DPhysicsShadowController::ObjectMaterialChanged( int materialIndex )
{
	if ( materialIndex != MATERIAL_INDEX_SHADOW )
		m_savedMaterialIndex = materialIndex;
}

float Box3DPhysicsShadowController::GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut )
{
	if ( pPositionOut )
		*pPositionOut = m_targetPosition;
	if ( pAnglesOut )
		*pAnglesOut = m_targetAngles;

	return m_secondsToArrival;
}

float Box3DPhysicsShadowController::GetTeleportDistance()
{
	return m_teleportDistance;
}

void Box3DPhysicsShadowController::GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut )
{
	if ( pMaxSpeedOut )
		*pMaxSpeedOut = m_maxSpeed;
	if ( pMaxAngularSpeedOut )
		*pMaxAngularSpeedOut = m_maxAngular;
}

void Box3DPhysicsShadowController::OnPreSimulate( float flDeltaTime )
{
	if ( !m_enabled )
		return;

	const b3BodyId bodyId = m_pObject->GetBodyID();

	if ( m_secondsToArrival > 0.0f )
	{
		b3WorldTransform target;
		target.p = SourceToBox::Distance( m_targetPosition );
		target.q = SourceToBox::Angle( m_targetAngles );
		b3Body_SetTargetTransform( bodyId, target, m_secondsToArrival, true );
	}
	else
	{
		b3Body_SetTransform( bodyId, SourceToBox::Distance( m_targetPosition ), SourceToBox::Angle( m_targetAngles ) );
		b3Body_SetLinearVelocity( bodyId, b3Vec3{} );
		b3Body_SetAngularVelocity( bodyId, b3Vec3{} );
		b3Body_SetAwake( bodyId, true );
		m_enabled = false;
	}

	m_secondsToArrival = Max( m_secondsToArrival - flDeltaTime, 0.0f );
}

//-------------------------------------------------------------------------------------------------
// Player controller. The shadow is a real dynamic object;
// the controller servos its velocity to the game's position and limits how hard it may drive 
// into contacts (that limit is what makes pushing respect mass).
//-------------------------------------------------------------------------------------------------

static void ComputeController( Vector &vCurrentSpeed, const Vector &vDelta, const Vector &vMaxSpeed, float flScaleDelta, float flDamping, Vector *pOutImpulse )
{
	Vector vAcceleration = vDelta * flScaleDelta;

	if ( vCurrentSpeed.LengthSqr() < 1e-6f )
		vCurrentSpeed = vec3_origin;

	vAcceleration += vCurrentSpeed * -flDamping;

	for ( int i = 0; i < 3; i++ )
	{
		if ( fabsf( vAcceleration[ i ] ) < vMaxSpeed[ i ] )
			continue;
		vAcceleration[ i ] = vAcceleration[ i ] < 0.0f ? -vMaxSpeed[ i ] : vMaxSpeed[ i ];
	}

	vCurrentSpeed += vAcceleration;
	if ( pOutImpulse )
		*pOutImpulse = vAcceleration;
}

// Contact normals the controller may not drive into faster than the push speed limit.
class CNormalList
{
public:
	void AddNormal( const Vector &vecNormal )
	{
		if ( m_nCount == MAX_NORMALS )
			return;
		for ( int i = 0; i < m_nCount; i++ )
		{
			if ( DotProduct( m_Normals[ i ], vecNormal ) > 0.99f )
				return;
		}
		m_Normals[ m_nCount++ ] = vecNormal;
	}

	Vector ClampVector( const Vector &vecIn, float flLimitVel )
	{
		if ( m_nCount > 2 )
		{
			for ( int i = 0; i < m_nCount; i++ )
			{
				if ( DotProduct( vecIn, m_Normals[ i ] ) > 0.0f )
					return vec3_origin;
			}
		}
		else if ( m_nCount == 2 )
		{
			Vector vecCrease;
			CrossProduct( m_Normals[ 0 ], m_Normals[ 1 ], vecCrease );
			return vecCrease * DotProduct( vecIn, vecCrease );
		}
		else if ( m_nCount == 1 )
		{
			const float flDot = DotProduct( vecIn, m_Normals[ 0 ] );
			if ( flDot > flLimitVel )
				return vecIn + m_Normals[ 0 ] * ( flLimitVel - flDot );
		}
		return vecIn;
	}

private:
	static constexpr int MAX_NORMALS = 8;
	Vector	m_Normals[ MAX_NORMALS ];
	int		m_nCount = 0;
};

Box3DPhysicsPlayerController::Box3DPhysicsPlayerController( Box3DPhysicsObject *pObject )
{
	SetObjectInternal( pObject );
}

Box3DPhysicsPlayerController::~Box3DPhysicsPlayerController()
{
	SetObjectInternal( nullptr );
}

void Box3DPhysicsPlayerController::SetObjectInternal( Box3DPhysicsObject *pObject )
{
	if ( m_pObject == pObject )
		return;

	if ( m_pObject )
	{
		const b3BodyId oldId = m_pObject->GetBodyID();
		if ( b3Body_IsValid( oldId ) )
			b3Body_SetAngularDamping( oldId, m_flSavedAngularDamping );
		m_pObject->SetCallbackFlags( m_pObject->GetCallbackFlags() & ~CALLBACK_IS_PLAYER_CONTROLLER );
	}

	m_pObject = pObject;
	SetGround( nullptr );

	if ( m_pObject )
	{
		const b3BodyId bodyId = m_pObject->GetBodyID();

		// IVP replaces the controlled core's rot_speed_damp_factor with (100,100,100); heavy
		// angular damping is the box3d equivalent.
		// Which hull collides is the game's business (EnableCollisions via SetVCollisionState).
		m_pObject->EnableDrag( false );
		m_flSavedAngularDamping = b3Body_GetAngularDamping( bodyId );
		b3Body_SetAngularDamping( bodyId, 100.0f );

		m_pObject->SetCallbackFlags( m_pObject->GetCallbackFlags() | CALLBACK_IS_PLAYER_CONTROLLER );
	}
}

void Box3DPhysicsPlayerController::SetGround( Box3DPhysicsObject *pGround )
{
	m_pGround = pGround;
}

void Box3DPhysicsPlayerController::ClearGround( Box3DPhysicsObject *pObject )
{
	if ( m_pGround == pObject )
		m_pGround = nullptr;
}

void Box3DPhysicsPlayerController::Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground )
{
	m_updatedSinceLast = true;

	// If the object hasn't moved, abort.
	if ( ( velocity - m_currentSpeed ).LengthSqr() < 1e-6f && ( position - m_targetPosition ).LengthSqr() < 1e-6f )
		return;

	m_targetPosition = position;
	m_currentSpeed = velocity;
	m_secondsToArrival = secondsToArrival < 0.0f ? 0.0f : secondsToArrival;

	if ( m_pObject )
		b3Body_SetAwake( m_pObject->GetBodyID(), true );

	m_enable = true;
	if ( velocity.LengthSqr() <= 0.1f )
	{
		// No input velocity, just go where physics takes you.
		m_enable = false;
		ground = nullptr;
	}
	else
	{
		MaxSpeed( velocity );
	}

	SetGround( static_cast< Box3DPhysicsObject * >( ground ) );
	if ( m_pGround )
		m_pGround->WorldToLocal( &m_groundPosition, m_targetPosition );
}

void Box3DPhysicsPlayerController::SetEventHandler( IPhysicsPlayerControllerEvent *handler )
{
	m_pHandler = handler;
}

static bool IsControlledByGame( Box3DPhysicsObject *pObject )
{
	IPhysicsShadowController *pShadow = pObject->GetShadowController();
	if ( pShadow && !pShadow->IsPhysicallyControlled() )
		return true;

	return ( pObject->GetCallbackFlags() & CALLBACK_IS_PLAYER_CONTROLLER ) != 0;
}

bool Box3DPhysicsPlayerController::IsInContact()
{
	if ( !m_pObject || !m_pObject->IsCollisionEnabled() )
		return false;

	b3ContactData contacts[ 32 ];
	const int nCount = b3Body_GetContactData( m_pObject->GetBodyID(), contacts, 32 );
	for ( int i = 0; i < nCount; i++ )
	{
		bool bTouching = false;
		for ( int j = 0; j < contacts[ i ].manifoldCount; j++ )
			bTouching |= contacts[ i ].manifolds[ j ].pointCount > 0;
		if ( !bTouching )
			continue;

		Box3DPhysicsObject *pOther = ContactOther( contacts[ i ], m_pObject );
		if ( !pOther || !pOther->IsCollisionEnabled() || !pOther->IsMoveable() )
			continue;

		// Skip game-controlled shadow objects; we want physically simulated contact.
		if ( IsControlledByGame( pOther ) )
			continue;

		return true;
	}
	return false;
}

void Box3DPhysicsPlayerController::MaxSpeed( const Vector &maxVelocity )
{
	if ( !m_pObject )
		return;

	Vector vCurrentVelocity;
	m_pObject->GetVelocity( &vCurrentVelocity, nullptr );

	Vector vDirection = maxVelocity;
	const float flLength = VectorNormalize( vDirection );

	Vector vAvailable = maxVelocity;
	const float flDot = DotProduct( vDirection, vCurrentVelocity );
	if ( flDot > 0.0f )
		vAvailable -= vDirection * ( flDot * flLength );

	m_maxSpeed = Vector( fabsf( vAvailable.x ), fabsf( vAvailable.y ), fabsf( vAvailable.z ) );
}

void Box3DPhysicsPlayerController::SetObject( IPhysicsObject *pObject )
{
	SetObjectInternal( static_cast< Box3DPhysicsObject * >( pObject ) );
}

int Box3DPhysicsPlayerController::GetShadowPosition( Vector *position, QAngle *angles )
{
	if ( m_pObject )
		m_pObject->GetPosition( position, angles );
	return 1;
}

void Box3DPhysicsPlayerController::StepUp( float height )
{
	if ( height == 0.0f || !m_pObject )
		return;

	Vector vPos;
	QAngle qAngles;
	m_pObject->GetPosition( &vPos, &qAngles );
	vPos.z += height;
	m_pObject->SetPosition( vPos, qAngles, true );
}

void Box3DPhysicsPlayerController::Jump()
{
}

void Box3DPhysicsPlayerController::GetShadowVelocity( Vector *velocity )
{
	if ( !velocity || !m_pObject )
		return;

	m_pObject->GetVelocity( velocity, nullptr );

	if ( m_pGround )
	{
		Vector vGroundPoint;
		m_pGround->LocalToWorld( &vGroundPoint, m_groundPosition );
		Vector vBaseVelocity;
		m_pGround->GetVelocityAtPoint( vGroundPoint, &vBaseVelocity );
		*velocity -= vBaseVelocity;
	}
}

IPhysicsObject *Box3DPhysicsPlayerController::GetObject()
{
	return m_pObject;
}

void Box3DPhysicsPlayerController::GetLastImpulse( Vector *pOut )
{
	if ( pOut )
		*pOut = m_lastImpulse;
}

void Box3DPhysicsPlayerController::SetPushMassLimit( float maxPushMass )
{
	m_pushableMassLimit = maxPushMass;
}

void Box3DPhysicsPlayerController::SetPushSpeedLimit( float maxPushSpeed )
{
	m_pushableSpeedLimit = maxPushSpeed;
}

float Box3DPhysicsPlayerController::GetPushMassLimit()
{
	return m_pushableMassLimit;
}

float Box3DPhysicsPlayerController::GetPushSpeedLimit()
{
	return m_pushableSpeedLimit;
}

bool Box3DPhysicsPlayerController::WasFrozen()
{
	return false;
}

int Box3DPhysicsPlayerController::TryTeleportObject()
{
	if ( m_pHandler && !m_pHandler->ShouldMoveTo( m_pObject, m_targetPosition ) )
		return 0;

	QAngle qAngles;
	m_pObject->GetPosition( nullptr, &qAngles );

	if ( m_pObject->IsCollisionEnabled() )
	{
		m_pObject->EnableCollisions( false );
		m_pObject->SetPosition( m_targetPosition, qAngles, true );
		m_pObject->EnableCollisions( true );
	}
	else
	{
		m_pObject->SetPosition( m_targetPosition, qAngles, true );
	}
	return 1;
}

void Box3DPhysicsPlayerController::OnPreSimulate( float flDeltaTime )
{
	if ( !m_pObject || !m_enable || flDeltaTime <= 0.0f )
		return;

	const b3BodyId bodyId = m_pObject->GetBodyID();
	if ( !b3Body_IsAwake( bodyId ) )
		return;

	Vector vPosition, vSpeed;
	m_pObject->GetPosition( &vPosition, nullptr );
	m_pObject->GetVelocity( &vSpeed, nullptr );

	// Ride a moving ground object: keep the target at the stored ground-local point and servo in
	// the ground's reference frame.
	Vector vBaseVelocity = vec3_origin;
	if ( m_pGround )
	{
		m_pGround->LocalToWorld( &m_targetPosition, m_groundPosition );
		m_pGround->GetVelocityAtPoint( m_targetPosition, &vBaseVelocity );
		vSpeed -= vBaseVelocity;
	}

	const Vector vDeltaPos = m_targetPosition - vPosition;

	if ( vDeltaPos.LengthSqr() > m_maxDeltaPosition * m_maxDeltaPosition )
	{
		if ( TryTeleportObject() )
		{
			// IVP modifies core speed in place, so the ground base-velocity subtraction
			// persists across the teleport.
			b3Body_SetLinearVelocity( bodyId, SourceToBox::Distance( vSpeed ) );
			return;
		}
	}

	float flFraction = 1.0f;
	if ( m_secondsToArrival > 0.0f )
		flFraction = Min( flDeltaTime / m_secondsToArrival, 1.0f );

	if ( !m_updatedSinceLast )
	{
		// No game update since the last step: the error estimate is stale, so cap the impulse to
		// the last known good one.
		const float flLen = m_lastImpulse.Length();
		ComputeController( vSpeed, vDeltaPos, Vector( flLen, flLen, flLen ), flFraction / flDeltaTime, m_dampFactor, nullptr );
	}
	else
	{
		ComputeController( vSpeed, vDeltaPos, m_maxSpeed, flFraction / flDeltaTime, m_dampFactor, &m_lastImpulse );
	}
	vSpeed += vBaseVelocity;
	m_updatedSinceLast = false;

	// Limit how fast the controller drives into its contacts: unmoveable or too-heavy objects can't
	// be pushed at all, everything else at most at the push speed limit.
	const float flMass = m_pObject->GetMass();
	const float flInvMass = flMass > 0.0f ? 1.0f / flMass : 0.0f;
	float flLimitVel = m_pushableSpeedLimit;
	bool bGround = false;
	CNormalList normalList;

	b3ContactData contacts[ 32 ];
	const int nCount = b3Body_GetContactData( bodyId, contacts, 32 );
	for ( int i = 0; i < nCount; i++ )
	{
		bool bSelfIsA;
		Box3DPhysicsObject *pOther = ContactOther( contacts[ i ], m_pObject, &bSelfIsA );

		for ( int j = 0; j < contacts[ i ].manifoldCount; j++ )
		{
			const b3Manifold &manifold = contacts[ i ].manifolds[ j ];
			if ( manifold.pointCount <= 0 )
				continue;

			// The manifold normal points A -> B; orient it from the player into the other object.
			Vector vecNormal = BoxToSource::Unitless( manifold.normal );
			if ( !bSelfIsA )
				vecNormal = -vecNormal;

			if ( vecNormal.z < kGroundNormalZ )
				bGround = true;

			if ( vecNormal.z > -0.99f )
			{
				if ( !pOther || !pOther->IsMoveable() || pOther->GetMass() > m_pushableMassLimit )
					flLimitVel = 0.0f;

				float flImpulse = 0.0f;
				for ( int p = 0; p < manifold.pointCount; p++ )
					flImpulse = Max( flImpulse, manifold.points[ p ].totalNormalImpulse );

				const float flPushSpeed = DotProduct( vSpeed, vecNormal );
				const float flContactVel = BoxToSource::Distance( flImpulse * flInvMass );
				if ( flPushSpeed + flContactVel > flLimitVel )
					normalList.AddNormal( vecNormal );
			}
		}
	}

	const Vector vLimit = normalList.ClampVector( vSpeed, flLimitVel ) - vSpeed;
	vSpeed += vLimit;
	m_lastImpulse += vLimit;

	if ( bGround )
	{
		// On the ground, press down with exactly one step of gravity and no more.
		const b3Vec3 vGravity = b3World_GetGravity( m_pObject->GetEnvironment()->GetWorldId() );
		const float flGravDt = BoxToSource::Distance( sqrtf( b3Dot( vGravity, vGravity ) ) ) * flDeltaTime;
		if ( m_lastImpulse.z <= 0.0f )
		{
			const float flDelta = -flGravDt - m_lastImpulse.z;
			vSpeed.z += flDelta;
			m_lastImpulse.z += flDelta;
		}
	}

	b3Body_SetLinearVelocity( bodyId, SourceToBox::Distance( vSpeed ) );

	m_secondsToArrival = Max( m_secondsToArrival - flDeltaTime, 0.0f );
}

//-------------------------------------------------------------------------------------------------
// Motion controller: each step the game's IMotionEvent computes a velocity/force for every attached
// object and we apply it. This is the gravity gun's grab.
//-------------------------------------------------------------------------------------------------

Box3DPhysicsMotionController::Box3DPhysicsMotionController( IMotionEvent *pHandler )
	: m_pHandler( pHandler )
{
}

void Box3DPhysicsMotionController::SetEventHandler( IMotionEvent *pHandler )
{
	m_pHandler = pHandler;
}

void Box3DPhysicsMotionController::AttachObject( IPhysicsObject *pObject, bool checkIfAlreadyAttached )
{
	if ( !pObject || pObject->IsStatic() )
		return;

	Box3DPhysicsObject *pPhysicsObject = static_cast< Box3DPhysicsObject * >( pObject );
	if ( checkIfAlreadyAttached && m_Objects.Find( pPhysicsObject ) != m_Objects.InvalidIndex() )
		return;

	m_Objects.AddToTail( pPhysicsObject );
}

void Box3DPhysicsMotionController::DetachObject( IPhysicsObject *pObject )
{
	m_Objects.FindAndRemove( static_cast< Box3DPhysicsObject * >( pObject ) );
}

int Box3DPhysicsMotionController::CountObjects()
{
	return m_Objects.Count();
}

void Box3DPhysicsMotionController::GetObjects( IPhysicsObject **pObjectList )
{
	for ( int i = 0; i < m_Objects.Count(); i++ )
		pObjectList[ i ] = m_Objects[ i ];
}

void Box3DPhysicsMotionController::ClearObjects()
{
	m_Objects.RemoveAll();
}

void Box3DPhysicsMotionController::WakeObjects()
{
	for ( int i = 0; i < m_Objects.Count(); i++ )
		m_Objects[ i ]->Wake();
}

void Box3DPhysicsMotionController::SetPriority( priority_t )
{
}

void Box3DPhysicsMotionController::OnPreSimulate( float flDeltaTime )
{
	if ( !m_pHandler )
		return;

	for ( int i = 0; i < m_Objects.Count(); i++ )
	{
		Box3DPhysicsObject *pObject = m_Objects[ i ];
		if ( !pObject->IsMoveable() )
			continue;

		Vector vecLinear = vec3_origin;
		AngularImpulse angLocalAngular = vec3_origin;
		const IMotionEvent::simresult_e result = m_pHandler->Simulate( this, pObject, flDeltaTime, vecLinear, angLocalAngular );

		vecLinear *= flDeltaTime;
		angLocalAngular *= flDeltaTime;

		// The linear value is local or global depending on the result type; the angular value is
		// always in the object's local space. AddVelocity takes it as-is; ApplyTorqueCenter takes
		// a world-space torque impulse, so rotate it out for the force path.
		Vector vecWorldLinear = vecLinear;
		if ( result == IMotionEvent::SIM_LOCAL_ACCELERATION || result == IMotionEvent::SIM_LOCAL_FORCE )
			pObject->LocalToWorldVector( &vecWorldLinear, vecLinear );

		Vector vecWorldAngular;
		pObject->LocalToWorldVector( &vecWorldAngular, angLocalAngular );

		switch ( result )
		{
			case IMotionEvent::SIM_GLOBAL_ACCELERATION:
			case IMotionEvent::SIM_LOCAL_ACCELERATION:
				pObject->AddVelocity( &vecWorldLinear, &angLocalAngular );
				break;

			case IMotionEvent::SIM_GLOBAL_FORCE:
			case IMotionEvent::SIM_LOCAL_FORCE:
				pObject->ApplyForceCenter( vecWorldLinear );
				pObject->ApplyTorqueCenter( vecWorldAngular );
				break;

			case IMotionEvent::SIM_NOTHING:
			default:
				break;
		}
	}
}

//-------------------------------------------------------------------------------------------------
// Fluid controller: water buoyancy, drag and current. No fluid model in Box3D, so each step we find the
// bodies overlapping the fluid volume and apply buoyancy. Submerged volume from the body AABB.
//-------------------------------------------------------------------------------------------------

// Store the world-space surface plane in the fluid object's local space so it tracks the object.
static cplane_t PlaneToLocalSpace( Box3DPhysicsObject *pFluidObject, const Vector4D &worldSurfacePlane )
{
	const cplane_t worldPlane = { worldSurfacePlane.AsVector3D(), worldSurfacePlane[ 3 ] };

	matrix3x4_t objectToWorld;
	pFluidObject->GetPositionMatrix( &objectToWorld );

	cplane_t localPlane;
	MatrixITransformPlane( objectToWorld, worldPlane, localPlane );
	return localPlane;
}

// Object volume (m^3) and tight world AABB. Uses the pristine collision hulls, not the inflated sim hulls
// (which over-report volume), and builds the AABB here since b3Body_ComputeAABB returns the fat one.
static float ComputeBodyBuoyancy( Box3DPhysicsObject *pObject, b3AABB *pAABB )
{
	const b3BodyId bodyId = pObject->GetBodyID();
	const b3WorldTransform wt = b3Body_GetTransform( bodyId );
	const b3Transform xf = { b3ToVec3( wt.p ), wt.q };

	float flVolume = 0.0f;
	bool bHasBounds = false;
	b3AABB bounds = {};

	const CPhysCollide *pCollide = pObject->GetCollide();
	if ( pCollide )
	{
		for ( int i = 0; i < pCollide->m_Convexes.Count(); i++ )
		{
			const b3HullData *pHull = pCollide->m_Convexes[ i ]->m_pHull;
			if ( !pHull )
				continue;
			flVolume += b3ComputeHullMass( pHull, 1.0f ).mass;
			const b3AABB hullAABB = b3ComputeHullAABB( pHull, xf );
			bounds = bHasBounds ? b3AABB_Union( bounds, hullAABB ) : hullAABB;
			bHasBounds = true;
		}
	}

	// Spheres (no CPhysCollide) fall back to the body's own shapes.
	if ( !bHasBounds )
	{
		const int nCount = b3Body_GetShapeCount( bodyId );
		CUtlVector< b3ShapeId > shapes;
		shapes.SetCount( nCount );
		b3Body_GetShapes( bodyId, shapes.Base(), nCount );
		for ( int i = 0; i < nCount; i++ )
		{
			if ( b3Shape_GetType( shapes[ i ] ) != b3_sphereShape )
				continue;
			const b3Sphere sphere = b3Shape_GetSphere( shapes[ i ] );
			flVolume += ( 4.0f / 3.0f ) * 3.14159265f * sphere.radius * sphere.radius * sphere.radius;
			const b3Vec3 center = b3TransformPoint( xf, sphere.center );
			const b3Vec3 rad = { sphere.radius, sphere.radius, sphere.radius };
			const b3AABB sphAABB = { b3Sub( center, rad ), b3Add( center, rad ) };
			bounds = bHasBounds ? b3AABB_Union( bounds, sphAABB ) : sphAABB;
			bHasBounds = true;
		}
	}

	if ( pAABB )
		*pAABB = bounds;
	return flVolume;
}

namespace
{
	struct FluidQuery
	{
		Box3DPhysicsObject *pFluidObject;
		CUtlVector< Box3DPhysicsObject * > *pOut;
	};

	// Collect the distinct dynamic bodies overlapping the fluid's AABB.
	bool FluidOverlapFcn( b3ShapeId shapeId, void *context )
	{
		FluidQuery *pQuery = static_cast< FluidQuery * >( context );
		Box3DPhysicsObject *pObject = static_cast< Box3DPhysicsObject * >( b3Body_GetUserData( b3Shape_GetBody( shapeId ) ) );
		if ( pObject && pObject != pQuery->pFluidObject && !pObject->IsStatic() &&
			 pQuery->pOut->Find( pObject ) == pQuery->pOut->InvalidIndex() )
			pQuery->pOut->AddToTail( pObject );
		return true;
	}
}

Box3DPhysicsFluidController::Box3DPhysicsFluidController( Box3DPhysicsObject *pFluidObject, const fluidparams_t *pParams )
	: m_pFluidObject( pFluidObject )
	, m_Params( *pParams )
	, m_LocalPlane( PlaneToLocalSpace( pFluidObject, pParams->surfacePlane ) )
{
	m_pFluidObject->BecomeTrigger();
}

Box3DPhysicsFluidController::~Box3DPhysicsFluidController()
{
	if ( m_pFluidObject )
		m_pFluidObject->RemoveTrigger();
}

void Box3DPhysicsFluidController::SetGameData( void *pGameData )		{ m_Params.pGameData = pGameData; }
void *Box3DPhysicsFluidController::GetGameData() const				{ return m_Params.pGameData; }
int Box3DPhysicsFluidController::GetContents() const				{ return m_Params.contents; }

void Box3DPhysicsFluidController::GetSurfacePlane( Vector *pNormal, float *pDist ) const
{
	const cplane_t worldPlane = GetWorldSurfacePlane();
	if ( pNormal )
		*pNormal = worldPlane.normal;
	if ( pDist )
		*pDist = worldPlane.dist;
}

// Water density in kg/m^3 from surfaceprops, defaulting to 1000 if unset.
float Box3DPhysicsFluidController::GetDensity() const
{
	surfacedata_t *pSurface = m_pFluidObject
		? Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData( m_pFluidObject->GetMaterialIndex() )
		: nullptr;
	return ( pSurface && pSurface->physics.density > 0.0f ) ? pSurface->physics.density : 1000.0f;
}

void Box3DPhysicsFluidController::WakeAllSleepingObjects()
{
	for ( int i = 0; i < m_ObjectsInFluid.Count(); i++ )
		m_ObjectsInFluid[ i ]->Wake();
}

void Box3DPhysicsFluidController::DetachObject( Box3DPhysicsObject *pObject )
{
	if ( pObject == m_pFluidObject )
		m_pFluidObject = nullptr;
	m_ObjectsInFluid.FindAndRemove( pObject );
}

cplane_t Box3DPhysicsFluidController::GetWorldSurfacePlane() const
{
	if ( !m_pFluidObject )
		return cplane_t{};

	matrix3x4_t objectToWorld;
	m_pFluidObject->GetPositionMatrix( &objectToWorld );

	cplane_t worldPlane;
	MatrixTransformPlane( objectToWorld, m_LocalPlane, worldPlane );
	return worldPlane;
}

void Box3DPhysicsFluidController::OnPreSimulate( float flDeltaTime )
{
	if ( !m_pFluidObject || flDeltaTime <= 0.0f )
		return;

	const b3WorldId worldId = m_pFluidObject->GetEnvironment()->GetWorldId();

	CUtlVector< Box3DPhysicsObject * > overlapping;
	FluidQuery query = { m_pFluidObject, &overlapping };
	b3World_OverlapAABB( worldId, b3Body_ComputeAABB( m_pFluidObject->GetBodyID() ), b3DefaultQueryFilter(), FluidOverlapFcn, &query );

	// The bodies actually below the surface this step; diffed against m_ObjectsInFluid for touch events.
	CUtlVector< Box3DPhysicsObject * > nowInFluid;

	const cplane_t worldPlane = GetWorldSurfacePlane();
	const b3Vec3 n = SourceToBox::Unitless( worldPlane.normal );		// unit, points up out of the water
	const float flPlaneDist = SourceToBox::Distance( worldPlane.dist );	// water is where dot(n,x) < planeDist
	const b3Vec3 gravity = b3World_GetGravity( worldId );
	const b3Vec3 current = SourceToBox::Distance( m_Params.currentVelocity );
	const float flWaterDensity = GetDensity();
	const float flLinearDrag = m_Params.damping;
	const float flAngularDrag = 0.1f;

	for ( int i = 0; i < overlapping.Count(); i++ )
	{
		Box3DPhysicsObject *pObject = overlapping[ i ];
		if ( pObject->GetShadowController() || !( pObject->GetCallbackFlags() & CALLBACK_DO_FLUID_SIMULATION ) )
			continue;

		const b3BodyId body = pObject->GetBodyID();
		const float flInvMass = b3Body_GetInverseMass( body );
		b3AABB aabb;
		const float flTotalVolume = ComputeBodyBuoyancy( pObject, &aabb );
		if ( flInvMass <= 0.0f || flTotalVolume <= 0.0f )
			continue;

		// Submerged fraction of the AABB along the surface normal.
		const b3Vec3 c = b3MulSV( 0.5f, b3Add( aabb.lowerBound, aabb.upperBound ) );
		const b3Vec3 half = b3MulSV( 0.5f, b3Sub( aabb.upperBound, aabb.lowerBound ) );
		const float flR = fabsf( n.x ) * half.x + fabsf( n.y ) * half.y + fabsf( n.z ) * half.z;
		const float flCN = b3Dot( n, c );
		const float flLo = flCN - flR, flHi = flCN + flR;
		if ( flHi <= flLo )
			continue;
		const float flTop = Min( flHi, flPlaneDist );
		const float flSubLen = clamp( flTop - flLo, 0.0f, flHi - flLo );
		if ( flSubLen <= 0.0f )
			continue;

		nowInFluid.AddToTail( pObject );

		const float flFraction = flSubLen / ( flHi - flLo );
		const float flSubmergedVolume = flTotalVolume * flFraction;

		// Centre of buoyancy: midpoint of the submerged span, relative to the centre of mass.
		const float flMidSub = 0.5f * ( flLo + flTop );
		const b3Vec3 cob = b3Add( c, b3MulSV( flMidSub - flCN, n ) );
		const b3Vec3 relCob = b3Sub( cob, b3Body_GetWorldCenterOfMass( body ) );

		const float flFluidDensity = flWaterDensity * pObject->BuoyancyRatio();

		// Buoyancy = -fluidDensity * submergedVolume * gravity * dt.
		const b3Vec3 buoyImpulse = b3MulSV( -flFluidDensity * flSubmergedVolume * flDeltaTime, gravity );

		// Velocity of the centre of buoyancy relative to the fluid.
		const b3Vec3 linVel = b3Body_GetLinearVelocity( body );
		const b3Vec3 angVel = b3Body_GetAngularVelocity( body );
		const b3Vec3 relVel = b3Sub( current, b3Add( linVel, b3Cross( angVel, relCob ) ) );

		// Quadratic drag; frontal area from the AABB.
		const b3Vec3 size = b3Sub( aabb.upperBound, aabb.lowerBound );
		const float flRelSq = b3Dot( relVel, relVel );
		b3Vec3 dragImpulse = { 0.0f, 0.0f, 0.0f };
		if ( flRelSq > 1e-12f )
		{
			const float flArea = ( fabsf( relVel.x ) * size.y * size.z + fabsf( relVel.y ) * size.z * size.x + fabsf( relVel.z ) * size.x * size.y ) / sqrtf( flRelSq );
			dragImpulse = b3MulSV( 0.5f * flFluidDensity * flLinearDrag * flArea * flDeltaTime * sqrtf( flRelSq ), relVel );

			// Clamp so drag can't reverse the velocity.
			const float flLinVelSq = b3Dot( linVel, linVel );
			const b3Vec3 dragDv = b3MulSV( flInvMass, dragImpulse );
			const float flDragDvSq = b3Dot( dragDv, dragDv );
			if ( flDragDvSq > flLinVelSq && flDragDvSq > 0.0f )
				dragImpulse = b3MulSV( sqrtf( flLinVelSq / flDragDvSq ), dragImpulse );
		}

		// wake=false: buoyancy cancels gravity at rest, so forcing wake would stop props ever sleeping.
		const b3Vec3 linImpulse = b3Add( buoyImpulse, dragImpulse );
		b3Body_ApplyLinearImpulseToCenter( body, linImpulse, false );
		b3Body_ApplyAngularImpulse( body, b3Cross( relCob, linImpulse ), false );

		// Angular drag: -angularDrag * fraction * dt * avgWidth^2 * mass * angVel.
		const float flL = ( size.x + size.y + size.z ) / 3.0f;
		b3Body_ApplyAngularImpulse( body, b3MulSV( -flAngularDrag * flFraction * flDeltaTime * flL * flL / flInvMass, angVel ), false );
	}

	// Fluid touch events: diff this step's submerged set against last step's.
	if ( IPhysicsCollisionEvent *pEvent = m_pFluidObject->GetEnvironment()->GetCollisionEvent() )
	{
		for ( int i = 0; i < nowInFluid.Count(); i++ )
			if ( m_ObjectsInFluid.Find( nowInFluid[ i ] ) == m_ObjectsInFluid.InvalidIndex() )
				pEvent->FluidStartTouch( nowInFluid[ i ], this );
		for ( int i = 0; i < m_ObjectsInFluid.Count(); i++ )
			if ( nowInFluid.Find( m_ObjectsInFluid[ i ] ) == nowInFluid.InvalidIndex() )
				pEvent->FluidEndTouch( m_ObjectsInFluid[ i ], this );
	}

	m_ObjectsInFluid.RemoveAll();
	m_ObjectsInFluid.AddVectorToTail( nowInFluid );
}
