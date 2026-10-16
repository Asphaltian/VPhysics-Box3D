#include "physics_player_controller.h"

#include "conversion.h"
#include "physics_object.h"

#include <algorithm>
#include <cmath>

static constexpr float PLAYER_ROT_DAMPING = 100.0f;
static constexpr float MAX_POSITION_ERROR = 24.0f * SOURCE_TO_IVP;
static constexpr double MIN_TARGET_CHANGE = 1e-6;
static constexpr float MIN_ACTIVE_SPEED_SQUARED = 0.1f;
static constexpr float MIN_SPEED_SQUARED = 1e-6f;
static constexpr float MIN_DIRECTION_LENGTH_SQUARED = 1e-10f;
static constexpr float FLOOR_NORMAL_Z = -0.99f;
static constexpr float DUPLICATE_PLANE_DOT = 0.99f;
static constexpr int MAX_CLIP_PLANES = 8;

CPhysicsPlayerController::CPhysicsPlayerController( CPhysicsObject *pObject )
{
	AttachObject( pObject );
}

CPhysicsPlayerController::~CPhysicsPlayerController()
{
	DetachObject();
}

void CPhysicsPlayerController::AttachObject( CPhysicsObject *pObject )
{
	m_pObject = pObject;
	pObject->EnableDrag( false );
	m_flSavedRotDamping = pObject->GetRotDamping();
	pObject->SetRotDamping( PLAYER_ROT_DAMPING );
	pObject->AttachController( this );
	pObject->SetCallbackFlags( pObject->GetCallbackFlags() | CALLBACK_IS_PLAYER_CONTROLLER );
}

void CPhysicsPlayerController::DetachObject()
{
	if ( !m_pObject )
		return;

	m_pObject->SetRotDamping( m_flSavedRotDamping );
	m_pObject->SetCallbackFlags( m_pObject->GetCallbackFlags() & ~CALLBACK_IS_PLAYER_CONTROLLER );
	m_pObject->DetachController( this );
	m_pObject = nullptr;
	m_pGround = nullptr;
}

void CPhysicsPlayerController::Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool, IPhysicsObject *ground )
{
	const RealVec3_t target = ToReal( SourceToIVP( position ) );
	const b3Vec3 targetVelocity = SourceToIVP( velocity );
	m_bUpdated = true;

	if ( double( b3LengthSquared( b3Sub( targetVelocity, m_TargetVelocity ) ) ) < MIN_TARGET_CHANGE && double( LengthSquared( target - m_TargetPosition ) ) < MIN_TARGET_CHANGE )
		return;

	m_TargetPosition = target;
	m_TargetVelocity = targetVelocity;
	m_flSecondsToArrival = std::max( secondsToArrival, 0.0f );
	m_pObject->Wake();
	m_bActive = true;

	if ( velocity.LengthSqr() > MIN_ACTIVE_SPEED_SQUARED )
	{
		MaxSpeed( velocity );
	}
	else
	{
		m_bActive = false;
		ground = nullptr;
	}

	m_pGround = static_cast<CPhysicsObject *>( ground );
	if ( m_pGround )
		m_GroundPosition = m_pGround->IVPToCore( ToFloat( m_TargetPosition - m_pGround->GetCorePosition() ) );
}

bool CPhysicsPlayerController::IsInContact()
{
	if ( !m_pObject->IsCollisionEnabled() )
		return false;

	for ( const CPhysicsEnvironment::ContactNormal_t &contact : m_pObject->GetEnvironment()->GetContactNormals( m_pObject ) )
	{
		const CPhysicsObject *pOther = contact.m_pOther;
		if ( !pOther->IsCollisionEnabled() || !pOther->IsMoveable() )
			continue;

		IPhysicsShadowController *pShadow = pOther->GetShadowController();
		const bool bGameControlled = ( pShadow && !pShadow->IsPhysicallyControlled() ) || ( pOther->GetCallbackFlags() & CALLBACK_IS_PLAYER_CONTROLLER );
		if ( !bGameControlled )
			return true;
	}

	return false;
}

void CPhysicsPlayerController::MaxSpeed( const Vector &maxVelocity )
{
	const b3Vec3 v = SourceToIVP( maxVelocity );
	const b3Vec3 speed = m_pObject->GetCoreVelocity();
	float flLength = 0.0f;
	b3Vec3 direction = v;
	if ( b3LengthSquared( v ) >= MIN_DIRECTION_LENGTH_SQUARED )
	{
		flLength = b3Length( v );
		direction = b3MulSV( 1.0f / flLength, v );
	}

	const float flDot = ( direction.y * speed.y + direction.x * speed.x ) + direction.z * speed.z;
	const b3Vec3 limit = flDot > 0.0f ? b3MulSub( v, flDot * flLength, direction ) : v;
	m_MaxSpeed = b3Abs( limit );
}

void CPhysicsPlayerController::SetObject( IPhysicsObject *pObject )
{
	if ( pObject == m_pObject )
		return;

	DetachObject();
	AttachObject( static_cast<CPhysicsObject *>( pObject ) );
}

int CPhysicsPlayerController::GetShadowPosition( Vector *position, QAngle *angles )
{
	return m_pObject->GetShadowPosition( position, angles );
}

void CPhysicsPlayerController::StepUp( float height )
{
	if ( height != 0.0f )
		m_pObject->StepUp( height );
}

void CPhysicsPlayerController::GetShadowVelocity( Vector *velocity )
{
	b3Vec3 v = b3Add( m_pObject->GetCoreVelocity(), m_pObject->GetPendingVelocity() );
	if ( m_pGround )
		v = b3Sub( v, m_pGround->GetCoreSurfaceVelocity( m_GroundPosition ) );
	*velocity = ToSource( IVPToSource( v ) );
}

IPhysicsObject *CPhysicsPlayerController::GetObject()
{
	return m_pObject;
}

void CPhysicsPlayerController::GetLastImpulse( Vector *pOut )
{
	*pOut = ToSource( IVPToSource( m_LastImpulse ) );
}

void CPhysicsPlayerController::Simulate( CPhysicsObject *pObject, float flTimestep )
{
	if ( !m_bActive )
		return;

	b3Vec3 speed = pObject->GetCoreVelocity();
	b3Vec3 groundVelocity = b3Vec3_zero;
	if ( m_pGround )
	{
		m_TargetPosition = m_pGround->TransformCorePoint( m_GroundPosition );
		groundVelocity = m_pGround->GetCoreSurfaceVelocity( m_GroundPosition );
		speed = b3Sub( speed, groundVelocity );
	}

	const b3Vec3 delta = ToFloat( m_TargetPosition - pObject->GetCoreOrigin() );
	if ( b3LengthSquared( delta ) > MAX_POSITION_ERROR * MAX_POSITION_ERROR )
	{
		if ( !m_pHandler || m_pHandler->ShouldMoveTo( pObject, IVPToSourcePosition( m_TargetPosition ) ) )
		{
			pObject->MoveTo( m_TargetPosition, pObject->GetNextRotation(), true );
			return;
		}
	}

	const RealVec3_t gravity = pObject->GetEnvironment()->GetIVPGravity();
	const b3Vec3 gravityStep = {
		static_cast<float>( real_t( flTimestep ) * gravity.x ),
		static_cast<float>( real_t( flTimestep ) * gravity.y ),
		static_cast<float>( real_t( flTimestep ) * gravity.z ),
	};

	float flFraction = 1.0f;
	if ( m_flSecondsToArrival > 0.0f )
		flFraction = std::min( flTimestep / m_flSecondsToArrival, 1.0f );

	const float flLastImpulse = b3Length( m_LastImpulse );
	const b3Vec3 limit = m_bUpdated ? m_MaxSpeed : b3Vec3{ flLastImpulse, flLastImpulse, flLastImpulse };
	const float flScale = flFraction / flTimestep;
	if ( b3LengthSquared( speed ) < MIN_SPEED_SQUARED )
		speed = b3Vec3_zero;

	const auto control = [flScale]( float flSpeed, float flDelta, float flLimit ) {
		const float flAcceleration = flSpeed * -1.0f + flScale * flDelta;
		if ( std::fabs( flAcceleration ) >= flLimit )
			return flAcceleration < 0.0f ? -flLimit : flLimit;
		return flAcceleration;
	};
	const b3Vec3 acceleration = { control( speed.x, delta.x, limit.x ), control( speed.y, delta.y, limit.y ), control( speed.z, delta.z, limit.z ) };

	speed = b3Add( b3Add( speed, acceleration ), groundVelocity );
	if ( m_bUpdated )
		m_LastImpulse = acceleration;
	m_bUpdated = false;
	pObject->SetCoreVelocity( speed );

	bool bStanding;
	ClipToContacts( pObject, bStanding );
	if ( bStanding && m_LastImpulse.y >= 0.0f )
	{
		const float flGravityStep = b3Length( gravityStep );
		speed = pObject->GetCoreVelocity();
		speed.y += flGravityStep - m_LastImpulse.y;
		pObject->SetCoreVelocity( speed );
		m_LastImpulse.y = flGravityStep;
	}

	m_flSecondsToArrival -= flTimestep;
	if ( m_flSecondsToArrival < 0.0f )
		m_flSecondsToArrival = 0.0f;
}

void CPhysicsPlayerController::ClipToContacts( CPhysicsObject *pObject, bool &bStanding )
{
	const b3Vec3 speed = pObject->GetCoreVelocity();
	const Vector velocity = ToSource( IVPToSource( speed ) );

	float flPushSpeed = m_flPushSpeedLimit;
	const float flInvMass = pObject->GetInvMass();
	Vector planes[MAX_CLIP_PLANES];
	int nPlanes = 0;
	bStanding = false;
	for ( const CPhysicsEnvironment::ContactNormal_t &contact : pObject->GetEnvironment()->GetContactNormals( pObject ) )
	{
		const Vector &normal = contact.m_Normal;
		if ( normal.z < STANDING_NORMAL_Z )
			bStanding = true;
		if ( !( normal.z > FLOOR_NORMAL_Z ) )
			continue;

		if ( !contact.m_pOther->IsMoveable() || contact.m_pOther->GetMass() > m_flPushMassLimit )
			flPushSpeed = 0.0f;

		if ( !( flPushSpeed < contact.m_flNormalForce * flInvMass + ( ( normal.y * velocity.y + normal.x * velocity.x ) + normal.z * velocity.z ) ) || nPlanes >= MAX_CLIP_PLANES )
			continue;

		if ( std::none_of( planes, planes + nPlanes, [&]( const Vector &plane ) { return DotProduct( normal, plane ) > DUPLICATE_PLANE_DOT; } ) )
			planes[nPlanes++] = normal;
	}

	Vector clipped = velocity;
	if ( nPlanes == 1 )
	{
		const float flDot = DotProduct( planes[0], velocity );
		if ( flPushSpeed < flDot )
			clipped = velocity + planes[0] * ( flPushSpeed - flDot );
	}
	else if ( nPlanes == 2 )
	{
		const Vector crease = CrossProduct( planes[0], planes[1] );
		clipped = crease * DotProduct( crease, velocity );
	}
	else if ( nPlanes >= 3 && std::any_of( planes, planes + nPlanes, [&]( const Vector &plane ) { return DotProduct( plane, velocity ) > 0.0f; } ) )
	{
		clipped.Init();
	}

	if ( nPlanes > 0 )
	{
		const b3Vec3 change = SourceToIVP( clipped - velocity );
		pObject->SetCoreVelocity( b3Add( speed, change ) );
		m_LastImpulse = b3Add( m_LastImpulse, change );
	}
}

void CPhysicsPlayerController::ObjectDestroyed( CPhysicsObject * )
{
	m_pObject = nullptr;
	m_pGround = nullptr;
}

void CPhysicsPlayerController::GroundDestroyed( const CPhysicsObject *pObject )
{
	if ( m_pGround == pObject )
		m_pGround = nullptr;
}
