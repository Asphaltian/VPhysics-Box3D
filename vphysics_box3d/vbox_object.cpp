//=================================================================================================
//
// A physics object
//
//=================================================================================================

#include "cbase.h"

#include "vbox_object.h"
#include "vbox_environment.h"
#include "vbox_collide.h"
#include "vbox_controllers.h"

#include "vphysics/friction.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

namespace
{
	// The game iterates `while ( snapshot->IsValid() )` without null-checking, so we return a valid
	// empty snapshot rather than null.
	class Box3DDummyFrictionSnapshot final : public IPhysicsFrictionSnapshot
	{
	public:
		bool IsValid() override { return false; }
		IPhysicsObject *GetObject( int ) override { return nullptr; }
		int GetMaterial( int ) override { return 0; }
		void GetContactPoint( Vector &out ) override { out = vec3_origin; }
		void GetSurfaceNormal( Vector &out ) override { out = vec3_origin; }
		float GetNormalForce() override { return 0.0f; }
		float GetEnergyAbsorbed() override { return 0.0f; }
		void RecomputeFriction() override {}
		void ClearFrictionForce() override {}
		void MarkContactForDelete() override {}
		void DeleteAllMarkedContacts( bool ) override {}
		void NextFrictionData() override {}
		float GetFrictionCoefficient() override { return 0.0f; }
	};
}

Box3DPhysicsObject::Box3DPhysicsObject( b3BodyId bodyId, Box3DPhysicsEnvironment *pEnvironment, bool bStatic, int nMaterialIndex, const CPhysCollide *pCollide, const objectparams_t *pParams )
	: m_bStatic( bStatic )
	, m_materialIndex( nMaterialIndex )
	, m_pCollide( pCollide )
	, m_BodyId( bodyId )
	, m_pEnvironment( pEnvironment )
{
	m_WorldId = b3Body_GetWorld( bodyId );
	b3Body_SetUserData( bodyId, this );

	if ( pParams )
	{
		m_pGameData = pParams->pGameData;
		if ( pParams->pName )
			m_pName = pParams->pName;

		m_flLinearDamping = pParams->damping;
		m_flAngularDamping = pParams->rotdamping;
		if ( !bStatic )
		{
			b3Body_SetLinearDamping( bodyId, pParams->damping );
			b3Body_SetAngularDamping( bodyId, pParams->rotdamping );
		}

		if ( pParams->mass > 0.0f )
			SetMass( pParams->mass );
	}

	if ( m_flCachedMass <= 0.0f )
	{
		m_flCachedMass = bStatic ? 0.0f : b3Body_GetMass( bodyId );
		m_flCachedInvMass = bStatic ? 0.0f : b3Body_GetInverseMass( bodyId );
	}
}

Box3DPhysicsObject::~Box3DPhysicsObject()
{
	// The environment owns the Box3D body lifetime (b3DestroyBody in DestroyObject).
}

//-------------------------------------------------------------------------------------------------

bool Box3DPhysicsObject::IsStatic() const						{ return m_bStatic; }
bool Box3DPhysicsObject::IsAsleep() const						{ return m_bStatic || !b3Body_IsAwake( m_BodyId ); }
bool Box3DPhysicsObject::IsTrigger() const						{ return false; }
bool Box3DPhysicsObject::IsFluid() const						{ return false; }
bool Box3DPhysicsObject::IsHinged() const						{ return false; }
bool Box3DPhysicsObject::IsCollisionEnabled() const				{ return m_bCollisionEnabled; }
bool Box3DPhysicsObject::IsGravityEnabled() const				{ return !m_bStatic && m_bGravityEnabled; }
bool Box3DPhysicsObject::IsDragEnabled() const					{ return m_bDragEnabled; }
bool Box3DPhysicsObject::IsMotionEnabled() const				{ return !m_bStatic && m_bMotionEnabled; }
bool Box3DPhysicsObject::IsMoveable() const						{ return IsMotionEnabled(); }
bool Box3DPhysicsObject::IsAttachedToConstraint( bool ) const	{ return false; }

void Box3DPhysicsObject::EnableCollisions( bool enable )
{
	m_bCollisionEnabled = enable;
}

void Box3DPhysicsObject::EnableGravity( bool enable )
{
	m_bGravityEnabled = enable;
	if ( !m_bStatic )
		b3Body_SetGravityScale( m_BodyId, enable ? 1.0f : 0.0f );
}

void Box3DPhysicsObject::EnableDrag( bool enable )
{
	m_bDragEnabled = enable;
}

void Box3DPhysicsObject::EnableMotion( bool enable )
{
	if ( m_bStatic || m_bMotionEnabled == enable )
		return;

	m_bMotionEnabled = enable;
	b3Body_SetType( m_BodyId, enable ? b3_dynamicBody : b3_staticBody );
	if ( enable )
	{
		b3Body_ApplyMassFromShapes( m_BodyId );
		b3Body_SetAwake( m_BodyId, true );
	}
}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsObject::SetGameData( void *pGameData )				{ m_pGameData = pGameData; }
void *Box3DPhysicsObject::GetGameData() const						{ return m_pGameData; }
void Box3DPhysicsObject::SetGameFlags( unsigned short userFlags )	{ m_gameFlags = userFlags; }
unsigned short Box3DPhysicsObject::GetGameFlags() const				{ return m_gameFlags; }
void Box3DPhysicsObject::SetGameIndex( unsigned short gameIndex )	{ m_gameIndex = gameIndex; }
unsigned short Box3DPhysicsObject::GetGameIndex() const				{ return m_gameIndex; }
void Box3DPhysicsObject::SetCallbackFlags( unsigned short flags )	{ m_callbackFlags = flags; }
unsigned short Box3DPhysicsObject::GetCallbackFlags() const			{ return m_callbackFlags; }

void Box3DPhysicsObject::Wake()										{ if ( !m_bStatic ) b3Body_SetAwake( m_BodyId, true ); }
void Box3DPhysicsObject::Sleep()									{ if ( !m_bStatic ) b3Body_SetAwake( m_BodyId, false ); }
void Box3DPhysicsObject::RecheckCollisionFilter()					{}
void Box3DPhysicsObject::RecheckContactPoints( bool )				{}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsObject::SetMass( float mass )
{
	m_flCachedMass = mass;
	m_flCachedInvMass = mass > 0.0f ? 1.0f / mass : 0.0f;

	if ( m_bStatic )
		return;

	b3MassData massData = b3Body_GetMassData( m_BodyId );
	const float scale = massData.mass > 0.0f ? mass / massData.mass : 1.0f;
	massData.mass = mass;
	massData.inertia.cx.x *= scale; massData.inertia.cx.y *= scale; massData.inertia.cx.z *= scale;
	massData.inertia.cy.x *= scale; massData.inertia.cy.y *= scale; massData.inertia.cy.z *= scale;
	massData.inertia.cz.x *= scale; massData.inertia.cz.y *= scale; massData.inertia.cz.z *= scale;
	b3Body_SetMassData( m_BodyId, massData );
}

float Box3DPhysicsObject::GetMass() const		{ return m_flCachedMass; }
float Box3DPhysicsObject::GetInvMass() const	{ return m_flCachedInvMass; }

Vector Box3DPhysicsObject::GetInertia() const
{
	const b3Matrix3 inertia = b3Body_GetLocalRotationalInertia( m_BodyId );
	// kg m^2 -> kg in^2
	const float k = MetresToInches * MetresToInches;
	return Vector( fabsf( inertia.cx.x ) * k, fabsf( inertia.cy.y ) * k, fabsf( inertia.cz.z ) * k );
}

Vector Box3DPhysicsObject::GetInvInertia() const
{
	const Vector inertia = GetInertia();
	return Vector(
		inertia.x > 0.0f ? 1.0f / inertia.x : 0.0f,
		inertia.y > 0.0f ? 1.0f / inertia.y : 0.0f,
		inertia.z > 0.0f ? 1.0f / inertia.z : 0.0f );
}

void Box3DPhysicsObject::SetInertia( const Vector & )	{ Log_Stub( LOG_VBox3D ); }

void Box3DPhysicsObject::SetDamping( const float *speed, const float *rot )
{
	if ( speed ) { m_flLinearDamping = *speed; if ( !m_bStatic ) b3Body_SetLinearDamping( m_BodyId, *speed ); }
	if ( rot )   { m_flAngularDamping = *rot;  if ( !m_bStatic ) b3Body_SetAngularDamping( m_BodyId, *rot ); }
}

void Box3DPhysicsObject::GetDamping( float *speed, float *rot ) const
{
	if ( speed ) *speed = m_flLinearDamping;
	if ( rot )   *rot = m_flAngularDamping;
}

void Box3DPhysicsObject::SetDragCoefficient( float *, float * )	{}
void Box3DPhysicsObject::SetBuoyancyRatio( float )				{}

int Box3DPhysicsObject::GetMaterialIndex() const				{ return m_materialIndex; }
void Box3DPhysicsObject::SetMaterialIndex( int materialIndex )	{ m_materialIndex = materialIndex; }
unsigned int Box3DPhysicsObject::GetContents() const			{ return m_contents; }
void Box3DPhysicsObject::SetContents( unsigned int contents )	{ m_contents = contents; }

float Box3DPhysicsObject::GetSphereRadius() const				{ return 0.0f; }
void Box3DPhysicsObject::SetSphereRadius( float )				{}
float Box3DPhysicsObject::GetEnergy() const						{ return 0.0f; }

Vector Box3DPhysicsObject::GetMassCenterLocalSpace() const
{
	return BoxToSource::Distance( b3Body_GetLocalCenterOfMass( m_BodyId ) );
}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsObject::SetPosition( const Vector &worldPosition, const QAngle &angles, bool )
{
	b3Body_SetTransform( m_BodyId, SourceToBox::Distance( worldPosition ), SourceToBox::Angle( angles ) );
}

void Box3DPhysicsObject::SetPositionMatrix( const matrix3x4_t &matrix, bool )
{
	const b3Transform xf = SourceToBox::Transform( matrix );
	b3Body_SetTransform( m_BodyId, xf.p, xf.q );
}

void Box3DPhysicsObject::GetPosition( Vector *worldPosition, QAngle *angles ) const
{
	const b3WorldTransform xf = b3Body_GetTransform( m_BodyId );
	if ( worldPosition ) *worldPosition = BoxToSource::Distance( xf.p );
	if ( angles )        *angles = BoxToSource::Angle( xf.q );
}

void Box3DPhysicsObject::GetPositionMatrix( matrix3x4_t *positionMatrix ) const
{
	if ( positionMatrix )
		*positionMatrix = BoxToSource::Matrix( b3Body_GetTransform( m_BodyId ) );
}

void Box3DPhysicsObject::SetVelocity( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( m_bStatic )
		return;
	// Never feed NaN/Inf into the solver — a bad velocity propagates to a bad position and the engine
	// deletes the entity.
	const bool bVel = velocity && velocity->IsValid();
	const bool bAng = angularVelocity && angularVelocity->IsValid();
	if ( bVel ) b3Body_SetLinearVelocity( m_BodyId, SourceToBox::Distance( *velocity ) );
	if ( bAng ) b3Body_SetAngularVelocity( m_BodyId, SourceToBox::AngularImpulse( *angularVelocity ) );
	if ( bVel || bAng ) b3Body_SetAwake( m_BodyId, true );
}

void Box3DPhysicsObject::SetVelocityInstantaneous( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	SetVelocity( velocity, angularVelocity );
}

void Box3DPhysicsObject::GetVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const
{
	if ( velocity )        *velocity = BoxToSource::Distance( b3Body_GetLinearVelocity( m_BodyId ) );
	if ( angularVelocity ) *angularVelocity = BoxToSource::AngularImpulse( b3Body_GetAngularVelocity( m_BodyId ) );
}

void Box3DPhysicsObject::AddVelocity( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( m_bStatic )
		return;
	if ( velocity )        b3Body_SetLinearVelocity( m_BodyId, b3Add( b3Body_GetLinearVelocity( m_BodyId ), SourceToBox::Distance( *velocity ) ) );
	if ( angularVelocity ) b3Body_SetAngularVelocity( m_BodyId, b3Add( b3Body_GetAngularVelocity( m_BodyId ), SourceToBox::AngularImpulse( *angularVelocity ) ) );
	if ( velocity || angularVelocity ) b3Body_SetAwake( m_BodyId, true );
}

void Box3DPhysicsObject::GetVelocityAtPoint( const Vector &worldPosition, Vector *pVelocity ) const
{
	if ( pVelocity )
		*pVelocity = BoxToSource::Distance( b3Body_GetWorldPointVelocity( m_BodyId, SourceToBox::Distance( worldPosition ) ) );
}

void Box3DPhysicsObject::GetImplicitVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const
{
	GetVelocity( velocity, angularVelocity );
}

void Box3DPhysicsObject::LocalToWorld( Vector *worldPosition, const Vector &localPosition ) const
{
	if ( worldPosition )
		*worldPosition = BoxToSource::Distance( b3Body_GetWorldPoint( m_BodyId, SourceToBox::Distance( localPosition ) ) );
}

void Box3DPhysicsObject::WorldToLocal( Vector *localPosition, const Vector &worldPosition ) const
{
	if ( localPosition )
		*localPosition = BoxToSource::Distance( b3Body_GetLocalPoint( m_BodyId, SourceToBox::Distance( worldPosition ) ) );
}

void Box3DPhysicsObject::LocalToWorldVector( Vector *worldVector, const Vector &localVector ) const
{
	if ( worldVector )
		*worldVector = BoxToSource::Unitless( b3Body_GetWorldVector( m_BodyId, SourceToBox::Unitless( localVector ) ) );
}

void Box3DPhysicsObject::WorldToLocalVector( Vector *localVector, const Vector &worldVector ) const
{
	if ( localVector )
		*localVector = BoxToSource::Unitless( b3Body_GetLocalVector( m_BodyId, SourceToBox::Unitless( worldVector ) ) );
}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsObject::ApplyForceCenter( const Vector &forceVector )
{
	if ( !m_bStatic )
		b3Body_ApplyLinearImpulseToCenter( m_BodyId, SourceToBox::Distance( forceVector ), true );
}

void Box3DPhysicsObject::ApplyForceOffset( const Vector &forceVector, const Vector &worldPosition )
{
	if ( !m_bStatic )
		b3Body_ApplyLinearImpulse( m_BodyId, SourceToBox::Distance( forceVector ), SourceToBox::Distance( worldPosition ), true );
}

void Box3DPhysicsObject::ApplyTorqueCenter( const AngularImpulse &torque )
{
	if ( !m_bStatic )
		b3Body_ApplyAngularImpulse( m_BodyId, SourceToBox::AngularImpulse( torque ), true );
}

void Box3DPhysicsObject::CalculateForceOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerForce, AngularImpulse *centerTorque ) const
{
	if ( centerForce ) *centerForce = forceVector;
	if ( centerTorque )
	{
		Vector com;
		com = BoxToSource::Distance( b3Body_GetWorldCenterOfMass( m_BodyId ) );
		*centerTorque = CrossProduct( worldPosition - com, forceVector );
	}
}

void Box3DPhysicsObject::CalculateVelocityOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerVelocity, AngularImpulse *centerAngularVelocity ) const
{
	if ( centerVelocity )        *centerVelocity = forceVector * GetInvMass();
	if ( centerAngularVelocity )
	{
		Vector centerTorque;
		CalculateForceOffset( forceVector, worldPosition, nullptr, &centerTorque );
		const Vector invInertia = GetInvInertia();
		*centerAngularVelocity = Vector( centerTorque.x * invInertia.x, centerTorque.y * invInertia.y, centerTorque.z * invInertia.z );
	}
}

float Box3DPhysicsObject::CalculateLinearDrag( const Vector & ) const	{ return 0.0f; }
float Box3DPhysicsObject::CalculateAngularDrag( const Vector & ) const	{ return 0.0f; }

bool Box3DPhysicsObject::GetContactPoint( Vector *, IPhysicsObject ** ) const { return false; }

//-------------------------------------------------------------------------------------------------
// Shadow controllers come in M3.

void Box3DPhysicsObject::SetShadow( float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation )
{
	if ( !m_pShadowController )
		m_pShadowController = static_cast< Box3DPhysicsShadowController * >(
			m_pEnvironment->CreateShadowController( this, allowPhysicsMovement, allowPhysicsRotation ) );
	m_pShadowController->MaxSpeed( maxSpeed, maxAngularSpeed );
}

void Box3DPhysicsObject::UpdateShadow( const Vector &targetPosition, const QAngle &targetAngles, bool tempDisableGravity, float timeOffset )
{
	if ( m_pShadowController )
		m_pShadowController->Update( targetPosition, targetAngles, timeOffset );
}

int Box3DPhysicsObject::GetShadowPosition( Vector *position, QAngle *angles ) const	{ GetPosition( position, angles ); return 1; }
IPhysicsShadowController *Box3DPhysicsObject::GetShadowController() const		{ return m_pShadowController; }

void Box3DPhysicsObject::RemoveShadowController()
{
	if ( m_pShadowController )
	{
		m_pEnvironment->DestroyShadowController( m_pShadowController );
		m_pShadowController = nullptr;
	}
}
// Damp the current velocity, then add an acceleration proportional to the remaining delta, each clamped.
static void ComputeController( Vector &vecCurrentVelocity, const Vector &vecDelta, float flMaxSpeed, float flMaxDampSpeed, float flScaleDelta, float flDamping )
{
	if ( vecCurrentVelocity.LengthSqr() < 1e-6f )
	{
		vecCurrentVelocity = vec3_origin;
	}
	else if ( flMaxDampSpeed > 0.0f )
	{
		Vector vecDampen = vecCurrentVelocity * -flDamping;
		const float flSpeed = vecCurrentVelocity.Length() * fabsf( flDamping );
		if ( flSpeed > flMaxDampSpeed )
			vecDampen *= flMaxDampSpeed / flSpeed;
		vecCurrentVelocity += vecDampen;
	}

	if ( flMaxSpeed > 0.0f )
	{
		Vector vecAccel = vecDelta * flScaleDelta;
		const float flSpeed = vecDelta.Length() * flScaleDelta;
		if ( flSpeed > flMaxSpeed )
			vecAccel *= flMaxSpeed / flSpeed;
		vecCurrentVelocity += vecAccel;
	}
}

// Shortest-arc rotation from cur to target as axis * angle, in degrees.
static Vector ComputeRotationDeltaDegrees( const QAngle &curAngles, const QAngle &targetAngles )
{
	Quaternion qCur, qTarget;
	AngleQuaternion( curAngles, qCur );
	AngleQuaternion( targetAngles, qTarget );

	Quaternion qInv, qDelta;
	QuaternionInvert( qCur, qInv );
	QuaternionMult( qTarget, qInv, qDelta );

	// QuaternionAxisAngle does acos( w ) with no clamp: once the object is aligned with the target, w
	// drifts a hair past 1.0 from float error and acos() returns NaN. That NaN becomes a NaN velocity and
	// the engine deletes the entity. Clamp to keep it finite.
	qDelta.w = clamp( qDelta.w, -1.0f, 1.0f );

	Vector axis;
	float angleDeg;
	QuaternionAxisAngle( qDelta, axis, angleDeg );

	return axis * angleDeg;
}

// The engine's grab/shadow (physgun, +use pickup, doors) drives a held object toward a target through
// this every step: compute a velocity toward the target position/rotation and set it on the body.
float Box3DPhysicsObject::ComputeShadowControl( const hlshadowcontrol_params_t &params, float flSecondsToArrival, float flDeltaTime )
{
	Vector position;
	QAngle angles;
	GetPosition( &position, &angles );

	Vector linearVelocity;
	AngularImpulse angularVelocity;
	GetVelocity( &linearVelocity, &angularVelocity );

	const float flFraction = flSecondsToArrival > 0.0f ? Min( flDeltaTime / flSecondsToArrival, 1.0f ) : 1.0f;
	flSecondsToArrival = Max( flSecondsToArrival - flDeltaTime, 0.0f );

	if ( flFraction <= 0.0f )
		return flSecondsToArrival;

	Vector deltaPosition = params.targetPosition - position;

	bool bTeleport = false;
	if ( params.teleportDistance > 0.0f && deltaPosition.LengthSqr() > Square( params.teleportDistance ) )
	{
		position = params.targetPosition;
		angles = params.targetRotation;
		deltaPosition = vec3_origin;
		bTeleport = true;
	}

	const float flFractionTime = flFraction / flDeltaTime;

	ComputeController( linearVelocity, deltaPosition, params.maxSpeed, params.maxDampSpeed, flFractionTime, params.dampFactor );

	const Vector deltaAngles = ComputeRotationDeltaDegrees( angles, params.targetRotation );
	ComputeController( angularVelocity, deltaAngles, params.maxAngular, params.maxDampAngular, flFractionTime, params.dampFactor );

	if ( bTeleport )
		SetPosition( position, angles, true );

	SetVelocity( &linearVelocity, &angularVelocity );

	return flSecondsToArrival;
}

const CPhysCollide *Box3DPhysicsObject::GetCollide() const	{ return m_pCollide; }
const char *Box3DPhysicsObject::GetName() const				{ return m_pName; }

void Box3DPhysicsObject::BecomeTrigger()					{ Log_Stub( LOG_VBox3D ); }
void Box3DPhysicsObject::RemoveTrigger()					{ Log_Stub( LOG_VBox3D ); }
void Box3DPhysicsObject::BecomeHinged( int )				{ Log_Stub( LOG_VBox3D ); }
void Box3DPhysicsObject::RemoveHinged()						{ Log_Stub( LOG_VBox3D ); }

IPhysicsFrictionSnapshot *Box3DPhysicsObject::CreateFrictionSnapshot()		{ return new Box3DDummyFrictionSnapshot; }
void Box3DPhysicsObject::DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot ) { delete static_cast< Box3DDummyFrictionSnapshot * >( pSnapshot ); }

void Box3DPhysicsObject::OutputDebugInfo() const			{ Log_Stub( LOG_VBox3D ); }

void Box3DPhysicsObject::SetUseAlternateGravity( bool )		{}
void Box3DPhysicsObject::SetCollisionHints( uint32 collisionHints ) { m_collisionHints = collisionHints; }
uint32 Box3DPhysicsObject::GetCollisionHints() const		{ return m_collisionHints; }

IPredictedPhysicsObject *Box3DPhysicsObject::GetPredictedInterface() const	{ return nullptr; }
void Box3DPhysicsObject::SyncWith( IPhysicsObject * )						{}
