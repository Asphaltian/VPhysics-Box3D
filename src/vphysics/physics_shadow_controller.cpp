#include "physics_shadow_controller.h"

#include "conversion.h"
#include "physics_surface_props.h"

#include <cmath>

static constexpr float SHADOW_ROT_DAMPING = 100.0f;
static constexpr float LOCKED_INERTIA = 1e15f;
static constexpr float LOCKED_MASS = 50000.0f;
static constexpr real_t MIN_POSITION_CHANGE = 1e-8;
static constexpr float MIN_ROTATION_CHANGE = 1e-8f;

CPhysicsShadowController::CPhysicsShadowController( CPhysicsObject *pObject, bool allowTranslation, bool allowRotation )
: m_pObject( pObject )
, m_bAllowTranslation( allowTranslation )
, m_bAllowRotation( allowRotation )
{
	MaxSpeed( 0.0f, 0.0f );
	m_Control.m_flDampFactor = 1.0f;
	m_Control.m_flTeleportDistance = 0.0f;
	m_Control.m_LastImpulse = b3Vec3_zero;

	m_flSavedRotDamping = pObject->GetRotDamping();
	m_SavedInertia = pObject->GetInertia();
	m_flSavedMass = pObject->GetMass();
	m_nSavedMaterialIndex = static_cast<unsigned short>( pObject->GetMaterialIndex() );

	Vector position;
	QAngle angles;
	pObject->GetPosition( &position, &angles );
	m_Control.m_TargetPosition = ToReal( SourceToIVP( position ) );
	m_Control.m_TargetRotation = AnglesToIVP( angles );

	UseShadowMaterial( true );
	pObject->SetRotDamping( SHADOW_ROT_DAMPING );
	if ( !allowRotation )
		pObject->SetInertia( Vector( LOCKED_INERTIA, LOCKED_INERTIA, LOCKED_INERTIA ) );
	if ( !allowTranslation )
	{
		pObject->SetMass( LOCKED_MASS );
		pObject->EnableGravity( false );
	}

	m_nSavedCallbackFlags = pObject->GetCallbackFlags();
	pObject->SetCallbackFlags( ( m_nSavedCallbackFlags & ~( CALLBACK_GLOBAL_FRICTION | CALLBACK_GLOBAL_COLLIDE_STATIC ) ) | CALLBACK_SHADOW_COLLISION );
	pObject->EnableDrag( false );

	pObject->AttachController( this );
	m_Control.m_LastPosition = {};
}

CPhysicsShadowController::~CPhysicsShadowController()
{
	if ( !m_pObject )
		return;

	if ( !( m_pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
	{
		m_pObject->SetRotDamping( m_flSavedRotDamping );
		m_pObject->ChangeMass( m_flSavedMass );
		m_pObject->SetCallbackFlags( m_nSavedCallbackFlags );
		m_pObject->EnableDrag( true );
		m_pObject->EnableGravity( true );
		UseShadowMaterial( false );
		m_pObject->SetInertia( m_SavedInertia );
	}

	CPhysicsObject *pObject = m_pObject;
	m_pObject = nullptr;
	pObject->DetachController( this );
}

void CPhysicsShadowController::Update( const Vector &position, const QAngle &angles, float timeOffset )
{
	const RealVec3_t targetPosition = ToReal( SourceToIVP( position ) );
	const RealQuat_t targetRotation = AnglesToIVP( angles );
	const RealVec3_t positionChange = m_Control.m_TargetPosition - targetPosition;
	const RealQuat_t &q0 = m_Control.m_TargetRotation;
	const float flRotationChange = static_cast<float>( std::fabs( q0.x - targetRotation.x ) + std::fabs( q0.y - targetRotation.y ) + std::fabs( q0.z - targetRotation.z ) + std::fabs( q0.w - targetRotation.w ) );
	const bool bChanged = !( LengthSquared( positionChange ) < MIN_POSITION_CHANGE && flRotationChange < MIN_ROTATION_CHANGE );

	m_Control.m_TargetPosition = targetPosition;
	m_Control.m_TargetRotation = targetRotation;
	m_flSecondsToArrival = timeOffset <= 0.0f ? 0.0f : timeOffset;
	m_bHasTarget = true;

	if ( bChanged )
		m_pObject->Wake();
}

void CPhysicsShadowController::MaxSpeed( float maxSpeed, float maxAngularSpeed )
{
	m_Control.m_flMaxSpeed = maxSpeed;
	m_Control.m_flMaxDampSpeed = maxSpeed;
	m_Control.m_flMaxAngular = maxAngularSpeed;
	m_Control.m_flMaxDampAngular = maxAngularSpeed;
}

void CPhysicsShadowController::StepUp( float height )
{
	m_pObject->StepUp( height );
}

void CPhysicsShadowController::SetTeleportDistance( float teleportDistance )
{
	m_Control.m_flTeleportDistance = teleportDistance * SOURCE_TO_IVP;
}

void CPhysicsShadowController::GetLastImpulse( Vector *pOut )
{
	*pOut = ToSource( IVPToSource( m_Control.m_LastImpulse ) );
}

void CPhysicsShadowController::UseShadowMaterial( bool bUseShadowMaterial )
{
	if ( !m_pObject )
		return;

	const int nMaterialIndex = bUseShadowMaterial ? CPhysicsSurfaceProps::MATERIAL_INDEX_SHADOW : m_nSavedMaterialIndex;
	if ( nMaterialIndex != m_pObject->GetMaterialIndex() )
		m_pObject->SetMaterialIndex( nMaterialIndex );
}

void CPhysicsShadowController::ObjectMaterialChanged( int materialIndex )
{
	if ( m_pObject )
		m_nSavedMaterialIndex = static_cast<unsigned short>( materialIndex );
}

float CPhysicsShadowController::GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut )
{
	if ( pPositionOut )
		*pPositionOut = IVPToSourcePosition( m_Control.m_TargetPosition );
	if ( pAnglesOut )
		*pAnglesOut = IVPToAngles( QuatToMatrix( m_Control.m_TargetRotation ) );
	return m_flSecondsToArrival;
}

float CPhysicsShadowController::GetTeleportDistance()
{
	return m_Control.m_flTeleportDistance * IVP_TO_SOURCE;
}

void CPhysicsShadowController::GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut )
{
	if ( pMaxSpeedOut )
		*pMaxSpeedOut = m_Control.m_flMaxSpeed;
	if ( pMaxAngularSpeedOut )
		*pMaxAngularSpeedOut = m_Control.m_flMaxAngular;
}

void CPhysicsShadowController::Simulate( CPhysicsObject *pObject, float flTimestep )
{
	if ( !m_bHasTarget )
	{
		m_Control.m_LastPosition = {};
		return;
	}

	pObject->ApplyShadowControl( m_Control, m_flSecondsToArrival, flTimestep );

	if ( m_bAllowTranslation )
	{
		const float flGravity = static_cast<float>( real_t( flTimestep ) * pObject->GetEnvironment()->GetIVPGravity().y );
		if ( flGravity < m_Control.m_LastImpulse.y )
		{
			for ( const CPhysicsEnvironment::ContactNormal_t &contact : pObject->GetEnvironment()->GetContactNormals( pObject ) )
			{
				if ( !( contact.m_Normal.z < STANDING_NORMAL_Z ) )
					continue;

				const float flExcess = flGravity - m_Control.m_LastImpulse.y;
				b3Vec3 velocity = pObject->GetCoreVelocity();
				velocity.y += flExcess;
				pObject->SetCoreVelocity( velocity );
				m_Control.m_LastImpulse.y += flExcess;
				break;
			}
		}
	}

	m_flSecondsToArrival -= flTimestep;
	if ( m_flSecondsToArrival < 0.0f )
		m_flSecondsToArrival = 0.0f;
}
