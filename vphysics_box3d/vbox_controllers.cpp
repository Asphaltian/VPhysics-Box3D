//=================================================================================================
//
// Shadow + motion controllers
//
//=================================================================================================

#include "cbase.h"

#include "vbox_controllers.h"
#include "vbox_object.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------
// Shadow controller: makes the object kinematic and drives it toward a target each step. This is
// how the game holds/moves objects it wants under its control (pickup, physgun, doors).
//-------------------------------------------------------------------------------------------------

Box3DPhysicsShadowController::Box3DPhysicsShadowController( Box3DPhysicsObject *pObject, bool allowTranslation, bool allowRotation )
	: m_pObject( pObject )
	, m_allowTranslation( allowTranslation )
	, m_allowRotation( allowRotation )
{
	m_bWasStatic = m_pObject->IsStatic();
	if ( !m_bWasStatic )
		b3Body_SetType( m_pObject->GetBodyID(), b3_kinematicBody );

	m_savedCallbackFlags = m_pObject->GetCallbackFlags();
	m_pObject->SetCallbackFlags( m_savedCallbackFlags | CALLBACK_SHADOW_COLLISION );
}

Box3DPhysicsShadowController::~Box3DPhysicsShadowController()
{
	const b3BodyId bodyId = m_pObject->GetBodyID();
	if ( !m_bWasStatic && b3Body_IsValid( bodyId ) )
	{
		b3Body_SetType( bodyId, b3_dynamicBody );
		b3Body_SetAwake( bodyId, true );
	}

	if ( !( m_pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
		m_pObject->SetCallbackFlags( m_savedCallbackFlags );
}

void Box3DPhysicsShadowController::Update( const Vector &position, const QAngle &angles, float timeOffset )
{
	m_targetPosition = position;
	m_targetAngles = angles;
	m_secondsToArrival = Max( timeOffset, 0.0f );
	m_enabled = true;
}

void Box3DPhysicsShadowController::MaxSpeed( float maxSpeed, float maxAngularSpeed )
{
	m_maxSpeed = maxSpeed;
	m_maxAngular = maxAngularSpeed;
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
		*pOut = vec3_origin;
}

void Box3DPhysicsShadowController::UseShadowMaterial( bool )
{
}

void Box3DPhysicsShadowController::ObjectMaterialChanged( int )
{
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
	if ( !m_enabled || m_bWasStatic )
		return;

	const b3BodyId bodyId = m_pObject->GetBodyID();

	Vector vecCurPos;
	QAngle angCur;
	m_pObject->GetPosition( &vecCurPos, &angCur );

	// Teleport the last little bit: snap to target and stop driving.
	if ( m_secondsToArrival <= flDeltaTime )
	{
		b3Body_SetTransform( bodyId, SourceToBox::Distance( m_targetPosition ), SourceToBox::Angle( m_targetAngles ) );
		b3Body_SetLinearVelocity( bodyId, b3Vec3{ 0.0f, 0.0f, 0.0f } );
		b3Body_SetAngularVelocity( bodyId, b3Vec3{ 0.0f, 0.0f, 0.0f } );
		m_secondsToArrival = 0.0f;
		m_enabled = false;
		return;
	}

	const float flInvTime = 1.0f / m_secondsToArrival;

	// Linear velocity to reach the target position, clamped to the max speed.
	Vector vecLinear = m_allowTranslation ? ( m_targetPosition - vecCurPos ) * flInvTime : vec3_origin;
	if ( m_maxSpeed > 0.0f )
	{
		const float flSpeed = vecLinear.Length();
		if ( flSpeed > m_maxSpeed )
			vecLinear *= m_maxSpeed / flSpeed;
	}

	// Angular velocity to reach the target orientation via the shortest arc.
	Vector vecAngular = vec3_origin;
	if ( m_allowRotation )
	{
		Quaternion qCur, qTarget;
		AngleQuaternion( angCur, qCur );
		AngleQuaternion( m_targetAngles, qTarget );

		// Keep both quaternions in the same hemisphere so we take the short way round.
		const float flDot = qCur.x * qTarget.x + qCur.y * qTarget.y + qCur.z * qTarget.z + qCur.w * qTarget.w;
		if ( flDot < 0.0f )
		{
			qTarget.x = -qTarget.x; qTarget.y = -qTarget.y; qTarget.z = -qTarget.z; qTarget.w = -qTarget.w;
		}

		Quaternion qInv, qDelta;
		QuaternionInvert( qCur, qInv );
		QuaternionMult( qTarget, qInv, qDelta );

		// Clamp before QuaternionAxisAngle: its acos( w ) returns NaN when w drifts past 1.0 (object
		// aligned with target), which would set a NaN velocity and get the entity deleted.
		qDelta.w = clamp( qDelta.w, -1.0f, 1.0f );

		Vector axis;
		float angleDeg;
		QuaternionAxisAngle( qDelta, axis, angleDeg );

		vecAngular = axis * ( angleDeg * flInvTime );	// degrees / second
		if ( m_maxAngular > 0.0f )
		{
			const float flAng = vecAngular.Length();
			if ( flAng > m_maxAngular )
				vecAngular *= m_maxAngular / flAng;
		}
	}

	b3Body_SetLinearVelocity( bodyId, SourceToBox::Distance( vecLinear ) );
	b3Body_SetAngularVelocity( bodyId, b3Vec3{ DEG2RAD( vecAngular.x ), DEG2RAD( vecAngular.y ), DEG2RAD( vecAngular.z ) } );
	b3Body_SetAwake( bodyId, true );

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

		// The event's angular value is always in the object's local space.
		Vector vecWorldAngular;
		pObject->LocalToWorldVector( &vecWorldAngular, angLocalAngular );

		// The linear value is local or global depending on the result type.
		Vector vecWorldLinear = vecLinear;
		if ( result == IMotionEvent::SIM_LOCAL_ACCELERATION || result == IMotionEvent::SIM_LOCAL_FORCE )
			pObject->LocalToWorldVector( &vecWorldLinear, vecLinear );

		switch ( result )
		{
			case IMotionEvent::SIM_GLOBAL_ACCELERATION:
			case IMotionEvent::SIM_LOCAL_ACCELERATION:
				pObject->AddVelocity( &vecWorldLinear, &vecWorldAngular );
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
