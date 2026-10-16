#include "physics_motion_controller.h"

#include "conversion.h"
#include "physics_object.h"
#include "vphysics_saverestore.h"

#include "tier0/dbg.h"

#include <algorithm>

static constexpr int LOW_PRIORITY_VALUE = 400;
static constexpr int MEDIUM_PRIORITY_VALUE = 500;
static constexpr int HIGH_PRIORITY_VALUE = 1601;

CPhysicsMotionController::CPhysicsMotionController( IMotionEvent *pHandler )
: m_pHandler( pHandler )
, m_nPriority( MEDIUM_PRIORITY_VALUE )
{
}

CPhysicsMotionController::~CPhysicsMotionController()
{
	for ( CPhysicsObject *pObject : m_Objects )
		pObject->DetachController( this );
}

void CPhysicsMotionController::AttachObject( IPhysicsObject *pObject, bool checkIfAlreadyAttached )
{
	if ( !pObject || pObject->IsStatic() )
		return;

	CPhysicsObject *pPhysicsObject = static_cast<CPhysicsObject *>( pObject );
	if ( checkIfAlreadyAttached && std::find( m_Objects.begin(), m_Objects.end(), pPhysicsObject ) != m_Objects.end() )
	{
		DevMsg( 1, "Attached core twice!!!\n" );
		return;
	}

	m_Objects.push_back( pPhysicsObject );
	pPhysicsObject->AttachController( this );
}

void CPhysicsMotionController::DetachObject( IPhysicsObject *pObject )
{
	auto it = std::find( m_Objects.begin(), m_Objects.end(), pObject );
	if ( it == m_Objects.end() )
		return;

	m_Objects.erase( it );
	static_cast<CPhysicsObject *>( pObject )->DetachController( this );
}

void CPhysicsMotionController::GetObjects( IPhysicsObject **pObjectList )
{
	std::copy( m_Objects.begin(), m_Objects.end(), pObjectList );
}

void CPhysicsMotionController::ClearObjects()
{
	while ( !m_Objects.empty() )
		DetachObject( m_Objects.back() );
}

void CPhysicsMotionController::WakeObjects()
{
	for ( CPhysicsObject *pObject : m_Objects )
		pObject->Wake();
}

void CPhysicsMotionController::SetPriority( priority_t priority )
{
	switch ( priority )
	{
	case LOW_PRIORITY:
		m_nPriority = LOW_PRIORITY_VALUE;
		break;
	case HIGH_PRIORITY:
		m_nPriority = HIGH_PRIORITY_VALUE;
		break;
	default:
		m_nPriority = MEDIUM_PRIORITY_VALUE;
		break;
	}
}

void CPhysicsMotionController::Simulate( CPhysicsObject *pObject, float flTimestep )
{
	if ( !m_pHandler || !pObject->IsMoveable() )
		return;

	Vector linear( 0.0f, 0.0f, 0.0f );
	AngularImpulse angular( 0.0f, 0.0f, 0.0f );
	const IMotionEvent::simresult_e result = m_pHandler->Simulate( this, pObject, flTimestep, linear, angular );

	const b3Vec3 l = SourceToIVP( linear );
	const b3Vec3 a = SourceToIVPAngular( angular );
	const auto applyTorque = [&]() {
		const b3Vec3 inverseInertia = pObject->GetCoreInverseInertia();
		const b3Vec3 change = {
			static_cast<float>( ( real_t( inverseInertia.x ) * real_t( a.x ) ) * real_t( flTimestep ) ),
			static_cast<float>( ( real_t( inverseInertia.y ) * real_t( a.y ) ) * real_t( flTimestep ) ),
			static_cast<float>( ( real_t( inverseInertia.z ) * real_t( a.z ) ) * real_t( flTimestep ) ),
		};
		pObject->SetCoreAngularVelocity( b3Add( pObject->GetCoreAngularVelocity(), change ) );
	};
	const real_t flForceScale = real_t( flTimestep ) * real_t( pObject->GetInvMass() );

	switch ( result )
	{
	case IMotionEvent::SIM_LOCAL_ACCELERATION:
		pObject->SetCoreVelocity( b3MulAdd( pObject->GetCoreVelocity(), flTimestep, pObject->CoreToIVP( l ) ) );
		pObject->SetCoreAngularVelocity( b3MulAdd( pObject->GetCoreAngularVelocity(), flTimestep, a ) );
		break;
	case IMotionEvent::SIM_LOCAL_FORCE:
		pObject->SetCoreVelocity( b3Add( pObject->GetCoreVelocity(), Scale( pObject->CoreToIVP( l ), flForceScale ) ) );
		applyTorque();
		break;
	case IMotionEvent::SIM_GLOBAL_ACCELERATION:
		pObject->SetCoreVelocity( b3MulAdd( pObject->GetCoreVelocity(), flTimestep, l ) );
		pObject->SetCoreAngularVelocity( b3MulAdd( pObject->GetCoreAngularVelocity(), flTimestep, a ) );
		break;
	case IMotionEvent::SIM_GLOBAL_FORCE:
		pObject->SetCoreVelocity( b3Add( pObject->GetCoreVelocity(), Scale( l, flForceScale ) ) );
		applyTorque();
		break;
	default:
		break;
	}

	pObject->ClampVelocity( false );
}

void CPhysicsMotionController::WriteRecord( vphysics_save_motioncontroller_t &record ) const
{
	for ( CPhysicsObject *pObject : m_Objects )
		record.m_objectList.AddToTail( pObject );
	record.m_nPriority = m_nPriority;
}

void CPhysicsMotionController::ReadRecord( const vphysics_save_motioncontroller_t &record )
{
	m_nPriority = record.m_nPriority;
	for ( int i = 0; i < record.m_objectList.Count(); ++i )
		AttachObject( record.m_objectList[i], true );
}

void CPhysicsMotionController::ObjectDestroyed( CPhysicsObject *pObject )
{
	m_Objects.erase( std::remove( m_Objects.begin(), m_Objects.end(), pObject ), m_Objects.end() );
}
