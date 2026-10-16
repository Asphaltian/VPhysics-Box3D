#include "physics_spring.h"

#include "conversion.h"
#include "physics_environment.h"
#include "physics_object.h"
#include "vphysics_saverestore.h"

#include "box3d/box3d.h"

#include <cmath>

static constexpr float MIN_SPRING_LENGTH = 1e-10f;
static constexpr float MIN_LENGTH_CHANGE = SOURCE_TO_IVP * 0.001f;
#if defined( VPHYSICS_GMOD )
static constexpr int INVERSE_LENGTH_REFINEMENTS = 5;
#endif

CPhysicsSpring::CPhysicsSpring( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pStart, CPhysicsObject *pEnd, const springparams_t &params )
: m_pEnvironment( pEnvironment )
, m_pStart( pStart )
, m_pEnd( pEnd )
, m_flLength( params.naturalLength * SOURCE_TO_IVP )
, m_flConstant( params.constant )
, m_flDamping( params.damping )
, m_flRelativeDamping( params.relativeDamping )
, m_bOnlyStretch( params.onlyStretch )
{
	const Vector *const positions[2] = { &params.startPosition, &params.endPosition };
	CPhysicsObject *const objects[2] = { pStart, pEnd };
	for ( int i = 0; i < 2; ++i )
	{
		Vector local = *positions[i];
		if ( !params.useLocalPositions )
			objects[i]->WorldToLocal( &local, *positions[i] );

		RealMatrix3_t rotation;
		RealVec3_t origin;
		objects[i]->GetPose( pEnvironment->GetCurrentTime(), rotation, origin );
		const RealVec3_t world = Rotate( rotation, SourceToIVP( local ) ) + origin;
		m_ObjectAnchors[i] = ToFloat( InverseRotate( rotation, world - origin ) );
		m_CoreAnchors[i] = b3Add( m_ObjectAnchors[i], objects[i]->GetShift() );
	}

	pStart->AttachSpring( this );
	if ( pEnd != pStart )
		pEnd->AttachSpring( this );

	if ( !pStart->IsStatic() )
		pStart->AttachController( this );
	if ( !pEnd->IsStatic() && pEnd != pStart )
		pEnd->AttachController( this );

	if ( pStart == pEnd || pStart->IsStatic() || pEnd->IsStatic() )
		return;

	b3FilterJointDef def = b3DefaultFilterJointDef();
	def.base.bodyIdA = pStart->GetBody();
	def.base.bodyIdB = pEnd->GetBody();
	def.base.collideConnected = true;
	m_Link = b3CreateFilterJoint( pEnvironment->GetWorld(), &def );
	for ( CPhysicsObject *pObject : objects )
	{
		if ( b3Body_IsAwake( pObject->GetBody() ) )
			pEnvironment->NotifyWoken( pObject );
	}
}

CPhysicsSpring::~CPhysicsSpring()
{
	Detach();
}

void CPhysicsSpring::Detach()
{
	if ( !m_pStart )
		return;

	m_pStart->DetachController( this );
	m_pEnd->DetachController( this );
	if ( B3_IS_NON_NULL( m_Link ) )
		b3DestroyJoint( m_Link, false );
	m_Link = b3_nullJointId;

	m_pStart->DetachSpring( this );
	if ( m_pEnd != m_pStart )
		m_pEnd->DetachSpring( this );
	m_pStart = nullptr;
	m_pEnd = nullptr;
}

void CPhysicsSpring::EnsureInSimulation() const
{
	for ( CPhysicsObject *pObject : { m_pStart, m_pEnd } )
	{
		if ( !pObject->IsStatic() )
			pObject->Wake();
	}
}

void CPhysicsSpring::GetEndpoints( Vector *worldPositionStart, Vector *worldPositionEnd )
{
	if ( !m_pStart )
		return;

	if ( worldPositionStart )
		m_pStart->LocalToWorld( worldPositionStart, ToSource( IVPToSource( m_ObjectAnchors[0] ) ) );
	if ( worldPositionEnd )
		m_pEnd->LocalToWorld( worldPositionEnd, ToSource( IVPToSource( m_ObjectAnchors[1] ) ) );
}

void CPhysicsSpring::SetSpringConstant( float flConstant )
{
	if ( !m_pStart || !std::islessgreater( flConstant, m_flConstant ) )
		return;

	m_flConstant = flConstant;
	EnsureInSimulation();
}

void CPhysicsSpring::SetSpringDamping( float flDamping )
{
	if ( !m_pStart )
		return;

	m_flDamping = flDamping;
	EnsureInSimulation();
}

void CPhysicsSpring::SetSpringLength( float flLength )
{
	if ( !m_pStart )
		return;

	const float flNewLength = flLength * SOURCE_TO_IVP;
	if ( !( std::fabs( flNewLength - m_flLength ) >= MIN_LENGTH_CHANGE ) )
		return;

	m_flLength = flNewLength;
	EnsureInSimulation();
}

IPhysicsObject *CPhysicsSpring::GetStartObject()
{
	return m_pStart;
}

IPhysicsObject *CPhysicsSpring::GetEndObject()
{
	return m_pEnd;
}

b3Vec3 CPhysicsSpring::GetAnchorVelocity( const CPhysicsObject *pObject, int nAnchor ) const
{
	return pObject->IsMoveable() ? pObject->GetCoreSurfaceVelocity( m_CoreAnchors[nAnchor] ) : b3Vec3_zero;
}

void CPhysicsSpring::Simulate( CPhysicsObject *, float flTimestep )
{
	if ( m_nSimulatedStep == m_pEnvironment->GetStepCount() )
		return;

	m_nSimulatedStep = m_pEnvironment->GetStepCount();
	const RealVec3_t start = m_pStart->TransformCorePoint( m_CoreAnchors[0] );
	const RealVec3_t end = m_pEnd->TransformCorePoint( m_CoreAnchors[1] );
	const b3Vec3 delta = ToFloat( start - end );
	b3Vec3 direction = delta;
	float flLength = 0.0f;
#if defined( VPHYSICS_GMOD )
	const double flLengthSquared = ( delta.x * delta.x + delta.y * delta.y ) + delta.z * delta.z;
	if ( !( flLengthSquared < REAL_RESOLUTION ) )
	{
		const double flInverse = ApproximateInverseSquareRoot( flLengthSquared, INVERSE_LENGTH_REFINEMENTS );
		direction = Scale( delta, flInverse );
		flLength = static_cast<float>( flInverse * flLengthSquared );
	}
#else
	const float flLengthSquared = ( delta.x * delta.x + delta.y * delta.y ) + delta.z * delta.z;
	if ( flLengthSquared >= REAL_RESOLUTION )
	{
		flLength = std::sqrt( flLengthSquared );
		direction = b3MulSV( 1.0f / flLength, delta );
	}
#endif

	if ( flLength < MIN_SPRING_LENGTH || ( m_bOnlyStretch && !( flLength > m_flLength ) ) )
		return;

	const b3Vec3 relative = b3Sub( GetAnchorVelocity( m_pEnd, 1 ), GetAnchorVelocity( m_pStart, 0 ) );
	const float flTension = ( flLength - m_flLength ) * m_flConstant;
	const float flApproach = ( relative.x * direction.x + relative.y * direction.y ) + relative.z * direction.z;
#if defined( VPHYSICS_GMOD )
	const double flAxial = ( static_cast<double>( flTension ) - static_cast<double>( flApproach ) * static_cast<double>( m_flDamping ) ) * static_cast<double>( flTimestep );
	const double flRelative = static_cast<double>( m_flRelativeDamping ) * static_cast<double>( -flTimestep );
	const auto component = [&]( float flDirection, float flVelocity ) { return static_cast<float>( static_cast<double>( static_cast<float>( static_cast<double>( flDirection ) * flAxial ) ) + static_cast<double>( flVelocity ) * flRelative ); };
#else
	const float flAxial = ( flTension - flApproach * m_flDamping ) * flTimestep;
	const float flRelative = -( flTimestep * m_flRelativeDamping );
	const auto component = [&]( float flDirection, float flVelocity ) { return flVelocity * flRelative + flDirection * flAxial; };
#endif
	const b3Vec3 impulse = { component( direction.x, relative.x ), component( direction.y, relative.y ), component( direction.z, relative.z ) };

	if ( m_pEnd->IsMoveable() && !m_pEnd->IsAsleep() )
		m_pEnd->ApplyImpulse( end, impulse );
	if ( m_pStart->IsMoveable() && !m_pStart->IsAsleep() )
		m_pStart->ApplyImpulse( start, b3MulSV( -1.0f, impulse ) );
}

void CPhysicsSpring::WriteRecord( vphysics_save_cphysicsspring_t &record ) const
{
	if ( !m_pStart )
		return;

	record.constant = m_flConstant;
	record.naturalLength = m_flLength * IVP_TO_SOURCE;
	record.damping = m_flDamping;
	record.relativeDamping = m_flRelativeDamping;
	record.startPosition = ToSource( IVPToSource( m_ObjectAnchors[0] ) );
	record.endPosition = ToSource( IVPToSource( m_ObjectAnchors[1] ) );
	record.useLocalPositions = true;
	record.onlyStretch = m_bOnlyStretch;
	record.pObjStart = m_pStart;
	record.pObjEnd = m_pEnd;
}

void CPhysicsSpring::ObjectDestroyed( CPhysicsObject * )
{
	Detach();
}
