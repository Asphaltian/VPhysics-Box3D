#include "physics_constraint.h"

#include "conversion.h"
#include "physics_environment.h"
#include "physics_object.h"
#include "vphysics_saverestore.h"

#include "tier0/dbg.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <numbers>

static constexpr int PRIVATE_MIN_ERROR_TICKS = 1;
static constexpr float PRIVATE_ERROR_TOLERANCE = 0.03f;
static constexpr float UNBREAKABLE_LIMIT = 1e12f;
static constexpr float FULL_CIRCLE_TOLERANCE = 1e-4f;
static constexpr float MIN_AXIS_RANGE = 1e-16f;
static constexpr float LOCKED_CONE_ANGLE = 0.1f;
static constexpr float PENETRATION_MIN_DISTANCE = 0.01f;
static constexpr float IVP_TO_SOURCE_SQUARED = IVP_TO_SOURCE * IVP_TO_SOURCE;

static constexpr int IVP_TO_SOURCE_AXIS[3] = { 0, 2, 1 };
static constexpr float CONE_UPPER_BOUND = 100.0f;
static constexpr float MAX_SCALED_MASS = 10000.0f;
static constexpr float ITERATION_ALPHA[CPhysicsConstraintGroup::MAX_ITERATIONS] = { 1.0f, 1.0f, 0.8f, 0.6f, 0.4f, 0.4f, 0.4f, 0.4f, 0.4f };
static constexpr float ITERATION_BETA[CPhysicsConstraintGroup::MAX_ITERATIONS] = { 1.0f, 1.0f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f };

static bool IsFullCircle( float flLower, float flUpper )
{
	return std::fabs( flLower + PI ) <= FULL_CIRCLE_TOLERANCE && std::fabs( flUpper - PI ) <= FULL_CIRCLE_TOLERANCE;
}

static b3Vec3 NormalizeOrZero( b3Vec3 v )
{
	const float flLength = b3Length( v );
	return flLength > 0.0f ? b3MulSV( 1.0f / flLength, v ) : b3Vec3_zero;
}

static b3Vec3 MatrixColumn( const matrix3x4_t &m, int nColumn )
{
	return { m[0][nColumn], m[1][nColumn], m[2][nColumn] };
}

static void SetMatrixColumn( matrix3x4_t &m, int nColumn, b3Vec3 v )
{
	m[0][nColumn] = v.x;
	m[1][nColumn] = v.y;
	m[2][nColumn] = v.z;
}

static void SetIdentity( matrix3x4_t &m )
{
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 4; ++j )
			m[i][j] = i == j ? 1.0f : 0.0f;
	}
}

static b3Vec3 SourceToIVPAxis( b3Vec3 v )
{
	return { v.x, -v.z, v.y };
}

static b3Vec3 IVPToSourceAxis( b3Vec3 v )
{
	return { v.x, v.z, -v.y };
}

static void ConvertRagdollAxes( const matrix3x4_t &m, b3Vec3 axes[3] )
{
	axes[0] = SourceToIVPAxis( MatrixColumn( m, 0 ) );
	axes[1] = b3Neg( SourceToIVPAxis( MatrixColumn( m, 2 ) ) );
	axes[2] = SourceToIVPAxis( MatrixColumn( m, 1 ) );
}

static void RotateRagdollAxes( b3Vec3 axes[3], int nAxis, float flAngle )
{
	const int nFirst = ( nAxis + 1 ) % 3, nSecond = ( nFirst + 1 ) % 3;
	const float flSine = std::sin( flAngle ), flCosine = std::cos( flAngle );
	float rotation[3][3] = {};
	rotation[nFirst][nFirst] = flCosine;
	rotation[nSecond][nSecond] = flCosine;
	rotation[nFirst][nSecond] = flSine;
	rotation[nSecond][nFirst] = -flSine;
	rotation[nAxis][nAxis] = 1.0f;

	b3Vec3 rotated[3];
	for ( int i = 0; i < 3; ++i )
		rotated[i] = b3Add( b3Add( b3MulSV( rotation[i][0], axes[0] ), b3MulSV( rotation[i][1], axes[1] ) ), b3MulSV( rotation[i][2], axes[2] ) );
	for ( int i = 0; i < 3; ++i )
		axes[i] = rotated[i];
}

static b3Vec3 RotateRows( const DoubleMatrix3_t &rotation, b3Vec3 v )
{
	const auto &m = rotation.m;
	const auto element = []( double x ) { return static_cast<float>( x ); };
	return {
		( v.y * element( m[1][0] ) + v.x * element( m[0][0] ) ) + v.z * element( m[2][0] ),
		( v.y * element( m[1][1] ) + v.x * element( m[0][1] ) ) + v.z * element( m[2][1] ),
		( v.y * element( m[1][2] ) + v.x * element( m[0][2] ) ) + v.z * element( m[2][2] ),
	};
}

static float GetLeverSquared( b3Vec3 r, b3Vec3 v )
{
	const b3Vec3 x = { v.z * r.y - v.y * r.z, v.x * r.z - v.z * r.x, v.y * r.x - v.x * r.y };
	return ( x.y * x.y + x.x * x.x ) + x.z * x.z;
}

static b3Quat MakeFrameRotation( b3Vec3 axisX, b3Vec3 axisZ )
{
	const b3Vec3 z = NormalizeOrZero( axisZ );
	if ( b3LengthSquared( z ) == 0.0f )
		return b3Quat_identity;

	b3Vec3 x = b3MulSub( axisX, b3Dot( axisX, z ), z );
	x = b3LengthSquared( x ) > 0.0f ? b3Normalize( x ) : b3Perp( z );
	const b3Matrix3 rotation = { x, b3Cross( z, x ), z };
	return b3MakeQuatFromMatrix( &rotation );
}

static b3Quat MakeSlideRotation( b3Vec3 axis )
{
	const b3Vec3 x = NormalizeOrZero( axis );
	if ( b3LengthSquared( x ) == 0.0f )
		return b3Quat_identity;

	const b3Vec3 y = b3Perp( x );
	const b3Matrix3 rotation = { x, y, b3Cross( x, y ) };
	return b3MakeQuatFromMatrix( &rotation );
}

static b3Transform MatrixToTransform( const matrix3x4_t &m )
{
	const b3Matrix3 rotation = { MatrixColumn( m, 0 ), MatrixColumn( m, 1 ), MatrixColumn( m, 2 ) };
	return { MatrixColumn( m, 3 ), b3NormalizeQuat( b3MakeQuatFromMatrix( &rotation ) ) };
}

static void ConcatenateMatrices( const matrix3x4_t &a, const matrix3x4_t &b, matrix3x4_t &out )
{
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
			out[i][j] = ( a[i][0] * b[0][j] + a[i][1] * b[1][j] ) + a[i][2] * b[2][j];
		out[i][3] = ( ( a[i][0] * b[0][3] + a[i][1] * b[1][3] ) + a[i][2] * b[2][3] ) + a[i][3];
	}
}

static b3Vec3 RoundTripPosition( b3Vec3 position )
{
	return IVPToSource( SourceToIVP( position ) );
}

b3JointDef &CPhysicsConstraint::GetBaseDefinition( JointDef_t &def, b3JointType nType )
{
	switch ( nType )
	{
	case b3_revoluteJoint:
		return def.m_Revolute.base;
	case b3_prismaticJoint:
		return def.m_Prismatic.base;
	case b3_weldJoint:
		return def.m_Weld.base;
	case b3_sphericalJoint:
		return def.m_Spherical.base;
	case b3_distanceJoint:
		return def.m_Distance.base;
	default:
		return def.m_Filter.base;
	}
}

static b3Vec3 GetWorldLocal( b3BodyId body, const Vector &worldPosition )
{
	return b3InvTransformWorldPoint( b3Body_GetTransform( body ), b3ToPos( ToBox3D( worldPosition ) ) );
}

CPhysicsConstraintGroup::CPhysicsConstraintGroup( CPhysicsEnvironment *pEnvironment, int nIterations, int nMinErrorTicks, float flErrorTolerance )
: m_pEnvironment( pEnvironment )
, m_nIterations( nIterations )
, m_nMinErrorTicks( nMinErrorTicks )
, m_flErrorTolerance( flErrorTolerance )
{
}

CPhysicsConstraintGroup::~CPhysicsConstraintGroup()
{
	for ( CPhysicsConstraint *pConstraint : m_Constraints )
	{
		pConstraint->DestroyJoint( false );
		pConstraint->GroupDestroyed();
	}

	if ( m_bActive )
	{
		for ( CPhysicsObject *pObject : m_Objects )
		{
			if ( !pObject->IsStatic() )
				pObject->DetachController( this );
		}
	}
}

void CPhysicsConstraintGroup::Activate()
{
	if ( m_bActive || m_Objects.empty() )
		return;

	m_bActive = true;
	for ( CPhysicsObject *pObject : m_Objects )
	{
		if ( !pObject->IsStatic() )
			pObject->AttachController( this );
	}

	for ( CPhysicsConstraint *pConstraint : m_Constraints )
		pConstraint->CreateJoint();
}

void CPhysicsConstraintGroup::Deactivate()
{
	if ( !m_bActive || std::none_of( m_Objects.begin(), m_Objects.end(), []( const CPhysicsObject *p ) { return !p->IsStatic(); } ) )
		return;

	m_bActive = false;
	for ( CPhysicsObject *pObject : m_Objects )
	{
		if ( !pObject->IsStatic() )
			pObject->DetachController( this );
	}

	for ( CPhysicsConstraint *pConstraint : m_Constraints )
		pConstraint->DestroyJoint( false );

	for ( CPhysicsObject *pObject : m_Objects )
	{
		if ( !pObject->IsStatic() )
			pObject->Wake();
	}
}

void CPhysicsConstraintGroup::GetErrorParams( constraint_groupparams_t *pParams )
{
	pParams->additionalIterations = m_nIterations - BASE_ITERATIONS;
	pParams->minErrorTicks = m_nMinErrorTicks;
	pParams->errorTolerance = m_flErrorTolerance * IVP_TO_SOURCE;
}

void CPhysicsConstraintGroup::SetErrorParams( const constraint_groupparams_t &params )
{
	m_nMinErrorTicks = params.minErrorTicks;
	m_flErrorTolerance = params.errorTolerance * SOURCE_TO_IVP;
}

void CPhysicsConstraintGroup::SolvePenetration( IPhysicsObject *pObj0, IPhysicsObject *pObj1 )
{
	if ( !pObj0 || !pObj1 || m_nPenetrationPairs >= MAX_PENETRATION_PAIRS )
		return;

	CPhysicsObject *pObject0 = static_cast<CPhysicsObject *>( pObj0 );
	CPhysicsObject *pObject1 = static_cast<CPhysicsObject *>( pObj1 );
	if ( std::find( m_Objects.begin(), m_Objects.end(), pObject0 ) == m_Objects.end() || std::find( m_Objects.begin(), m_Objects.end(), pObject1 ) == m_Objects.end() )
		return;

	m_PenetrationPairs[m_nPenetrationPairs++] = { pObject0, pObject1 };
}

void CPhysicsConstraintGroup::Simulate( CPhysicsObject *, float flTimestep )
{
	if ( m_nSimulatedStep == m_pEnvironment->GetStepCount() )
		return;

	m_nSimulatedStep = m_pEnvironment->GetStepCount();
	if ( m_bOrderDirty )
	{
		std::stable_partition( m_Constraints.begin(), m_Constraints.end(), []( const CPhysicsConstraint *p ) { return !p->IsFixed(); } );
		m_bOrderDirty = false;
	}

	const int nCount = static_cast<int>( m_Constraints.size() );
	if ( m_nRowConstraints > 0 )
	{
		const ConstraintStep_t step = { flTimestep, m_pEnvironment->GetInverseTimestep() };
		for ( int i = 0; i < nCount; ++i )
		{
			if ( m_Constraints[i]->IsSolvedByRows() )
				m_Constraints[i]->SetupRows( step );
		}

		for ( int nIteration = 0; nIteration < std::min( m_nIterations, MAX_ITERATIONS ); ++nIteration )
		{
			for ( int i = nCount - 1; i >= 0; --i )
			{
				if ( m_Constraints[i]->IsSolvedByRows() )
					m_Constraints[i]->SolveRows( ITERATION_ALPHA[nIteration], ITERATION_BETA[nIteration], step );
			}
			for ( int i = 0; i < nCount; ++i )
			{
				if ( m_Constraints[i]->IsSolvedByRows() )
					m_Constraints[i]->SolveRows( ITERATION_ALPHA[nIteration], ITERATION_BETA[nIteration], step );
			}
		}
	}

	float flError = 0.0f;
	for ( int i = 0; i < nCount; ++i )
	{
		CPhysicsConstraint *pConstraint = m_Constraints[i];
		pConstraint->UpdateBreakThresholds( flTimestep );
		if ( pConstraint->ReportsError() )
		{
			const float flSeparation = pConstraint->GetJointSeparation() * SOURCE_TO_IVP;
			flError = std::max( flError, flSeparation * flSeparation );
		}
	}

	ApplyPenetrationPairs( flTimestep );

	if ( flError <= m_flErrorTolerance * m_flErrorTolerance )
		m_nErrorTicks = 0;
	else if ( m_nErrorTicks <= m_nMinErrorTicks )
		++m_nErrorTicks;
}

void CPhysicsConstraintGroup::ApplyPenetrationPairs( float flTimestep )
{
	if ( m_nPenetrationPairs == 0 )
		return;

	float flMass = 0.0f;
	for ( const CPhysicsObject *pObject : m_Objects )
	{
		if ( pObject->IsMoveable() )
			flMass += pObject->GetMass();
	}

	const float flImpulse = ( ( flMass * 2.0f ) * static_cast<float>( Length( m_pEnvironment->GetIVPGravity() ) ) ) * flTimestep;
	for ( int i = 0; i < m_nPenetrationPairs; ++i )
	{
		auto [pObject0, pObject1] = m_PenetrationPairs[i];
		const DoubleVec3_t position0 = pObject0->GetCorePosition();
		const b3Vec3 delta = ToFloat( pObject1->GetCorePosition() - position0 );
		const float flLength = b3Length( delta );
		const b3Vec3 normal = flLength <= PENETRATION_MIN_DISTANCE ? b3Vec3{ 1.0f, 0.0f, 0.0f } : b3MulSV( 1.0f / flLength, delta );
		const b3Vec3 impulse = b3MulSV( flImpulse, normal );

		if ( pObject1->IsMoveable() && !pObject1->IsAsleep() )
		{
			pObject1->ApplyImpulse( position0, impulse );
			pObject1->AddPendingAngularVelocity( { flTimestep, 0.0f, 0.0f } );
		}

		if ( pObject0->IsMoveable() && !pObject0->IsAsleep() )
		{
			pObject0->ApplyImpulse( position0, b3Neg( impulse ) );
			pObject0->AddPendingAngularVelocity( { -flTimestep, 0.0f, 0.0f } );
		}
	}
	m_nPenetrationPairs = 0;
}

void CPhysicsConstraintGroup::ObjectDestroyed( CPhysicsObject *pObject )
{
	m_Objects.erase( std::remove( m_Objects.begin(), m_Objects.end(), pObject ), m_Objects.end() );
	RemovePenetrationPairs( pObject );
}

void CPhysicsConstraintGroup::RemovePenetrationPairs( const CPhysicsObject *pObject )
{
	int nKept = 0;
	for ( int i = 0; i < m_nPenetrationPairs; ++i )
	{
		if ( m_PenetrationPairs[i].first != pObject && m_PenetrationPairs[i].second != pObject )
			m_PenetrationPairs[nKept++] = m_PenetrationPairs[i];
	}
	m_nPenetrationPairs = nKept;
}

void CPhysicsConstraintGroup::AddObject( CPhysicsObject *pObject )
{
	if ( !pObject || std::find( m_Objects.begin(), m_Objects.end(), pObject ) != m_Objects.end() )
		return;

	m_Objects.push_back( pObject );
	if ( m_bActive && !pObject->IsStatic() )
		pObject->AttachController( this );
}

void CPhysicsConstraintGroup::AddConstraint( CPhysicsConstraint *pConstraint )
{
	m_Constraints.push_back( pConstraint );
	if ( pConstraint->IsSolvedByRows() )
		++m_nRowConstraints;
	AddObject( pConstraint->GetReference() );
	AddObject( pConstraint->GetAttached() );
	m_bOrderDirty = true;
	if ( !m_bActive )
		return;

	for ( CPhysicsObject *pObject : m_Objects )
	{
		if ( !pObject->IsStatic() )
			pObject->Wake();
	}
	pConstraint->CreateJoint();
}

void CPhysicsConstraintGroup::RemoveConstraint( CPhysicsConstraint *pConstraint )
{
	auto it = std::find( m_Constraints.begin(), m_Constraints.end(), pConstraint );
	if ( it == m_Constraints.end() )
		return;

	m_Constraints.erase( it );
	if ( pConstraint->IsSolvedByRows() )
		--m_nRowConstraints;
	if ( !m_Constraints.empty() )
		RebuildObjects();
}

void CPhysicsConstraintGroup::RebuildObjects()
{
	std::vector<CPhysicsObject *> objects;
	for ( const CPhysicsConstraint *pConstraint : m_Constraints )
	{
		for ( CPhysicsObject *pObject : { pConstraint->GetReference(), pConstraint->GetAttached() } )
		{
			if ( pObject && std::find( objects.begin(), objects.end(), pObject ) == objects.end() )
				objects.push_back( pObject );
		}
	}

	for ( CPhysicsObject *pObject : m_Objects )
	{
		if ( std::find( objects.begin(), objects.end(), pObject ) != objects.end() )
			continue;

		if ( m_bActive && !pObject->IsStatic() )
			pObject->DetachController( this );
		RemovePenetrationPairs( pObject );
	}
	m_Objects = std::move( objects );
}

bool CPhysicsConstraintGroup::HasExternalObject( const CPhysicsObject *pObject ) const
{
	return std::any_of( m_Objects.begin(), m_Objects.end(), [&]( const CPhysicsObject *p ) { return !p->IsStatic() && p->GetGameData() != pObject->GetGameData(); } );
}

void CPhysicsConstraintGroup::WriteRecord( vphysics_save_cphysicsconstraintgroup_t &record ) const
{
	record.additionalIterations = m_nIterations - BASE_ITERATIONS;
	record.minErrorTicks = m_nMinErrorTicks;
	record.errorTolerance = m_flErrorTolerance * IVP_TO_SOURCE;
	record.isActive = m_bActive;
}

void CPhysicsConstraintGroup::OutputDebugInfo() const
{
	const int nCount = static_cast<int>( m_Constraints.size() );
	Msg( "System of %d constraints\n", nCount );
	for ( int i = 0; i < nCount; ++i )
		Msg( "\tConstraint %d) %s\n", i, m_Constraints[nCount - 1 - i]->GetSolverName() );
}

CPhysicsConstraint::CPhysicsConstraint( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, Type_t nType )
: m_pEnvironment( pEnvironment )
, m_pReference( pReference && pAttached ? pReference : nullptr )
, m_pAttached( pReference && pAttached ? pAttached : nullptr )
, m_nType( nType )
{
	if ( !m_pReference )
		return;

	m_bListenerRegistered[0] = !( m_pReference->GetCallbackFlags() & CALLBACK_NEVER_DELETED );
	m_bListenerRegistered[1] = !( m_pAttached->GetCallbackFlags() & CALLBACK_NEVER_DELETED );
	m_pReference->AttachConstraint( this );
	if ( m_pAttached != m_pReference )
		m_pAttached->AttachConstraint( this );
}

CPhysicsConstraint::~CPhysicsConstraint()
{
	DetachObjects();
	DestroyJoint( false );
	if ( m_pGroup )
		m_pGroup->RemoveConstraint( this );
	delete m_pPrivateGroup;
}

void CPhysicsConstraint::Initialize( CPhysicsConstraintGroup *pGroup, const constraint_breakableparams_t &constraint, bool bActivate )
{
	m_bBreakable = ( constraint.forceLimit != 0.0f && constraint.forceLimit < UNBREAKABLE_LIMIT ) || ( constraint.torqueLimit != 0.0f && constraint.torqueLimit < UNBREAKABLE_LIMIT ) || ( constraint.bodyMassScale[0] != 1.0f && constraint.bodyMassScale[0] != 0.0f ) || ( constraint.bodyMassScale[1] != 1.0f && constraint.bodyMassScale[1] != 0.0f );
	if ( m_bBreakable )
	{
		const float flForceLimit = constraint.forceLimit * SOURCE_TO_IVP;
		const float flTorqueLimit = constraint.torqueLimit * DEGREES_TO_RADIANS;
		m_flForceLimit = flForceLimit <= 0.0f ? UNBREAKABLE_LIMIT : flForceLimit;
		m_flTorqueLimit = flTorqueLimit <= 0.0f ? UNBREAKABLE_LIMIT : flTorqueLimit;
		for ( int i = 0; i < 2; ++i )
			m_BodyMassScale[i] = constraint.bodyMassScale[i] <= 0.0f ? 1.0f : constraint.bodyMassScale[i];
	}

	if ( !pGroup )
	{
		m_pPrivateGroup = new CPhysicsConstraintGroup( m_pEnvironment, CPhysicsConstraintGroup::BASE_ITERATIONS, PRIVATE_MIN_ERROR_TICKS, PRIVATE_ERROR_TOLERANCE );
		pGroup = m_pPrivateGroup;
	}

	m_pGroup = pGroup;
	m_pGroup->AddConstraint( this );
	if ( m_pPrivateGroup && bActivate )
		m_pPrivateGroup->Activate();
}

CPhysicsConstraint *CPhysicsConstraint::CreateRagdoll( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	int nFreeAxis = -1, nFreeCount = 0;
	for ( int i = 0; i < 3; ++i )
	{
		if ( ragdoll.axes[i].minRotation != ragdoll.axes[i].maxRotation )
		{
			nFreeAxis = i;
			++nFreeCount;
		}
	}

	if ( nFreeCount == 1 )
	{
		matrix3x4_t referenceMatrix, world;
		if ( pReference )
			pReference->GetPositionMatrix( &referenceMatrix );
		else
			SetIdentity( referenceMatrix );
		ConcatenateMatrices( referenceMatrix, ragdoll.constraintToReference, world );

		constraint_limitedhingeparams_t hinge;
		hinge.worldPosition = ToSource( MatrixColumn( world, 3 ) );
		hinge.worldAxisDirection = ToSource( MatrixColumn( world, nFreeAxis ) );

		Vector perpendicular( 0.0f, 0.0f, 0.0f );
		perpendicular[( nFreeAxis + 1 ) % 3] = 1.0f;
		hinge.referencePerpAxisDirection = perpendicular;
		hinge.attachedPerpAxisDirection = MatrixRotateVector( ragdoll.constraintToAttached, MatrixInverseRotateVector( ragdoll.constraintToReference, perpendicular ) );

		hinge.hingeAxis = ragdoll.axes[nFreeAxis];
		hinge.hingeAxis.torque = ( ragdoll.axes[nFreeAxis].torque * ( pReference ? pReference->GetMass() : 0.0f ) ) * RADIANS_TO_DEGREES;
		if ( !ragdoll.useClockwiseRotations )
		{
			hinge.hingeAxis.minRotation = -ragdoll.axes[nFreeAxis].maxRotation;
			hinge.hingeAxis.maxRotation = -ragdoll.axes[nFreeAxis].minRotation;
		}
		hinge.constraint = ragdoll.constraint;
		return CreateHinge( pEnvironment, pReference, pAttached, pGroup, hinge );
	}

	CPhysicsConstraint *pConstraint = new CPhysicsConstraint( pEnvironment, pReference, pAttached, CONSTRAINT_RAGDOLL );
	pConstraint->BuildRagdoll( ragdoll );
	pConstraint->Initialize( pGroup, ragdoll.constraint, ragdoll.isActive );
	return pConstraint;
}

CPhysicsConstraint *CPhysicsConstraint::CreateHinge( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_limitedhingeparams_t &hinge )
{
	CPhysicsConstraint *pConstraint = new CPhysicsConstraint( pEnvironment, pReference, pAttached, CONSTRAINT_HINGE );
	pConstraint->BuildHinge( hinge );
	pConstraint->Initialize( pGroup, hinge.constraint, hinge.constraint.isActive );
	return pConstraint;
}

CPhysicsConstraint *CPhysicsConstraint::CreateFixed( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed )
{
	CPhysicsConstraint *pConstraint = new CPhysicsConstraint( pEnvironment, pReference, pAttached, CONSTRAINT_FIXED );
	b3WeldJointDef &def = pConstraint->m_Definition.m_Weld;
	def = b3DefaultWeldJointDef();
	def.base.localFrameA = MatrixToTransform( fixed.attachedRefXform );
	def.base.localFrameB = b3Transform_identity;
	def.base.constraintHertz *= std::max( fixed.constraint.strength, 0.0f );
	pConstraint->m_nJointType = b3_weldJoint;
	pConstraint->m_AttachedRefXform = fixed.attachedRefXform;
	pConstraint->Initialize( pGroup, fixed.constraint, fixed.constraint.isActive );
	return pConstraint;
}

CPhysicsConstraint *CPhysicsConstraint::CreateSliding( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding )
{
	CPhysicsConstraint *pConstraint = new CPhysicsConstraint( pEnvironment, pReference, pAttached, CONSTRAINT_SLIDING );
	b3PrismaticJointDef &def = pConstraint->m_Definition.m_Prismatic;
	def = b3DefaultPrismaticJointDef();

	const b3Transform attachedToReference = MatrixToTransform( sliding.attachedRefXform );
	const b3Transform frameA = { attachedToReference.p, MakeSlideRotation( ToBox3D( sliding.slideAxisRef ) ) };
	def.base.localFrameA = frameA;
	def.base.localFrameB = b3MulTransforms( b3InvertTransform( attachedToReference ), frameA );
	def.base.constraintHertz *= std::max( sliding.constraint.strength, 0.0f );

	if ( sliding.limitMin != sliding.limitMax && !IsFullCircle( sliding.limitMin * SOURCE_TO_IVP, sliding.limitMax * SOURCE_TO_IVP ) )
	{
		def.enableLimit = true;
		def.lowerTranslation = std::min( sliding.limitMin, sliding.limitMax );
		def.upperTranslation = std::max( sliding.limitMin, sliding.limitMax );
	}

	if ( sliding.friction != 0.0f )
	{
		def.enableMotor = true;
		def.maxMotorForce = std::fabs( sliding.friction );
		def.motorSpeed = sliding.velocity;
	}

	pConstraint->m_nJointType = b3_prismaticJoint;
	pConstraint->m_AttachedRefXform = sliding.attachedRefXform;
	pConstraint->m_SlideAxis = NormalizeOrZero( ToBox3D( sliding.slideAxisRef ) );
	pConstraint->Initialize( pGroup, sliding.constraint, sliding.constraint.isActive );
	return pConstraint;
}

CPhysicsConstraint *CPhysicsConstraint::CreateBallsocket( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket )
{
	CPhysicsConstraint *pConstraint = new CPhysicsConstraint( pEnvironment, pReference, pAttached, CONSTRAINT_BALLSOCKET );
	b3SphericalJointDef &def = pConstraint->m_Definition.m_Spherical;
	def = b3DefaultSphericalJointDef();
	def.base.localFrameA = { ToBox3D( ballsocket.constraintPosition[0] ), b3Quat_identity };
	def.base.localFrameB = { ToBox3D( ballsocket.constraintPosition[1] ), b3Quat_identity };
	def.base.constraintHertz *= std::max( ballsocket.constraint.strength, 0.0f );
	pConstraint->m_nJointType = b3_sphericalJoint;
	pConstraint->m_Anchors[0] = ToBox3D( ballsocket.constraintPosition[0] );
	pConstraint->m_Anchors[1] = ToBox3D( ballsocket.constraintPosition[1] );
	pConstraint->Initialize( pGroup, ballsocket.constraint, ballsocket.constraint.isActive );
	return pConstraint;
}

CPhysicsConstraint *CPhysicsConstraint::CreateEmpty( CPhysicsEnvironment *pEnvironment )
{
	return new CPhysicsConstraint( pEnvironment, nullptr, nullptr, CONSTRAINT_NONE );
}

CPhysicsConstraint *CPhysicsConstraint::CreateLength( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length )
{
	CPhysicsConstraint *pConstraint = new CPhysicsConstraint( pEnvironment, pReference, pAttached, CONSTRAINT_LENGTH );
	b3DistanceJointDef &def = pConstraint->m_Definition.m_Distance;
	def = b3DefaultDistanceJointDef();
	def.base.localFrameA = { ToBox3D( length.objectPosition[0] ), b3Quat_identity };
	def.base.localFrameB = { ToBox3D( length.objectPosition[1] ), b3Quat_identity };
	def.base.constraintHertz *= std::max( length.constraint.strength, 0.0f );

	const float flMaxLength = std::max( length.totalLength, B3_LINEAR_SLOP );
	def.length = flMaxLength;
	if ( length.minLength * SOURCE_TO_IVP < length.totalLength * SOURCE_TO_IVP )
	{
		def.enableSpring = true;
		def.enableLimit = true;
		def.minLength = length.minLength;
		def.maxLength = flMaxLength;
	}

	pConstraint->m_nJointType = b3_distanceJoint;
	pConstraint->Initialize( pGroup, length.constraint, length.constraint.isActive );
	return pConstraint;
}

CPhysicsConstraint *CPhysicsConstraint::CreatePulley( CPhysicsEnvironment *pEnvironment, CPhysicsObject *pReference, CPhysicsObject *pAttached, CPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley )
{
	CPhysicsConstraint *pConstraint = new CPhysicsConstraint( pEnvironment, pReference, pAttached, CONSTRAINT_PULLEY );
	const b3Vec3 anchors[2] = { SourceToIVP( pulley.objectPosition[0] ), SourceToIVP( pulley.objectPosition[1] ) };
	const b3Vec3 pulleys[2] = { SourceToIVP( pulley.pulleyPosition[0] ), SourceToIVP( pulley.pulleyPosition[1] ) };
	pConstraint->m_Pulley.Initialize( anchors, pulleys, pulley.totalLength * SOURCE_TO_IVP, pulley.gearRatio, pulley.constraint.strength, pulley.isRigid );
	pConstraint->m_Definition.m_Filter = b3DefaultFilterJointDef();
	pConstraint->m_nJointType = b3_filterJoint;
	pConstraint->Initialize( pGroup, pulley.constraint, pulley.constraint.isActive );
	return pConstraint;
}

void CPhysicsConstraint::BuildHinge( const constraint_limitedhingeparams_t &hinge )
{
	b3RevoluteJointDef &def = m_Definition.m_Revolute;
	def = b3DefaultRevoluteJointDef();
	def.base.constraintHertz *= std::max( hinge.constraint.strength, 0.0f );
	m_nJointType = b3_revoluteJoint;

	if ( m_pReference )
	{
		const b3WorldTransform transformA = b3Body_GetTransform( m_pReference->GetBody() );
		const b3WorldTransform transformB = b3Body_GetTransform( m_pAttached->GetBody() );
		const b3Vec3 axis = NormalizeOrZero( ToBox3D( hinge.worldAxisDirection ) );
		const b3Vec3 axisA = b3InvRotateVector( transformA.q, axis );
		const b3Vec3 axisB = b3InvRotateVector( transformB.q, axis );

		b3Vec3 perpendicularA = NormalizeOrZero( ToBox3D( hinge.referencePerpAxisDirection ) );
		if ( b3LengthSquared( perpendicularA ) == 0.0f )
			perpendicularA = b3Perp( axisA );

		b3Vec3 perpendicularB = NormalizeOrZero( ToBox3D( hinge.attachedPerpAxisDirection ) );
		if ( b3LengthSquared( perpendicularB ) == 0.0f )
			perpendicularB = b3InvRotateVector( transformB.q, b3RotateVector( transformA.q, perpendicularA ) );

		def.base.localFrameA = { GetWorldLocal( m_pReference->GetBody(), hinge.worldPosition ), MakeFrameRotation( perpendicularA, axisA ) };
		def.base.localFrameB = { GetWorldLocal( m_pAttached->GetBody(), hinge.worldPosition ), MakeFrameRotation( perpendicularB, axisB ) };
	}

	const constraint_axislimit_t &axis = hinge.hingeAxis;
	if ( axis.minRotation != axis.maxRotation )
	{
		const float flLower = axis.minRotation * DEGREES_TO_RADIANS;
		const float flUpper = axis.maxRotation * DEGREES_TO_RADIANS;
		if ( !IsFullCircle( flLower, flUpper ) )
		{
			def.enableLimit = true;
			def.lowerAngle = -flUpper;
			def.upperAngle = -flLower;
		}
	}

	if ( axis.torque != 0.0f )
	{
		def.enableMotor = true;
		def.maxMotorTorque = ( std::fabs( axis.torque ) * DEGREES_TO_RADIANS ) * IVP_TO_SOURCE_SQUARED;
		def.motorSpeed = -( axis.angularVelocity * DEGREES_TO_RADIANS );
	}
}

void CPhysicsConstraint::BuildRagdoll( const constraint_ragdollparams_t &ragdoll )
{
	m_bOnlyAngularLimits = ragdoll.onlyAngularLimits;

	float lower[3], upper[3], range[3];
	int nFree = 0, nFreeAxis = 0;
	for ( int i = 0; i < 3; ++i )
	{
		const constraint_axislimit_t &axis = ragdoll.axes[IVP_TO_SOURCE_AXIS[i]];
		float flLower = i == 1 ? axis.maxRotation * -DEGREES_TO_RADIANS : axis.minRotation * DEGREES_TO_RADIANS;
		float flUpper = i == 1 ? axis.minRotation * -DEGREES_TO_RADIANS : axis.maxRotation * DEGREES_TO_RADIANS;
		if ( ragdoll.useClockwiseRotations )
		{
			const float flSwap = flLower;
			flLower = -flUpper;
			flUpper = -flSwap;
		}
		lower[i] = flLower;
		upper[i] = flUpper;
		range[i] = flUpper - flLower;
		if ( range[i] > MIN_AXIS_RANGE )
		{
			++nFree;
			nFreeAxis = i;
		}
	}

	b3Vec3 columns[2][3];
	ConvertRagdollAxes( ragdoll.constraintToReference, columns[0] );
	ConvertRagdollAxes( ragdoll.constraintToAttached, columns[1] );
	m_RagdollFrames.m_Origins[0] = SourceToIVP( MatrixColumn( ragdoll.constraintToReference, 3 ) );
	m_RagdollFrames.m_Origins[1] = SourceToIVP( MatrixColumn( ragdoll.constraintToAttached, 3 ) );

	int nTwist = 0, nCone = 2, nPlane = 1;
	if ( nFree == 1 )
	{
		nCone = nFreeAxis;
		nTwist = ( nFreeAxis + 1 ) % 3;
		nPlane = ( nFreeAxis + 2 ) % 3;
	}
	else if ( nFree >= 2 )
	{
		nTwist = SelectTwistAxis( columns );
		const int nNext = ( nTwist + 1 ) % 3, nPrevious = ( nTwist + 2 ) % 3;
		nCone = range[nNext] > range[nPrevious] ? nNext : nPrevious;
		nPlane = nCone == nNext ? nPrevious : nNext;
	}

	if ( nFree >= 1 )
		RotateRagdollAxes( columns[0], nCone, ( upper[nCone] + lower[nCone] ) * 0.5f );

	m_RagdollFrames.m_SlotAxes[CRagdollSolver::SLOT_TWIST] = nTwist;
	m_RagdollFrames.m_SlotAxes[CRagdollSolver::SLOT_CONE] = nCone;
	m_RagdollFrames.m_SlotAxes[CRagdollSolver::SLOT_PLANE] = nPlane;
	for ( int i = 0; i < 2; ++i )
		StoreRagdollRows( i, columns[i] );
	UpdateRagdollSolverAxes();

	float ( &limits )[CRagdollSolver::SLOT_COUNT][2] = m_RagdollLimits;
	limits[CRagdollSolver::SLOT_TWIST][0] = 0.0f;
	limits[CRagdollSolver::SLOT_TWIST][1] = 0.0f;
	limits[CRagdollSolver::SLOT_CONE][0] = -LOCKED_CONE_ANGLE;
	limits[CRagdollSolver::SLOT_CONE][1] = LOCKED_CONE_ANGLE;
	limits[CRagdollSolver::SLOT_PLANE][0] = 0.0f;
	limits[CRagdollSolver::SLOT_PLANE][1] = 0.0f;
	if ( nFree == 1 )
	{
		float flConeRange = range[nCone];
		if ( PI < flConeRange && flConeRange < TWO_PI )
			flConeRange = PI;
		limits[CRagdollSolver::SLOT_CONE][0] = flConeRange * -0.5f;
		limits[CRagdollSolver::SLOT_CONE][1] = flConeRange * 0.5f;
	}
	else if ( nFree >= 2 )
	{
		limits[CRagdollSolver::SLOT_TWIST][0] = -upper[nTwist];
		limits[CRagdollSolver::SLOT_TWIST][1] = -lower[nTwist];
		limits[CRagdollSolver::SLOT_CONE][0] = range[nCone] * -0.5f;
		limits[CRagdollSolver::SLOT_CONE][1] = range[nCone] * 0.5f;
		limits[CRagdollSolver::SLOT_PLANE][0] = lower[nPlane];
		limits[CRagdollSolver::SLOT_PLANE][1] = upper[nPlane];
	}

	bool limited[CRagdollSolver::SLOT_COUNT];
	for ( int i = 0; i < CRagdollSolver::SLOT_COUNT; ++i )
		limited[i] = !IsFullCircle( limits[i][0], limits[i][1] ) && limits[i][1] - limits[i][0] < TWO_PI;

	CRagdollSolver::Limit_t &twist = m_Ragdoll.GetLimit( CRagdollSolver::SLOT_TWIST );
	CRagdollSolver::Limit_t &cone = m_Ragdoll.GetLimit( CRagdollSolver::SLOT_CONE );
	CRagdollSolver::Limit_t &plane = m_Ragdoll.GetLimit( CRagdollSolver::SLOT_PLANE );
	twist.m_flLower = limits[CRagdollSolver::SLOT_TWIST][0];
	twist.m_flUpper = limits[CRagdollSolver::SLOT_TWIST][1];
	twist.m_bLimited = limited[CRagdollSolver::SLOT_TWIST];
	const float flConeHalfRange = limits[CRagdollSolver::SLOT_CONE][1];
	cone.m_flLower = std::cos( flConeHalfRange );
	cone.m_flUpper = CONE_UPPER_BOUND;
	cone.m_bLimited = limited[CRagdollSolver::SLOT_CONE];
	plane.m_flLower = -std::sin( limits[CRagdollSolver::SLOT_PLANE][1] );
	plane.m_flUpper = -std::sin( limits[CRagdollSolver::SLOT_PLANE][0] );
	plane.m_bLimited = limited[CRagdollSolver::SLOT_PLANE];

	const float flReferenceMass = m_pReference ? m_pReference->GetMass() : 0.0f;
	float flMaxFriction = 0.0f;
	for ( int i = 0; i < CRagdollSolver::SLOT_COUNT; ++i )
	{
		const constraint_axislimit_t &axis = ragdoll.axes[IVP_TO_SOURCE_AXIS[m_RagdollFrames.m_SlotAxes[i]]];
		CRagdollSolver::Limit_t &limit = m_Ragdoll.GetLimit( static_cast<CRagdollSolver::Slot_t>( i ) );
		const float flFriction = flReferenceMass * axis.torque;
		limit.m_flMaxTorque = std::fabs( flFriction );
		limit.m_flTargetSpeed = axis.angularVelocity * DEGREES_TO_RADIANS;
		limit.m_bFriction = flFriction != 0.0f;
		flMaxFriction = std::max( flMaxFriction, limit.m_flMaxTorque );
	}

	if ( m_bOnlyAngularLimits )
	{
		m_Definition.m_Filter = b3DefaultFilterJointDef();
		m_nJointType = b3_filterJoint;
		return;
	}

	b3SphericalJointDef &def = m_Definition.m_Spherical;
	def = b3DefaultSphericalJointDef();
	def.base.localFrameA = GetRagdollFrame( 0 );
	def.base.localFrameB = GetRagdollFrame( 1 );
	m_nJointType = b3_sphericalJoint;
	m_Ragdoll.SetPlaneOnly( plane.m_bLimited );

	if ( twist.m_bLimited )
	{
		def.enableTwistLimit = true;
		def.lowerTwistAngle = -twist.m_flUpper;
		def.upperTwistAngle = -twist.m_flLower;
	}

	if ( cone.m_bLimited )
	{
		def.enableConeLimit = true;
		def.coneAngle = std::min( flConeHalfRange, 0.5f * PI );
	}

	if ( flMaxFriction != 0.0f )
	{
		def.enableMotor = true;
		def.maxMotorTorque = flMaxFriction * IVP_TO_SOURCE_SQUARED;
		def.motorVelocity = b3Vec3_zero;
	}
}

void CPhysicsConstraint::StoreRagdollRows( int nBody, const b3Vec3 columns[3] )
{
	b3Vec3 *rows = m_RagdollFrames.m_Rows[nBody];
	rows[RagdollFrames_t::ROW_TWIST] = columns[m_RagdollFrames.m_SlotAxes[CRagdollSolver::SLOT_TWIST]];
	rows[RagdollFrames_t::ROW_PLANE] = columns[m_RagdollFrames.m_SlotAxes[CRagdollSolver::SLOT_PLANE]];
	rows[RagdollFrames_t::ROW_CONE] = columns[m_RagdollFrames.m_SlotAxes[CRagdollSolver::SLOT_CONE]];
}

int CPhysicsConstraint::SelectTwistAxis( const b3Vec3 columns[2][3] ) const
{
	if ( !m_pReference )
		return 0;

	const CPhysicsObject *pObjects[2] = { m_pReference, m_pAttached };
	b3Vec3 levers[2];
	for ( int i = 0; i < 2; ++i )
	{
		const DoubleMatrix3_t &rotation = pObjects[i]->GetCoreRotation();
		const DoubleVec3_t position = pObjects[i]->GetCorePosition();
		const DoubleVec3_t anchor = Rotate( rotation, m_RagdollFrames.m_Origins[i] ) + ( Rotate( rotation, pObjects[i]->GetShift() ) + position );
		levers[i] = ToFloat( anchor - position );
	}

	const DoubleMatrix3_t &referenceRotation = m_pReference->GetCoreRotation();
	const DoubleMatrix3_t &attachedRotation = m_pAttached->GetCoreRotation();
	const b3Vec3 inverseInertia = m_pReference->GetCoreInverseInertia();
	int nBest = 0;
	float flBest = -1.0f;
	for ( int i = 0; i < 3; ++i )
	{
		const b3Vec3 a = ToFloat( Rotate( referenceRotation, columns[0][i] ) );
		const b3Vec3 b = ToFloat( Rotate( attachedRotation, columns[1][i] ) );
		float flAngular = 0.0f;
		if ( m_pReference->IsMoveable() )
		{
			for ( const b3Vec3 &row : { RotateRows( referenceRotation, a ), RotateRows( attachedRotation, b3MulSV( -1.0f, a ) ) } )
			{
				const b3Vec3 response = { row.x * inverseInertia.x, row.y * inverseInertia.y, row.z * inverseInertia.z };
				flAngular = ( 0.0f + row.y * response.y ) + ( ( row.z * response.z + flAngular ) + row.x * response.x );
			}
		}

		const float flMetric = ( ( GetLeverSquared( levers[0], a ) * m_pReference->GetInvMass() ) + flAngular ) + ( GetLeverSquared( levers[1], b ) * m_pAttached->GetInvMass() );
		if ( flMetric > flBest )
		{
			nBest = i;
			flBest = flMetric;
		}
	}
	return nBest;
}

void CPhysicsConstraint::UpdateRagdollSolverAxes()
{
	for ( int i = 0; i < 2; ++i )
		m_Ragdoll.SetAxes( i, m_RagdollFrames.m_Rows[i][RagdollFrames_t::ROW_TWIST], m_RagdollFrames.m_Rows[i][RagdollFrames_t::ROW_CONE] );
}

b3Transform CPhysicsConstraint::GetRagdollFrame( int nBody ) const
{
	const b3Vec3 *rows = m_RagdollFrames.m_Rows[nBody];
	return { IVPToSource( m_RagdollFrames.m_Origins[nBody] ), MakeFrameRotation( IVPToSourceAxis( rows[RagdollFrames_t::ROW_CONE] ), IVPToSourceAxis( rows[RagdollFrames_t::ROW_TWIST] ) ) };
}

void CPhysicsConstraint::PrepareBodies( ConstraintBody_t bodies[2], float scales[2] ) const
{
	CPhysicsObject *const objects[2] = { m_pReference, m_pAttached };
	for ( int i = 0; i < 2; ++i )
	{
		ConstraintBody_t &body = bodies[i];
		body.m_pObject = objects[i];
		body.m_bDynamic = objects[i]->IsMoveable();
		body.m_flInverseMass = objects[i]->GetInvMass();
		body.m_InverseInertia = objects[i]->GetCoreInverseInertia();
		scales[i] = 1.0f;
		if ( !m_bBreakable || !body.m_bDynamic || m_BodyMassScale[i] == 1.0f || !( m_BodyMassScale[i] > 0.0f ) )
			continue;

		float flScale = m_BodyMassScale[i];
		const float flMass = objects[i]->GetMass();
		if ( flMass * flScale > MAX_SCALED_MASS )
			flScale = MAX_SCALED_MASS / flMass;
		const float flInverseScale = 1.0f / flScale;
		scales[i] = flScale;
		body.m_flInverseMass *= flInverseScale;
		body.m_InverseInertia = b3MulSV( flInverseScale, body.m_InverseInertia );
	}
}

template<typename Function>
void CPhysicsConstraint::RunRows( Function function )
{
	if ( !m_pReference || m_pReference == m_pAttached )
		return;

	ConstraintBody_t bodies[2];
	float scales[2];
	PrepareBodies( bodies, scales );
	if ( !m_bBreakable || m_Ragdoll.IsPlaneOnly() )
	{
		function( bodies );
		return;
	}

	b3Vec3 speeds[2], rotationSpeeds[2];
	for ( int i = 0; i < 2; ++i )
	{
		speeds[i] = bodies[i].m_pObject->GetCoreVelocity();
		rotationSpeeds[i] = bodies[i].m_pObject->GetCoreAngularVelocity();
	}

	function( bodies );

	bool bBroken = false;
	for ( int i = 0; i < 2; ++i )
	{
		if ( !bodies[i].m_bDynamic )
			continue;

		const float flMass = bodies[i].m_pObject->GetMass() * scales[i];
		const b3Vec3 linear = b3Sub( bodies[i].m_pObject->GetCoreVelocity(), speeds[i] );
		const b3Vec3 angular = b3Sub( bodies[i].m_pObject->GetCoreAngularVelocity(), rotationSpeeds[i] );
		const Vector inertia = bodies[i].m_pObject->GetInertia();
		const b3Vec3 momentum = { angular.x * inertia.x * scales[i], angular.y * inertia.z * scales[i], angular.z * inertia.y * scales[i] };
		if ( m_flForceLimit * m_flForceLimit <= b3LengthSquared( linear ) * flMass || m_flTorqueLimit * m_flTorqueLimit <= b3LengthSquared( momentum ) )
			bBroken = true;
	}

	if ( bBroken && m_pGroup && m_pGroup->IsActive() )
	{
		if ( IPhysicsConstraintEvent *pHandler = m_pEnvironment->GetConstraintEventHandler() )
			pHandler->ConstraintBroken( this );
	}

	if ( m_pGroup && m_pGroup->IsActive() )
		return;

	for ( int i = 0; i < 2; ++i )
	{
		if ( !bodies[i].m_bDynamic )
			continue;

		bodies[i].m_pObject->SetCoreVelocity( speeds[i] );
		bodies[i].m_pObject->SetCoreAngularVelocity( rotationSpeeds[i] );
	}
}

void CPhysicsConstraint::SetupRows( const ConstraintStep_t &step )
{
	if ( m_nType == CONSTRAINT_PULLEY )
		RunRows( [&]( ConstraintBody_t bodies[2] ) { m_Pulley.Setup( bodies, step ); } );
	else
		RunRows( [&]( ConstraintBody_t bodies[2] ) { m_Ragdoll.Setup( bodies, step ); } );
}

void CPhysicsConstraint::SolveRows( float flAlpha, float flBeta, const ConstraintStep_t &step )
{
	if ( m_nType == CONSTRAINT_PULLEY )
		RunRows( [&]( ConstraintBody_t bodies[2] ) { m_Pulley.Solve( bodies, flAlpha, flBeta, step ); } );
	else
		RunRows( [&]( ConstraintBody_t bodies[2] ) { m_Ragdoll.Solve( bodies, flAlpha, flBeta, step ); } );
}

void CPhysicsConstraint::DetachObjects()
{
	if ( !m_pReference )
		return;

	m_pReference->DetachConstraint( this );
	if ( m_pAttached != m_pReference )
		m_pAttached->DetachConstraint( this );
	m_pReference = nullptr;
	m_pAttached = nullptr;
}

void CPhysicsConstraint::NotifyWokenObjects() const
{
	for ( CPhysicsObject *pObject : { m_pReference, m_pAttached } )
	{
		if ( pObject && !pObject->IsStatic() && b3Body_IsAwake( pObject->GetBody() ) )
			m_pEnvironment->NotifyWoken( pObject );
	}
}

void CPhysicsConstraint::CreateJoint()
{
	if ( HasJoint() || !m_pReference || m_pReference == m_pAttached )
		return;

	JointDef_t def = m_Definition;
	b3JointDef &base = GetBaseDefinition( def, m_nJointType );
	base.bodyIdA = m_pReference->GetBody();
	base.bodyIdB = m_pAttached->GetBody();
	base.userData = this;
	base.collideConnected = true;

	const b3WorldId world = m_pEnvironment->GetWorld();
	switch ( m_nJointType )
	{
	case b3_revoluteJoint:
		m_Joint = b3CreateRevoluteJoint( world, &def.m_Revolute );
		break;
	case b3_prismaticJoint:
		m_Joint = b3CreatePrismaticJoint( world, &def.m_Prismatic );
		break;
	case b3_weldJoint:
		m_Joint = b3CreateWeldJoint( world, &def.m_Weld );
		break;
	case b3_sphericalJoint:
		m_Joint = b3CreateSphericalJoint( world, &def.m_Spherical );
		break;
	case b3_distanceJoint:
		m_Joint = b3CreateDistanceJoint( world, &def.m_Distance );
		break;
	default:
		m_Joint = b3CreateFilterJoint( world, &def.m_Filter );
		break;
	}

	m_flBreakMass = -1.0f;
	UpdateBreakThresholds( m_pEnvironment->GetSimulationTimestep() );
	NotifyWokenObjects();
}

void CPhysicsConstraint::DestroyJoint( bool bWakeAttached )
{
	if ( !HasJoint() )
		return;

	b3DestroyJoint( m_Joint, bWakeAttached );
	m_Joint = b3_nullJointId;
	if ( bWakeAttached )
		NotifyWokenObjects();
}

float CPhysicsConstraint::GetBreakMass() const
{
	float flMass = 0.0f;
	for ( const CPhysicsObject *pObject : { m_pReference, m_pAttached } )
	{
		if ( pObject && pObject->IsMoveable() && ( flMass == 0.0f || pObject->GetMass() < flMass ) )
			flMass = pObject->GetMass();
	}
	return flMass;
}

void CPhysicsConstraint::UpdateBreakThresholds( float flTimestep )
{
	if ( !m_bBreakable || !HasJoint() || m_nJointType == b3_filterJoint )
		return;

	const float flMass = GetBreakMass();
	if ( flMass == m_flBreakMass && flTimestep == m_flBreakTimestep )
		return;

	m_flBreakMass = flMass;
	m_flBreakTimestep = flTimestep;
	const bool bMoveable = flMass > 0.0f;
	const float flForce = bMoveable && m_flForceLimit < UNBREAKABLE_LIMIT ? ( m_flForceLimit * IVP_TO_SOURCE ) * std::sqrt( flMass ) / flTimestep : FLT_MAX;
	const float flTorque = bMoveable && m_flTorqueLimit < UNBREAKABLE_LIMIT ? ( m_flTorqueLimit * IVP_TO_SOURCE_SQUARED ) / flTimestep : FLT_MAX;
	b3Joint_SetForceThreshold( m_Joint, flForce );
	b3Joint_SetTorqueThreshold( m_Joint, flTorque );
}

float CPhysicsConstraint::GetJointSeparation() const
{
	return HasJoint() && m_nJointType != b3_filterJoint ? b3Joint_GetLinearSeparation( m_Joint ) : 0.0f;
}

void CPhysicsConstraint::ObjectDestroyed( CPhysicsObject *pObject )
{
	const bool bNotify = pObject == m_pReference ? m_bListenerRegistered[0] : m_bListenerRegistered[1];
	DetachObjects();
	DestroyJoint( false );
	if ( m_pGroup )
	{
		m_pGroup->ObjectDestroyed( pObject );
		m_pGroup->RemoveConstraint( this );
	}
	m_pGroup = nullptr;
	delete m_pPrivateGroup;
	m_pPrivateGroup = nullptr;

	IPhysicsConstraintEvent *pHandler = m_pEnvironment->GetConstraintEventHandler();
	if ( bNotify && m_pEnvironment->IsConstraintNotifyEnabled() && pHandler )
		pHandler->ConstraintBroken( this );
}

void CPhysicsConstraint::Activate()
{
	if ( m_pPrivateGroup )
		m_pPrivateGroup->Activate();
}

void CPhysicsConstraint::Deactivate()
{
	if ( m_pPrivateGroup )
		m_pPrivateGroup->Deactivate();
}

IPhysicsObject *CPhysicsConstraint::GetReferenceObject() const
{
	return m_pReference;
}

IPhysicsObject *CPhysicsConstraint::GetAttachedObject() const
{
	return m_pAttached;
}

void CPhysicsConstraint::SetLinearMotor( float speed, float maxLinearImpulse )
{
	if ( m_nType != CONSTRAINT_SLIDING )
		return;

	b3PrismaticJointDef &def = m_Definition.m_Prismatic;
	def.enableMotor = maxLinearImpulse != 0.0f;
	def.motorSpeed = speed;
	def.maxMotorForce = std::fabs( maxLinearImpulse );
	if ( !HasJoint() )
		return;

	b3PrismaticJoint_EnableMotor( m_Joint, def.enableMotor );
	b3PrismaticJoint_SetMotorSpeed( m_Joint, def.motorSpeed );
	b3PrismaticJoint_SetMaxMotorForce( m_Joint, def.maxMotorForce );
}

void CPhysicsConstraint::SetAngularMotor( float rotSpeed, float maxAngularImpulse )
{
	const float flTorque = std::fabs( maxAngularImpulse ) * DEGREES_TO_RADIANS;
	if ( m_nType == CONSTRAINT_HINGE )
	{
		b3RevoluteJointDef &def = m_Definition.m_Revolute;
		def.enableMotor = maxAngularImpulse != 0.0f;
		def.motorSpeed = -( rotSpeed * DEGREES_TO_RADIANS );
		def.maxMotorTorque = flTorque * IVP_TO_SOURCE_SQUARED;
		if ( !HasJoint() )
			return;

		b3RevoluteJoint_EnableMotor( m_Joint, def.enableMotor );
		b3RevoluteJoint_SetMotorSpeed( m_Joint, def.motorSpeed );
		b3RevoluteJoint_SetMaxMotorTorque( m_Joint, def.maxMotorTorque );
		return;
	}

	if ( m_nType != CONSTRAINT_RAGDOLL || rotSpeed != 0.0f )
		return;

	for ( int i = 0; i < CRagdollSolver::SLOT_COUNT; ++i )
	{
		CRagdollSolver::Limit_t &limit = m_Ragdoll.GetLimit( static_cast<CRagdollSolver::Slot_t>( i ) );
		limit.m_flMaxTorque = flTorque;
		limit.m_flTargetSpeed = 0.0f;
		limit.m_bFriction = flTorque != 0.0f;
	}

	if ( m_nJointType != b3_sphericalJoint )
		return;

	b3SphericalJointDef &def = m_Definition.m_Spherical;
	def.enableMotor = flTorque != 0.0f;
	def.maxMotorTorque = flTorque * IVP_TO_SOURCE_SQUARED;
	def.motorVelocity = b3Vec3_zero;
	if ( !HasJoint() )
		return;

	b3SphericalJoint_EnableMotor( m_Joint, def.enableMotor );
	b3SphericalJoint_SetMaxMotorTorque( m_Joint, def.maxMotorTorque );
	b3SphericalJoint_SetMotorVelocity( m_Joint, def.motorVelocity );
}

void CPhysicsConstraint::UpdateRagdollTransforms( const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached )
{
	if ( m_nType != CONSTRAINT_RAGDOLL )
		return;

	b3Vec3 columns[2][3];
	ConvertRagdollAxes( constraintToReference, columns[0] );
	ConvertRagdollAxes( constraintToAttached, columns[1] );
	for ( int i = 0; i < 2; ++i )
		StoreRagdollRows( i, columns[i] );
	UpdateRagdollSolverAxes();
	if ( m_nJointType != b3_sphericalJoint )
		return;

	b3SphericalJointDef &def = m_Definition.m_Spherical;
	def.base.localFrameA = GetRagdollFrame( 0 );
	def.base.localFrameB = GetRagdollFrame( 1 );
	if ( !HasJoint() )
		return;

	b3Joint_SetLocalFrameA( m_Joint, def.base.localFrameA );
	b3Joint_SetLocalFrameB( m_Joint, def.base.localFrameB );
}

bool CPhysicsConstraint::GetConstraintTransform( matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached ) const
{
	switch ( m_nType )
	{
	case CONSTRAINT_FIXED:
		for ( matrix3x4_t *pOutput : { pConstraintToReference, pConstraintToAttached } )
		{
			if ( !pOutput )
				continue;
			*pOutput = m_AttachedRefXform;
			SetMatrixColumn( *pOutput, 3, RoundTripPosition( MatrixColumn( m_AttachedRefXform, 3 ) ) );
		}
		return true;
	case CONSTRAINT_BALLSOCKET:
		for ( int i = 0; i < 2; ++i )
		{
			matrix3x4_t *pOutput = i == 0 ? pConstraintToReference : pConstraintToAttached;
			if ( !pOutput )
				continue;
			SetIdentity( *pOutput );
			SetMatrixColumn( *pOutput, 3, RoundTripPosition( m_Anchors[i] ) );
		}
		return true;
	case CONSTRAINT_RAGDOLL:
		for ( int i = 0; i < 2; ++i )
		{
			matrix3x4_t *pOutput = i == 0 ? pConstraintToReference : pConstraintToAttached;
			if ( !pOutput )
				continue;
			const b3Vec3 *rows = m_RagdollFrames.m_Rows[i];
			SetMatrixColumn( *pOutput, 0, IVPToSourceAxis( rows[RagdollFrames_t::ROW_TWIST] ) );
			SetMatrixColumn( *pOutput, 1, IVPToSourceAxis( rows[RagdollFrames_t::ROW_CONE] ) );
			SetMatrixColumn( *pOutput, 2, b3Neg( IVPToSourceAxis( rows[RagdollFrames_t::ROW_PLANE] ) ) );
			SetMatrixColumn( *pOutput, 3, IVPToSource( m_RagdollFrames.m_Origins[i] ) );
		}
		return true;
	default:
		return false;
	}
}

bool CPhysicsConstraint::GetConstraintParams( constraint_breakableparams_t *pParams ) const
{
	if ( !pParams || !m_pReference || !m_pAttached )
		return false;

	pParams->strength = 1.0f;
	pParams->forceLimit = m_bBreakable ? m_flForceLimit * IVP_TO_SOURCE : 0.0f;
	pParams->torqueLimit = m_bBreakable ? m_flTorqueLimit * RADIANS_TO_DEGREES : 0.0f;
	pParams->bodyMassScale[0] = m_bBreakable ? m_BodyMassScale[0] : 1.0f;
	pParams->bodyMassScale[1] = m_bBreakable ? m_BodyMassScale[1] : 1.0f;
	pParams->isActive = m_pPrivateGroup ? m_pPrivateGroup->IsActive() : !m_bBreakable;
	return true;
}

void CPhysicsConstraint::OutputDebugInfo()
{
	if ( m_pGroup )
		m_pGroup->OutputDebugInfo();
}

constraint_breakableparams_t CPhysicsConstraint::GetSavedBreakableParams() const
{
	constraint_breakableparams_t params;
	GetConstraintParams( &params );
	if ( !m_pPrivateGroup )
		params.isActive = !( m_nType == CONSTRAINT_HINGE || m_nType == CONSTRAINT_FIXED || m_nType == CONSTRAINT_RAGDOLL );
	return params;
}

static float RoundTripLength( float flLength )
{
	return ( flLength * SOURCE_TO_IVP ) * IVP_TO_SOURCE;
}

static matrix3x4_t RoundTripTransform( const matrix3x4_t &matrix )
{
	matrix3x4_t result = matrix;
	SetMatrixColumn( result, 3, RoundTripPosition( MatrixColumn( matrix, 3 ) ) );
	return result;
}

static matrix3x4_t RoundTripRotation( const matrix3x4_t &matrix )
{
	DoubleMatrix3_t rotation;
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
			rotation.m[i][j] = matrix[i][j];
	}
	rotation = QuatToMatrix( MatrixToQuat( rotation ) );

	matrix3x4_t result = RoundTripTransform( matrix );
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
			result[i][j] = static_cast<float>( rotation.m[i][j] );
	}
	return result;
}

void CPhysicsConstraint::Save( ISave *pSave ) const
{
	vphysics_save_cphysicsconstraint_t header = {};
	header.constraintType = m_nType;
	if ( m_pReference )
	{
		header.pGroup = m_pPrivateGroup ? nullptr : m_pGroup;
		header.pObjReference = m_pReference;
		header.pObjAttached = m_pAttached;
	}
	pSave->WriteAll( &header );
	if ( m_nType == CONSTRAINT_NONE || !m_pReference || !m_pAttached )
		return;

	const constraint_breakableparams_t breakable = GetSavedBreakableParams();
	switch ( m_nType )
	{
	case CONSTRAINT_HINGE:
	{
		const b3RevoluteJointDef &def = m_Definition.m_Revolute;
		vphysics_save_constrainthinge_t record = {};
		m_pReference->LocalToWorld( &record.worldPosition, ToSource( def.base.localFrameA.p ) );
		m_pReference->LocalToWorldVector( &record.worldAxisDirection, ToSource( b3RotateVector( def.base.localFrameA.q, b3Vec3_axisZ ) ) );
		if ( def.enableLimit )
		{
			record.hingeAxis.minRotation = -def.upperAngle * RADIANS_TO_DEGREES;
			record.hingeAxis.maxRotation = -def.lowerAngle * RADIANS_TO_DEGREES;
		}
		if ( def.enableMotor )
		{
			record.hingeAxis.angularVelocity = -def.motorSpeed * RADIANS_TO_DEGREES;
			record.hingeAxis.torque = ( def.maxMotorTorque / IVP_TO_SOURCE_SQUARED ) * RADIANS_TO_DEGREES;
		}
		record.constraint = breakable;
		pSave->WriteAll( &record );
		break;
	}
	case CONSTRAINT_FIXED:
	{
		vphysics_save_constraintfixed_t record = {};
		record.attachedRefXform = RoundTripTransform( m_AttachedRefXform );
		record.constraint = breakable;
		pSave->WriteAll( &record );
		break;
	}
	case CONSTRAINT_BALLSOCKET:
	{
		vphysics_save_constraintballsocket_t record = {};
		for ( int i = 0; i < 2; ++i )
			record.constraintPosition[i] = ToSource( RoundTripPosition( m_Anchors[i] ) );
		record.constraint = breakable;
		pSave->WriteAll( &record );
		break;
	}
	case CONSTRAINT_SLIDING:
	{
		const b3PrismaticJointDef &def = m_Definition.m_Prismatic;
		vphysics_save_constraintsliding_t record = {};
		record.attachedRefXform = RoundTripRotation( m_AttachedRefXform );
		record.slideAxisRef = ToSource( m_SlideAxis );
		if ( def.enableLimit )
		{
			record.limitMin = RoundTripLength( def.lowerTranslation );
			record.limitMax = RoundTripLength( def.upperTranslation );
		}
		if ( def.enableMotor )
		{
			record.friction = def.maxMotorForce;
			record.velocity = def.motorSpeed;
		}
		record.constraint = breakable;
		pSave->WriteAll( &record );
		break;
	}
	case CONSTRAINT_PULLEY:
	{
		vphysics_save_constraintpulley_t record = {};
		for ( int i = 0; i < 2; ++i )
		{
			record.pulleyPosition[i] = ToSource( IVPToSource( m_Pulley.GetPulley( i ) ) );
			record.objectPosition[i] = ToSource( IVPToSource( m_Pulley.GetAnchor( i ) ) );
		}
		record.totalLength = m_Pulley.GetLength() * IVP_TO_SOURCE;
		record.gearRatio = m_Pulley.GetGearRatio();
		record.isRigid = m_Pulley.IsRigid();
		record.constraint = breakable;
		pSave->WriteAll( &record );
		break;
	}
	case CONSTRAINT_LENGTH:
	{
		const b3DistanceJointDef &def = m_Definition.m_Distance;
		vphysics_save_constraintlength_t record = {};
		record.objectPosition[0] = ToSource( RoundTripPosition( def.base.localFrameA.p ) );
		record.objectPosition[1] = ToSource( RoundTripPosition( def.base.localFrameB.p ) );
		record.totalLength = RoundTripLength( def.length );
		record.minLength = RoundTripLength( def.enableSpring ? def.minLength : def.length );
		record.constraint = breakable;
		pSave->WriteAll( &record );
		break;
	}
	case CONSTRAINT_RAGDOLL:
	{
		vphysics_save_constraintragdoll_t record = {};
		record.constraint = breakable;
		const int *slotAxes = m_RagdollFrames.m_SlotAxes;
		matrix3x4_t *const matrices[2] = { &record.constraintToReference, &record.constraintToAttached };
		for ( int i = 0; i < 2; ++i )
		{
			const b3Vec3 *rows = m_RagdollFrames.m_Rows[i];
			b3Vec3 columns[3];
			columns[slotAxes[CRagdollSolver::SLOT_TWIST]] = rows[RagdollFrames_t::ROW_TWIST];
			columns[slotAxes[CRagdollSolver::SLOT_PLANE]] = rows[RagdollFrames_t::ROW_PLANE];
			columns[slotAxes[CRagdollSolver::SLOT_CONE]] = rows[RagdollFrames_t::ROW_CONE];
			SetMatrixColumn( *matrices[i], 0, IVPToSourceAxis( columns[0] ) );
			SetMatrixColumn( *matrices[i], 1, IVPToSourceAxis( columns[2] ) );
			SetMatrixColumn( *matrices[i], 2, b3Neg( IVPToSourceAxis( columns[1] ) ) );
			SetMatrixColumn( *matrices[i], 3, IVPToSource( m_RagdollFrames.m_Origins[i] ) );
		}

		const float flInverseMass = m_pReference->GetInvMass();
		for ( int i = 0; i < 3; ++i )
		{
			int nSlot = 0;
			while ( slotAxes[nSlot] != IVP_TO_SOURCE_AXIS[i] )
				++nSlot;

			const CRagdollSolver::Limit_t &limit = m_Ragdoll.GetLimit( static_cast<CRagdollSolver::Slot_t>( nSlot ) );
			float flLower = m_RagdollLimits[nSlot][0], flUpper = m_RagdollLimits[nSlot][1];
			if ( nSlot == CRagdollSolver::SLOT_TWIST )
			{
				flLower = -m_RagdollLimits[nSlot][1];
				flUpper = -m_RagdollLimits[nSlot][0];
			}

			constraint_axislimit_t &axis = record.axes[i];
			axis.minRotation = i == 2 ? flUpper * -RADIANS_TO_DEGREES : flLower * RADIANS_TO_DEGREES;
			axis.maxRotation = i == 2 ? flLower * -RADIANS_TO_DEGREES : flUpper * RADIANS_TO_DEGREES;
			axis.angularVelocity = limit.m_flTargetSpeed * RADIANS_TO_DEGREES;
			axis.torque = flInverseMass * limit.m_flMaxTorque;
		}
		record.onlyAngularLimits = m_bOnlyAngularLimits;
		record.isActive = m_pPrivateGroup ? m_pPrivateGroup->IsActive() : true;
		record.useClockwiseRotations = false;
		pSave->WriteAll( &record );
		break;
	}
	default:
		break;
	}
}

const char *CPhysicsConstraint::GetSolverName() const
{
	switch ( m_nType )
	{
	case CONSTRAINT_RAGDOLL:
		return "ragdoll";
	case CONSTRAINT_HINGE:
		return "hinge";
	case CONSTRAINT_FIXED:
		return "fixed";
	case CONSTRAINT_BALLSOCKET:
		return "ball_socket";
	case CONSTRAINT_SLIDING:
		return "prismatic";
	case CONSTRAINT_PULLEY:
		return "pulley";
	default:
		return "stiff_spring";
	}
}
