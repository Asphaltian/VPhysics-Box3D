#include "vphysics_saverestore.h"

#include "physics_constraint.h"
#include "physics_environment.h"
#include "physics_motion_controller.h"
#include "physics_object.h"
#include "physics_player_controller.h"
#include "physics_shadow_controller.h"
#include "physics_spring.h"
#include "physics_surface_props.h"

#include "isaverestore.h"
#include "tier0/dbg.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <unordered_map>

static constexpr int MAX_MATERIAL_NAME = 1024;
static constexpr unsigned int OBJECT_RECORD_SIZE = 0xa0;

static_assert( sizeof( vphysics_save_cphysicsobject_t ) == OBJECT_RECORD_SIZE );

static std::unordered_map<uint32_t, void *> s_PointerMap;

static uint32_t GetSaveId( const void *pPointer )
{
	return static_cast<uint32_t>( reinterpret_cast<uintptr_t>( pPointer ) );
}

static void *FindRestoredPointer( uint32_t nId )
{
	auto it = s_PointerMap.find( nId );
	return it != s_PointerMap.end() ? it->second : nullptr;
}

class CPhysicsPointerOps final : public CDefSaveRestoreOps
{
public:
	void Save( const SaveRestoreFieldInfo_t &fieldInfo, ISave *pSave ) override
	{
		void *const *ppPointers = static_cast<void *const *>( fieldInfo.pField );
		for ( int i = 0; i < fieldInfo.pTypeDesc->fieldSize; ++i )
		{
			const uint32_t nId = GetSaveId( ppPointers[i] );
			pSave->WriteInt( &nId );
		}
	}

	void Restore( const SaveRestoreFieldInfo_t &fieldInfo, IRestore *pRestore ) override
	{
		void **ppPointers = static_cast<void **>( fieldInfo.pField );
		for ( int i = 0; i < fieldInfo.pTypeDesc->fieldSize; ++i )
		{
			uint32_t nId = 0;
			pRestore->ReadInt( &nId );
			ppPointers[i] = FindRestoredPointer( nId );
		}
	}
};

class CPhysicsMaterialOps final : public CDefSaveRestoreOps
{
public:
	void Save( const SaveRestoreFieldInfo_t &fieldInfo, ISave *pSave ) override
	{
		const char *pName = g_PhysicsSurfaceProps.GetPropName( *static_cast<const int *>( fieldInfo.pField ) );
		if ( !pName )
			pName = g_PhysicsSurfaceProps.GetPropName( 0 );

		const int nLength = static_cast<int>( std::strlen( pName ) ) + 1;
		pSave->WriteInt( &nLength );
		pSave->WriteString( pName );
	}

	void Restore( const SaveRestoreFieldInfo_t &fieldInfo, IRestore *pRestore ) override
	{
		const int nLength = pRestore->ReadInt();
		char name[MAX_MATERIAL_NAME];
		pRestore->ReadString( name, sizeof( name ), nLength );
		const int nIndex = g_PhysicsSurfaceProps.GetSurfaceIndex( name );
		*static_cast<int *>( fieldInfo.pField ) = nIndex < 0 ? 0 : nIndex;
	}

	bool IsEmpty( const SaveRestoreFieldInfo_t &fieldInfo ) override { return *static_cast<const int *>( fieldInfo.pField ) == 0; }
};

class CPhysicsObjectListOps final : public CDefSaveRestoreOps
{
public:
	void Save( const SaveRestoreFieldInfo_t &fieldInfo, ISave *pSave ) override
	{
		const CUtlVector<IPhysicsObject *> &list = *static_cast<const CUtlVector<IPhysicsObject *> *>( fieldInfo.pField );
		const int nCount = list.Count();
		pSave->WriteInt( &nCount );
		for ( int i = 0; i < nCount; ++i )
		{
			const uint32_t nId = GetSaveId( list[i] );
			pSave->WriteInt( &nId );
		}
	}

	void Restore( const SaveRestoreFieldInfo_t &fieldInfo, IRestore *pRestore ) override
	{
		CUtlVector<IPhysicsObject *> &list = *static_cast<CUtlVector<IPhysicsObject *> *>( fieldInfo.pField );
		const int nCount = pRestore->ReadInt();
		for ( int i = 0; i < nCount; ++i )
		{
			uint32_t nId = 0;
			pRestore->ReadInt( &nId );
			list.AddToTail( static_cast<IPhysicsObject *>( FindRestoredPointer( nId ) ) );
		}
	}
};

static CPhysicsPointerOps s_PointerOps;
static CPhysicsMaterialOps s_MaterialOps;
static CPhysicsObjectListOps s_ObjectListOps;

BEGIN_SIMPLE_DATADESC( vphysics_save_cphysicsobject_t )
	DEFINE_FIELD( sphereRadius, FIELD_FLOAT ),
	DEFINE_FIELD( isStatic, FIELD_BOOLEAN ),
	DEFINE_FIELD( collisionEnabled, FIELD_BOOLEAN ),
	DEFINE_FIELD( gravityEnabled, FIELD_BOOLEAN ),
	DEFINE_FIELD( dragEnabled, FIELD_BOOLEAN ),
	DEFINE_FIELD( motionEnabled, FIELD_BOOLEAN ),
	DEFINE_FIELD( isAsleep, FIELD_BOOLEAN ),
	DEFINE_FIELD( isTrigger, FIELD_BOOLEAN ),
	DEFINE_FIELD( asleepSinceCreation, FIELD_BOOLEAN ),
	DEFINE_FIELD( hasTouchedDynamic, FIELD_BOOLEAN ),
	DEFINE_CUSTOM_FIELD( materialIndex, &s_MaterialOps ),
	DEFINE_FIELD( mass, FIELD_FLOAT ),
	DEFINE_FIELD( rotInertia, FIELD_VECTOR ),
	DEFINE_FIELD( speedDamping, FIELD_FLOAT ),
	DEFINE_FIELD( rotSpeedDamping, FIELD_FLOAT ),
	DEFINE_FIELD( massCenterOverride, FIELD_VECTOR ),
	DEFINE_FIELD( callbacks, FIELD_INTEGER ),
	DEFINE_FIELD( gameFlags, FIELD_INTEGER ),
	DEFINE_FIELD( contentsMask, FIELD_INTEGER ),
	DEFINE_FIELD( volume, FIELD_FLOAT ),
	DEFINE_FIELD( dragCoefficient, FIELD_FLOAT ),
	DEFINE_FIELD( angDragCoefficient, FIELD_FLOAT ),
	DEFINE_FIELD( hasShadowController, FIELD_BOOLEAN ),
	DEFINE_FIELD( origin, FIELD_POSITION_VECTOR ),
	DEFINE_FIELD( angles, FIELD_VECTOR ),
	DEFINE_FIELD( velocity, FIELD_VECTOR ),
	DEFINE_FIELD( angVelocity, FIELD_VECTOR ),
	DEFINE_FIELD( collideType, FIELD_SHORT ),
	DEFINE_FIELD( gameIndex, FIELD_SHORT ),
	DEFINE_FIELD( hingeAxis, FIELD_INTEGER ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_shadowcontrolparams_t )
	DEFINE_FIELD( targetPosition, FIELD_POSITION_VECTOR ),
	DEFINE_FIELD( targetRotation, FIELD_VECTOR ),
	DEFINE_FIELD( maxSpeed, FIELD_FLOAT ),
	DEFINE_FIELD( maxDampSpeed, FIELD_FLOAT ),
	DEFINE_FIELD( maxAngular, FIELD_FLOAT ),
	DEFINE_FIELD( maxDampAngular, FIELD_FLOAT ),
	DEFINE_FIELD( dampFactor, FIELD_FLOAT ),
	DEFINE_FIELD( teleportDistance, FIELD_FLOAT ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_cshadowcontroller_t )
	DEFINE_FIELD( secondsToArrival, FIELD_FLOAT ),
	DEFINE_ARRAY( saveRot.k, FIELD_FLOAT, 3 ),
	DEFINE_ARRAY( savedRI.k, FIELD_FLOAT, 3 ),
	DEFINE_ARRAY( currentSpeed.k, FIELD_FLOAT, 3 ),
	DEFINE_FIELD( savedMass, FIELD_FLOAT ),
	DEFINE_FIELD( savedFlags, FIELD_INTEGER ),
	DEFINE_CUSTOM_FIELD( savedMaterial, &s_MaterialOps ),
	DEFINE_FIELD( enable, FIELD_BOOLEAN ),
	DEFINE_FIELD( allowPhysicsMovement, FIELD_BOOLEAN ),
	DEFINE_FIELD( allowPhysicsRotation, FIELD_BOOLEAN ),
	DEFINE_FIELD( isPhysicallyControlled, FIELD_BOOLEAN ),
	DEFINE_EMBEDDED( shadowParams ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_cphysicsspring_t )
	DEFINE_FIELD( constant, FIELD_FLOAT ),
	DEFINE_FIELD( naturalLength, FIELD_FLOAT ),
	DEFINE_FIELD( damping, FIELD_FLOAT ),
	DEFINE_FIELD( relativeDamping, FIELD_FLOAT ),
	DEFINE_FIELD( startPosition, FIELD_VECTOR ),
	DEFINE_FIELD( endPosition, FIELD_VECTOR ),
	DEFINE_FIELD( useLocalPositions, FIELD_BOOLEAN ),
	DEFINE_FIELD( onlyStretch, FIELD_BOOLEAN ),
	DEFINE_CUSTOM_FIELD( pObjStart, &s_PointerOps ),
	DEFINE_CUSTOM_FIELD( pObjEnd, &s_PointerOps ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_cphysicsconstraintgroup_t )
	DEFINE_FIELD( isActive, FIELD_BOOLEAN ),
	DEFINE_FIELD( additionalIterations, FIELD_INTEGER ),
	DEFINE_FIELD( minErrorTicks, FIELD_INTEGER ),
	DEFINE_FIELD( errorTolerance, FIELD_FLOAT ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_motioncontroller_t )
	DEFINE_CUSTOM_FIELD( m_objectList, &s_ObjectListOps ),
	DEFINE_FIELD( m_nPriority, FIELD_INTEGER ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_cphysicsconstraint_t )
	DEFINE_FIELD( constraintType, FIELD_INTEGER ),
	DEFINE_CUSTOM_FIELD( pGroup, &s_PointerOps ),
	DEFINE_CUSTOM_FIELD( pObjReference, &s_PointerOps ),
	DEFINE_CUSTOM_FIELD( pObjAttached, &s_PointerOps ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_constraintbreakable_t )
	DEFINE_FIELD( strength, FIELD_FLOAT ),
	DEFINE_FIELD( forceLimit, FIELD_FLOAT ),
	DEFINE_FIELD( torqueLimit, FIELD_FLOAT ),
	DEFINE_AUTO_ARRAY( bodyMassScale, FIELD_FLOAT ),
	DEFINE_FIELD( isActive, FIELD_BOOLEAN ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_constraintaxislimit_t )
	DEFINE_FIELD( minRotation, FIELD_FLOAT ),
	DEFINE_FIELD( maxRotation, FIELD_FLOAT ),
	DEFINE_FIELD( angularVelocity, FIELD_FLOAT ),
	DEFINE_FIELD( torque, FIELD_FLOAT ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_constrainthinge_t )
	DEFINE_FIELD( worldPosition, FIELD_POSITION_VECTOR ),
	DEFINE_FIELD( worldAxisDirection, FIELD_VECTOR ),
	DEFINE_EMBEDDED_OVERRIDE( constraint, vphysics_save_constraintbreakable_t ),
	DEFINE_EMBEDDED_OVERRIDE( hingeAxis, vphysics_save_constraintaxislimit_t ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_constraintfixed_t )
	DEFINE_ARRAY( attachedRefXform, FIELD_FLOAT, 12 ),
	DEFINE_EMBEDDED_OVERRIDE( constraint, vphysics_save_constraintbreakable_t ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_constraintballsocket_t )
	DEFINE_AUTO_ARRAY( constraintPosition, FIELD_VECTOR ),
	DEFINE_EMBEDDED_OVERRIDE( constraint, vphysics_save_constraintbreakable_t ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_constraintsliding_t )
	DEFINE_ARRAY( attachedRefXform, FIELD_FLOAT, 12 ),
	DEFINE_FIELD( slideAxisRef, FIELD_VECTOR ),
	DEFINE_FIELD( limitMin, FIELD_FLOAT ),
	DEFINE_FIELD( limitMax, FIELD_FLOAT ),
	DEFINE_FIELD( friction, FIELD_FLOAT ),
	DEFINE_FIELD( velocity, FIELD_FLOAT ),
	DEFINE_EMBEDDED_OVERRIDE( constraint, vphysics_save_constraintbreakable_t ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_constraintpulley_t )
	DEFINE_AUTO_ARRAY( pulleyPosition, FIELD_POSITION_VECTOR ),
	DEFINE_AUTO_ARRAY( objectPosition, FIELD_VECTOR ),
	DEFINE_FIELD( totalLength, FIELD_FLOAT ),
	DEFINE_FIELD( gearRatio, FIELD_FLOAT ),
	DEFINE_FIELD( isRigid, FIELD_BOOLEAN ),
	DEFINE_EMBEDDED_OVERRIDE( constraint, vphysics_save_constraintbreakable_t ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_constraintlength_t )
	DEFINE_AUTO_ARRAY( objectPosition, FIELD_VECTOR ),
	DEFINE_FIELD( totalLength, FIELD_FLOAT ),
	DEFINE_FIELD( minLength, FIELD_FLOAT ),
	DEFINE_EMBEDDED_OVERRIDE( constraint, vphysics_save_constraintbreakable_t ),
	END_DATADESC()

BEGIN_SIMPLE_DATADESC( vphysics_save_constraintragdoll_t )
	DEFINE_EMBEDDED_OVERRIDE( constraint, vphysics_save_constraintbreakable_t ),
	DEFINE_ARRAY( constraintToReference, FIELD_FLOAT, 12 ),
	DEFINE_ARRAY( constraintToAttached, FIELD_FLOAT, 12 ),
	DEFINE_EMBEDDED_OVERRIDE( axes[0], vphysics_save_constraintaxislimit_t ),
	DEFINE_EMBEDDED_OVERRIDE( axes[1], vphysics_save_constraintaxislimit_t ),
	DEFINE_EMBEDDED_OVERRIDE( axes[2], vphysics_save_constraintaxislimit_t ),
	DEFINE_FIELD( onlyAngularLimits, FIELD_BOOLEAN ),
	DEFINE_FIELD( isActive, FIELD_BOOLEAN ),
	DEFINE_FIELD( useClockwiseRotations, FIELD_BOOLEAN ),
	END_DATADESC()

template<typename Record>
static Record ReadRecord( IRestore *pRestore )
{
	Record record = {};
	pRestore->ReadAll( &record );
	return record;
}

void CPhysicsEnvironment::RestoreConstraint( const physrestoreparams_t &params )
{
	vphysics_save_cphysicsconstraint_t header = {};
	params.pRestore->ReadAll( &header );
	if ( header.constraintType == CPhysicsConstraint::CONSTRAINT_NONE || !header.pObjReference || !header.pObjAttached )
	{
		*params.ppObject = static_cast<IPhysicsConstraint *>( CPhysicsConstraint::CreateEmpty( this ) );
		return;
	}

	IPhysicsConstraint *pConstraint = static_cast<IPhysicsConstraint *>( *params.ppObject );
	switch ( header.constraintType )
	{
	case CPhysicsConstraint::CONSTRAINT_RAGDOLL:
		pConstraint = params.pEnvironment->CreateRagdollConstraint( header.pObjReference, header.pObjAttached, header.pGroup, ReadRecord<vphysics_save_constraintragdoll_t>( params.pRestore ) );
		break;
	case CPhysicsConstraint::CONSTRAINT_HINGE:
		pConstraint = params.pEnvironment->CreateHingeConstraint( header.pObjReference, header.pObjAttached, header.pGroup, ReadRecord<vphysics_save_constrainthinge_t>( params.pRestore ) );
		break;
	case CPhysicsConstraint::CONSTRAINT_FIXED:
		pConstraint = params.pEnvironment->CreateFixedConstraint( header.pObjReference, header.pObjAttached, header.pGroup, ReadRecord<vphysics_save_constraintfixed_t>( params.pRestore ) );
		break;
	case CPhysicsConstraint::CONSTRAINT_BALLSOCKET:
		pConstraint = params.pEnvironment->CreateBallsocketConstraint( header.pObjReference, header.pObjAttached, header.pGroup, ReadRecord<vphysics_save_constraintballsocket_t>( params.pRestore ) );
		break;
	case CPhysicsConstraint::CONSTRAINT_SLIDING:
		pConstraint = params.pEnvironment->CreateSlidingConstraint( header.pObjReference, header.pObjAttached, header.pGroup, ReadRecord<vphysics_save_constraintsliding_t>( params.pRestore ) );
		break;
	case CPhysicsConstraint::CONSTRAINT_PULLEY:
		pConstraint = params.pEnvironment->CreatePulleyConstraint( header.pObjReference, header.pObjAttached, header.pGroup, ReadRecord<vphysics_save_constraintpulley_t>( params.pRestore ) );
		break;
	case CPhysicsConstraint::CONSTRAINT_LENGTH:
		pConstraint = params.pEnvironment->CreateLengthConstraint( header.pObjReference, header.pObjAttached, header.pGroup, ReadRecord<vphysics_save_constraintlength_t>( params.pRestore ) );
		break;
	default:
		break;
	}

	*params.ppObject = pConstraint;
	if ( pConstraint )
		pConstraint->SetGameData( params.pGameData );
}

bool CPhysicsEnvironment::Save( const physsaveparams_t &params )
{
	if ( params.type >= PIID_NUM_TYPES )
		return false;

	const uint32_t nId = GetSaveId( params.pObject );
	params.pSave->WriteInt( &nId );

	switch ( params.type )
	{
	case PIID_IPHYSICSOBJECT:
	{
		const CPhysicsObject *pObject = static_cast<const CPhysicsObject *>( static_cast<IPhysicsObject *>( params.pObject ) );
		vphysics_save_cphysicsobject_t record = {};
		pObject->WriteRecord( record );
		params.pSave->WriteAll( &record );
		if ( record.hasShadowController )
		{
			vphysics_save_cshadowcontroller_t shadow = {};
			static_cast<const CPhysicsShadowController *>( pObject->GetShadowController() )->WriteRecord( shadow );
			params.pSave->WriteAll( &shadow );
		}
		return true;
	}
	case PIID_IPHYSICSSPRING:
	{
		vphysics_save_cphysicsspring_t record = {};
		static_cast<const CPhysicsSpring *>( static_cast<IPhysicsSpring *>( params.pObject ) )->WriteRecord( record );
		params.pSave->WriteAll( &record );
		return true;
	}
	case PIID_IPHYSICSCONSTRAINTGROUP:
	{
		vphysics_save_cphysicsconstraintgroup_t record = {};
		static_cast<const CPhysicsConstraintGroup *>( static_cast<IPhysicsConstraintGroup *>( params.pObject ) )->WriteRecord( record );
		params.pSave->WriteAll( &record );
		return true;
	}
	case PIID_IPHYSICSCONSTRAINT:
		static_cast<const CPhysicsConstraint *>( static_cast<IPhysicsConstraint *>( params.pObject ) )->Save( params.pSave );
		return true;
	case PIID_IPHYSICSSHADOWCONTROLLER:
	{
		vphysics_save_cshadowcontroller_t record = {};
		static_cast<const CPhysicsShadowController *>( static_cast<IPhysicsShadowController *>( params.pObject ) )->WriteRecord( record );
		params.pSave->WriteAll( &record );
		return true;
	}
	case PIID_IPHYSICSMOTIONCONTROLLER:
	{
		vphysics_save_motioncontroller_t record;
		static_cast<const CPhysicsMotionController *>( static_cast<IPhysicsMotionController *>( params.pObject ) )->WriteRecord( record );
		params.pSave->WriteAll( &record );
		return true;
	}
	default:
		return false;
	}
}

void CPhysicsEnvironment::PreRestore( const physprerestoreparams_t &params )
{
	for ( int i = 0; i < params.recreatedObjectCount; ++i )
		s_PointerMap[GetSaveId( params.recreatedObjectList[i].pOldObject )] = params.recreatedObjectList[i].pNewObject;
}

void CPhysicsEnvironment::ForgetRestoredObject( CPhysicsObject *pObject )
{
	m_RestoredCollisions.erase( std::remove( m_RestoredCollisions.begin(), m_RestoredCollisions.end(), pObject ), m_RestoredCollisions.end() );
	std::erase_if( s_PointerMap, [&]( const auto &entry ) { return entry.second == static_cast<IPhysicsObject *>( pObject ); } );
}

void CPhysicsEnvironment::RestoreObject( const physrestoreparams_t &params )
{
	vphysics_save_cphysicsobject_t record = {};
	params.pRestore->ReadAll( &record );
	record.pCollide = params.pCollisionModel;
	record.pName = params.pName;

	const bool bEnableCollisions = record.collisionEnabled;
	record.collisionEnabled = false;
	CPhysicsObject *pObject = CPhysicsObject::CreateFromRecord( static_cast<CPhysicsEnvironment *>( params.pEnvironment ), record, params.pGameData );
	*params.ppObject = pObject;
	if ( !pObject )
		return;

	if ( bEnableCollisions )
		m_RestoredCollisions.push_back( pObject );

	if ( record.hasShadowController )
	{
		vphysics_save_cshadowcontroller_t shadow = {};
		params.pRestore->ReadAll( &shadow );
		pObject->SetShadowController( new CPhysicsShadowController( pObject, shadow ) );
	}
}

bool CPhysicsEnvironment::Restore( const physrestoreparams_t &params )
{
	if ( params.type >= PIID_NUM_TYPES )
		return false;

	uint32_t nId = 0;
	params.pRestore->ReadInt( &nId );

	switch ( params.type )
	{
	case PIID_IPHYSICSOBJECT:
		RestoreObject( params );
		break;
	case PIID_IPHYSICSSPRING:
	{
		vphysics_save_cphysicsspring_t record = {};
		params.pRestore->ReadAll( &record );
		if ( !record.pObjStart || !record.pObjEnd )
		{
			DevMsg( "Failed to restore spring enpoints\n" );
			*params.ppObject = nullptr;
			break;
		}

		springparams_t spring;
		spring.constant = record.constant;
		spring.naturalLength = record.naturalLength;
		spring.damping = record.damping;
		spring.relativeDamping = record.relativeDamping;
		spring.startPosition = record.startPosition;
		spring.endPosition = record.endPosition;
		spring.useLocalPositions = record.useLocalPositions;
		spring.onlyStretch = record.onlyStretch;
		*params.ppObject = params.pEnvironment->CreateSpring( record.pObjStart, record.pObjEnd, &spring );
		break;
	}
	case PIID_IPHYSICSCONSTRAINTGROUP:
	{
		vphysics_save_cphysicsconstraintgroup_t record = {};
		params.pRestore->ReadAll( &record );
		constraint_groupparams_t group;
		group.Defaults();
		group.additionalIterations = record.additionalIterations;
		if ( record.errorTolerance != 0.0f || record.minErrorTicks != 0 )
		{
			group.minErrorTicks = record.minErrorTicks;
			group.errorTolerance = record.errorTolerance;
		}
		IPhysicsConstraintGroup *pGroup = params.pEnvironment->CreateConstraintGroup( group );
		*params.ppObject = pGroup;
		if ( pGroup && record.isActive )
			m_RestoredGroups.push_back( static_cast<CPhysicsConstraintGroup *>( pGroup ) );
		break;
	}
	case PIID_IPHYSICSCONSTRAINT:
		RestoreConstraint( params );
		break;
	case PIID_IPHYSICSMOTIONCONTROLLER:
	{
		CPhysicsMotionController *pController = new CPhysicsMotionController( nullptr );
		*params.ppObject = static_cast<IPhysicsMotionController *>( pController );
		vphysics_save_motioncontroller_t record;
		params.pRestore->ReadAll( &record );
		pController->ReadRecord( record );
		break;
	}
	default:
		return false;
	}

	s_PointerMap[nId] = *params.ppObject;
	if ( params.type == PIID_IPHYSICSOBJECT && *params.ppObject )
		m_Objects.push_back( static_cast<IPhysicsObject *>( *params.ppObject ) );
	return true;
}

void CPhysicsEnvironment::PostRestore()
{
	s_PointerMap.clear();

	for ( int i = static_cast<int>( m_RestoredCollisions.size() ) - 1; i >= 0; --i )
		m_RestoredCollisions[i]->EnableCollisions( true );
	m_RestoredCollisions.clear();

	for ( CPhysicsConstraintGroup *pGroup : m_RestoredGroups )
		pGroup->Activate();
	m_RestoredGroups.clear();
}

unsigned int CPhysicsEnvironment::GetObjectSerializeSize( IPhysicsObject * ) const
{
	return OBJECT_RECORD_SIZE;
}

void CPhysicsEnvironment::SerializeObjectToBuffer( IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize )
{
	if ( bufferSize < OBJECT_RECORD_SIZE )
		return;

	vphysics_save_cphysicsobject_t record = {};
	static_cast<const CPhysicsObject *>( pObject )->WriteRecord( record );
	const unsigned char *pRecord = reinterpret_cast<const unsigned char *>( &record );
	constexpr size_t nNameEnd = offsetof( vphysics_save_cphysicsobject_t, pName ) + sizeof( record.pName );
	constexpr size_t nGameIndexEnd = offsetof( vphysics_save_cphysicsobject_t, gameIndex ) + sizeof( record.gameIndex );
	constexpr size_t nHingeAxis = offsetof( vphysics_save_cphysicsobject_t, hingeAxis );
	std::memcpy( pBuffer, pRecord, offsetof( vphysics_save_cphysicsobject_t, pName ) );
	std::memcpy( pBuffer + nNameEnd, pRecord + nNameEnd, nGameIndexEnd - nNameEnd );
	std::memcpy( pBuffer + nHingeAxis, pRecord + nHingeAxis, OBJECT_RECORD_SIZE - nHingeAxis );
}

IPhysicsObject *CPhysicsEnvironment::UnserializeObjectFromBuffer( void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions )
{
	if ( bufferSize < OBJECT_RECORD_SIZE )
		return nullptr;

	pBuffer[offsetof( vphysics_save_cphysicsobject_t, hasShadowController )] = 0;
	vphysics_save_cphysicsobject_t record;
	std::memcpy( static_cast<void *>( &record ), pBuffer, OBJECT_RECORD_SIZE );
	CPhysicsObject *pObject = CPhysicsObject::CreateFromRecord( this, record, pGameData );
	if ( !pObject )
		return nullptr;

	if ( record.collisionEnabled && enableCollisions )
		pObject->EnableCollisions( true );
	m_Objects.push_back( pObject );
	return pObject;
}

bool CPhysicsEnvironment::TransferObject( IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment )
{
	auto it = std::find( m_Objects.begin(), m_Objects.end(), pObject );
	if ( it == m_Objects.end() || ( pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
		return false;

	CPhysicsObject *pPhysicsObject = static_cast<CPhysicsObject *>( pObject );
	void *pGameData = pPhysicsObject->GetGameData();
	IPhysicsShadowController *pShadow = pPhysicsObject->GetShadowController();

	CPhysicsPlayerController *pPlayer = nullptr;
	if ( pShadow )
	{
		pPhysicsObject->SetShadowController( nullptr );
		pPhysicsObject->DetachController( static_cast<CPhysicsShadowController *>( pShadow ) );
	}
	else if ( pPhysicsObject->GetCallbackFlags() & CALLBACK_IS_PLAYER_CONTROLLER )
	{
		auto player = std::find_if( m_PlayerControllers.rbegin(), m_PlayerControllers.rend(), [&]( CPhysicsPlayerController *p ) { return p->GetObject() == pObject; } );
		if ( player != m_PlayerControllers.rend() )
		{
			pPlayer = *player;
			m_PlayerControllers.erase( std::next( player ).base() );
			pPlayer->DetachFromObject();
		}
	}

	vphysics_save_cphysicsobject_t record = {};
	pPhysicsObject->WriteRecord( record );

	*it = m_Objects.back();
	m_Objects.pop_back();
	RemoveActiveObject( pPhysicsObject );

	CPhysicsEnvironment *pDestination = static_cast<CPhysicsEnvironment *>( pDestinationEnvironment );
	pPhysicsObject->MoveToEnvironment( pDestination, record, pGameData );
	if ( record.collisionEnabled )
		pPhysicsObject->EnableCollisions( true );
	pDestination->m_Objects.push_back( pPhysicsObject );
	pPhysicsObject->Wake();
	pPhysicsObject->ClearAsleepSinceCreation();

	if ( pPlayer )
	{
		pPlayer->AttachToObject();
		if ( std::find( pDestination->m_PlayerControllers.begin(), pDestination->m_PlayerControllers.end(), pPlayer ) == pDestination->m_PlayerControllers.end() )
			pDestination->m_PlayerControllers.push_back( pPlayer );
	}
	else if ( pShadow )
	{
		pPhysicsObject->SetShadowController( pShadow );
		pPhysicsObject->AttachController( static_cast<CPhysicsShadowController *>( pShadow ) );
	}
	return true;
}
