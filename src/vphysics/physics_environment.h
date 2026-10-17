#pragma once

#include "vphysics_interface.h"
#include "vphysics/performance.h"
#include "vphysics/stats.h"
#include "tier0/valve_minmax_off.h"

#include "compact_builder/precision.h"
#include "physics_controller.h"
#include "physics_filter_table.h"
#include "physics_task_pool.h"

#include "box3d/collision.h"
#include "box3d/id.h"

#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class CPhysConvex;
struct CollideMesh_t;
class CPhysicsConstraint;
class CPhysicsConstraintGroup;
class CPhysicsObject;
class CPhysicsFluidController;
class CPhysicsPlayerController;

class CCollisionData final : public IPhysicsCollisionData
{
public:
	CCollisionData( const Vector &normal, const Vector &point, const Vector &speed )
	: m_Normal( normal )
	, m_Point( point )
	, m_Speed( speed )
	{
	}

	void GetSurfaceNormal( Vector &out ) override { out = m_Normal; }

	void GetContactPoint( Vector &out ) override { out = m_Point; }

	void GetContactSpeed( Vector &out ) override { out = m_Speed; }

private:
	Vector m_Normal;
	Vector m_Point;
	Vector m_Speed;
};

struct FrictionContact_t
{
	FrictionContact_t *GetNext( const CPhysicsObject *pObject ) const { return m_pNext[m_pObjects[0] == pObject ? 0 : 1]; }

	CPhysicsObject *m_pObjects[2];
	FrictionContact_t *m_pPrev[2];
	FrictionContact_t *m_pNext[2];
	uint64_t m_nSequence;
	uint32_t m_nFeatureId;
	int m_nTriangle;
	int m_nSide;
	int m_Materials[2];
	Vector m_Point;
	Vector m_Normal;
	b3Vec3 m_Span;
	float m_flFriction;
	float m_flSeparation;
	float m_flNormalImpulse;
	float m_flEnergy;
	uint32_t m_nSpanStep;
	bool m_bRemoved;
	bool m_bLinked;
	bool m_bEnding;
};

class CPhysicsEnvironment final : public IPhysicsEnvironment
{
public:
	static constexpr unsigned short NOT_LISTED = 0xFFFF;

	struct ContactNormal_t
	{
		CPhysicsObject *m_pOther;
		Vector m_Normal;
		float m_flNormalForce;
	};

	CPhysicsEnvironment();
	~CPhysicsEnvironment() override;

	void SetDebugOverlay( CreateInterfaceFn debugOverlayFactory ) override;

	IVPhysicsDebugOverlay *GetDebugOverlay() override { return m_pDebugOverlay; }

	void SetGravity( const Vector &gravityVector ) override;
	void GetGravity( Vector *pGravityVector ) const override;

	void SetAirDensity( float density ) override { m_flAirDensity = density; }

	float GetAirDensity() const override { return m_flAirDensity; }

	IPhysicsObject *CreatePolyObject( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams ) override;
	IPhysicsObject *CreatePolyObjectStatic( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams ) override;
	IPhysicsObject *CreateSphereObject( float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic ) override;
	void DestroyObject( IPhysicsObject *pObject ) override;

	IPhysicsFluidController *CreateFluidController( IPhysicsObject *pFluidObject, fluidparams_t *pParams ) override;
	void DestroyFluidController( IPhysicsFluidController *pController ) override;

	IPhysicsSpring *CreateSpring( IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams ) override;
	void DestroySpring( IPhysicsSpring *pSpring ) override;

	IPhysicsConstraint *CreateRagdollConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll ) override;
	IPhysicsConstraint *CreateHingeConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge ) override;
	IPhysicsConstraint *CreateFixedConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed ) override;
	IPhysicsConstraint *CreateSlidingConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding ) override;
	IPhysicsConstraint *CreateBallsocketConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket ) override;
	IPhysicsConstraint *CreatePulleyConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley ) override;
	IPhysicsConstraint *CreateLengthConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length ) override;
	void DestroyConstraint( IPhysicsConstraint *pConstraint ) override;

	IPhysicsConstraintGroup *CreateConstraintGroup( const constraint_groupparams_t &groupParams ) override;
	void DestroyConstraintGroup( IPhysicsConstraintGroup *pGroup ) override;

	IPhysicsShadowController *CreateShadowController( IPhysicsObject *pObject, bool allowTranslation, bool allowRotation ) override;
	void DestroyShadowController( IPhysicsShadowController *pController ) override;

	IPhysicsPlayerController *CreatePlayerController( IPhysicsObject *pObject ) override;
	void DestroyPlayerController( IPhysicsPlayerController *pController ) override;

	IPhysicsMotionController *CreateMotionController( IMotionEvent *pHandler ) override;
	void DestroyMotionController( IPhysicsMotionController *pController ) override;

	IPhysicsVehicleController *CreateVehicleController( IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace ) override;
	void DestroyVehicleController( IPhysicsVehicleController *pController ) override;

	void SetCollisionSolver( IPhysicsCollisionSolver *pSolver ) override { m_pCollisionSolver = pSolver; }

	void Simulate( float deltaTime ) override;

	bool IsInSimulation() const override { return m_bInSimulation; }

	float GetSimulationTimestep() const override { return m_flTimestep; }

	void SetSimulationTimestep( float timestep ) override;

	float GetSimulationTime() const override { return static_cast<float>( m_flCurrentTime ); }

	double GetCurrentTime() const { return m_flCurrentTime; }

	double GetLastStepTime() const { return m_flLastStepTime; }

	double GetNextStepTime() const { return m_flNextStepTime; }

	float GetInverseTimestep() const { return m_flInverseTimestep; }

	void ResetSimulationClock() override;

	float GetNextFrameTime() const override { return static_cast<float>( m_flNextStepTime ); }

	void SetCollisionEventHandler( IPhysicsCollisionEvent *pCollisionEvents ) override { m_pCollisionEvents = pCollisionEvents; }

	void SetObjectEventHandler( IPhysicsObjectEvent *pObjectEvents ) override { m_pObjectEvents = pObjectEvents; }

	void SetConstraintEventHandler( IPhysicsConstraintEvent *pConstraintEvents ) override { m_pConstraintEvents = pConstraintEvents; }

	void SetQuickDelete( bool bQuickDelete ) override { m_bQuickDelete = bQuickDelete; }

	int GetActiveObjectCount() const override { return static_cast<int>( m_ActiveObjects.size() ); }

	void GetActiveObjects( IPhysicsObject **pOutputObjectList ) const override;
	const IPhysicsObject **GetObjectList( int *pOutputObjectCount ) const override;
	bool TransferObject( IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment ) override;

	void CleanupDeleteList() override;

	void EnableDeleteQueue( bool enable ) override { m_bDeleteQueue = enable; }

	bool Save( const physsaveparams_t &params ) override;
	void PreRestore( const physprerestoreparams_t &params ) override;
	bool Restore( const physrestoreparams_t &params ) override;
	void PostRestore() override;

	bool IsCollisionModelUsed( CPhysCollide *pCollide ) const override;

	void TraceRay( const Ray_t &, unsigned int, IPhysicsTraceFilter *, trace_t * ) override {}

	void SweepCollideable( const CPhysCollide *, const Vector &, const Vector &, const QAngle &, unsigned int, IPhysicsTraceFilter *, trace_t * ) override {}

	void GetPerformanceSettings( physics_performanceparams_t *pOutput ) const override;
	void SetPerformanceSettings( const physics_performanceparams_t *pSettings ) override;

	void ReadStats( physics_stats_t *pOutput ) override;
	void ClearStats() override;

	unsigned int GetObjectSerializeSize( IPhysicsObject *pObject ) const override;
	void SerializeObjectToBuffer( IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize ) override;
	IPhysicsObject *UnserializeObjectFromBuffer( void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions ) override;

	void EnableConstraintNotify( bool bEnable ) override { m_bConstraintNotify = bEnable; }

	bool IsConstraintNotifyEnabled() const { return m_bConstraintNotify; }

	IPhysicsConstraintEvent *GetConstraintEventHandler() const { return m_pConstraintEvents; }

	IPhysicsCollisionEvent *GetCollisionEventHandler() const { return m_pCollisionEvents; }

	void DebugCheckContacts() override;

	b3WorldId GetWorld() const { return m_World; }

	float GetMaxSpeed() const { return m_flMaxSpeed; }

	float GetMaxRotationPerStep() const { return m_flMaxRotationPerStep; }

	bool AllowsFastRotation() const { return m_flMaxRotationPerStep < CONTINUOUS_ROTATION_LIMIT; }

	RealVec3_t GetIVPGravity() const { return m_IVPGravity; }

	Vector GetGravityVector() const;

	uint32_t GetStepCount() const { return m_nStepCount; }

	uint32_t AllocateObjectId() { return ++m_nLastObjectId; }

	CFilterDecisionTable &GetFilterDecisions() { return m_FilterDecisions; }

	CGravityController *GetGravityController() { return &m_GravityController; }

	CDragController *GetDragController() { return &m_DragController; }

	void QueuePendingVelocity( CPhysicsObject *pObject ) { m_PendingVelocityObjects.push_back( pObject ); }

	void RemovePendingVelocity( CPhysicsObject *pObject );

	void QueueVelocityFlush( CPhysicsObject *pObject ) { m_VelocityFlushObjects.push_back( pObject ); }

	void RemoveVelocityFlush( CPhysicsObject *pObject );
	void NotifyWoken( CPhysicsObject *pObject );
	void RemoveWoken( CPhysicsObject *pObject );
	void SyncSleepingObjects();
	void ForceRest( CPhysicsObject *pObject );
	void RemovePlayerGround( const CPhysicsObject *pObject );

	bool IsCollisionAllowed( CPhysicsObject *pObject0, CPhysicsObject *pObject1 );
	bool IsContactEnabled( const CPhysicsObject *pObject0, const CPhysicsObject *pObject1 ) const;
	void QueuePenetrationCheck( CPhysicsObject *pObject );
	bool IsShapePairHeld( b3ShapeId shape0, b3ShapeId shape1 ) const;
	void ResolveFilterDecisions( CPhysicsObject *pObject, bool bForce );
	void ResolveSteppedFilterDecisions();
	void RecheckCollisionFilter( CPhysicsObject *pObject );
	void RecheckContactPoints( CPhysicsObject *pObject );
	void RemoveContacts( CPhysicsObject *pObject, CPhysicsObject *pPartner );
	void RemoveAllContacts( CPhysicsObject *pObject );
	void RemoveObjectContacts( CPhysicsObject *pObject );
	void PrepareShapeReset( CPhysicsObject *pObject );
	void RemoveTriggerOverlaps( CPhysicsObject *pTrigger );
	void RemoveVisitorOverlaps( CPhysicsObject *pObject );
	IPhysicsFrictionSnapshot *CreateFrictionSnapshot( CPhysicsObject *pObject );
	Vector GetFrictionNormal( const FrictionContact_t &friction, const CPhysicsObject *pObject ) const;
	const std::vector<ContactNormal_t> &GetContactNormals( const CPhysicsObject *pObject );
	bool GetContactPoint( const CPhysicsObject *pObject, Vector *pContactPoint, IPhysicsObject **ppContactObject ) const;
	bool HasFrictionContact( const CPhysicsObject *pObject ) const;
	bool GetStrongestContact( const CPhysicsObject *pObject, Vector &point, int &nSurface ) const;

private:
	static constexpr int SUBSTEP_COUNT = 4;
	static constexpr float CONTINUOUS_ROTATION_LIMIT = 0.5f * B3_PI;
	static constexpr short SLEEP_COUNTDOWN = 10;

	struct ContactPoint_t
	{
		uint32_t m_nFeatureId;
		int m_nTriangle;
		uint32_t m_nStep;
		bool m_bSeparated;
	};

	struct Contact_t
	{
		CPhysicsObject *m_pObjects[2];
		const CPhysConvex *m_pConvexes[2];
		const CollideMesh_t *m_pMeshes[2];
		b3ShapeId m_Shapes[2];
		b3ContactId m_ContactId;
		uint32_t m_nKeyIndex[2];
		uint32_t m_nScanStep;
		int m_nFirstSide;
		int m_nDisableCount;
		float m_flSpanCapacity;
		float m_flMinSeparation;
		bool m_bCompound;
		bool m_bPenetrationRefused;
		bool m_bDetached;
		std::vector<ContactPoint_t> m_Points;
		std::vector<FrictionContact_t *> m_Friction;
	};

	struct ContactEvent_t
	{
		enum Type_t
		{
			START_TOUCH,
			END_TOUCH,
			IMPACT,
		};

		Type_t m_nType;
		CPhysicsObject *m_pObjects[2];
		int m_Materials[2];
		Vector m_Point;
		Vector m_Normal;
		float m_flApproachSpeed;
		FrictionContact_t *m_pEnding;
	};

	struct DisabledContact_t
	{
		b3ContactId m_ContactId;
		uint64_t m_nPairKey;
		int m_nCount;
	};

	struct TriggerOverlap_t
	{
		CPhysicsObject *m_pTrigger;
		CPhysicsObject *m_pObject;
		int m_nCount;
		bool m_bReported;
		bool m_bLeavePending;
	};

	struct PairTime_t
	{
		double m_flLastImpactTime;
		int m_nContactCount;
	};

	struct SimulationUnit_t
	{
		int m_nFirst;
		int m_nCount;
		uint32_t m_nSequence;
		bool m_bCheckRest;
	};

	struct FrictionDispatch_t
	{
		CPhysicsObject *m_pObject;
		float m_flEnergy;
		int m_nSurfaceHit;
		Vector m_Point;
		Vector m_Normal;
	};

	void UpdateContactTuning();
	void RemoveActiveObject( CPhysicsObject *pObject );
	void Step( double flTime );
	void SweepSleepingObjects();
	void BuildSimulationUnits();
	void UpdateRestTimers();
	void CheckSimulationUnits();
	void FreezeIsland( const SimulationUnit_t &unit );
	void ResolvePendingFilterPairs();
	b3AABB GetFilterQueryBounds( const CPhysicsObject *pObject, const b3AABB &bounds ) const;
	void QueryFilterPartners( CPhysicsObject *pObject, const b3AABB &query );
	void ProcessJointEvents();
	void RestoreObject( const physrestoreparams_t &params );
	void RestoreConstraint( const physrestoreparams_t &params );
	void ForgetRestoredObject( CPhysicsObject *pObject );
	bool EvaluateCollision( CPhysicsObject *pObject0, CPhysicsObject *pObject1 ) const;
	bool ShouldCollide( CPhysicsObject *pObject0, CPhysicsObject *pObject1 );
	void ProcessContactEvents();
	void ProcessSensorEvents();
	void ProcessPenetration();
	void ResolveStaticPenetration( const Contact_t &contact );
	void HoldStaticPenetration( const Contact_t &contact );
	void ResolveLedgePenetration( const CPhysConvex &convex, const CPhysicsObject *pStatic, CPhysicsObject *pObject );
	void DetectPendingPenetrationHolds();
	void TryHoldPenetration( b3ShapeId shape, b3ShapeId meshShape );
	bool FindHeldLedges( b3ShapeId shape, b3ShapeId meshShape, std::vector<int> &ledges );
	bool HasShapeFriction( const CPhysicsObject *pObject, b3ShapeId shape, b3ShapeId meshShape ) const;
	void UpdatePenetrationHolds();
	void QueuePenetrationReset( CPhysicsObject *pObject );
	void ApplyPenetrationResets();
	void ResolveHeldLedges( CPhysicsObject *pObject, CPhysicsObject *pStatic, const std::vector<int> &ledges );
	void ReleaseDisabledContacts();
	void DispatchFriction();
	void PurgePairTimes();
	void AddFrictionPair( uint64_t nPairKey );
	void RemoveFrictionPair( uint64_t nPairKey );
	void InsertContact( uint64_t nKey, Contact_t &&contact );
	void EraseContact( std::unordered_map<uint64_t, Contact_t>::iterator it );
	void RetireContact( const Contact_t &contact, bool bReport );
	void UpdateFrictionContacts( Contact_t &contact );
	bool ShouldDetachContact( const Contact_t &contact ) const;
	bool UpdateDetachedContact( Contact_t &contact );
	float GetMeshFrictionGap( const Contact_t &contact, const FrictionContact_t &friction ) const;
	FrictionContact_t *CreateFrictionContact( Contact_t &contact, const b3Manifold &manifold, const b3ManifoldPoint &point, b3Vec3 worldPoint, bool bBall );
	int GetPointMaterials( const Contact_t &contact, const b3Manifold &manifold, const b3ManifoldPoint &point, b3Vec3 worldPoint, bool bBall, int materials[2] ) const;
	void SelectLedgeImpacts();
	void AccumulateFrictionEnergy( Contact_t &contact, float flCapacity );
	void LinkFriction( FrictionContact_t *pFriction );
	void UnlinkFriction( FrictionContact_t *pFriction );
	void ReleaseFriction( const Contact_t &contact, FrictionContact_t *pFriction, bool bReport );
	void QueueContactEvent( ContactEvent_t::Type_t nType, const Contact_t &contact, const FrictionContact_t &friction, float flApproachSpeed );
	void DispatchContactEvents();
	void ReportTouch( CPhysicsObject *pObject0, CPhysicsObject *pObject1, const Vector &point, const Vector &normal, bool bStart );
	void ReportImpact( const ContactEvent_t &event );
	void ReportTrigger( CPhysicsObject *pTrigger, CPhysicsObject *pObject, bool bEnter );
	void ReportOverlap( TriggerOverlap_t &overlap, bool bEnter );
	void ReportPendingLeaves( CPhysicsObject *pObject );
	void ResetCollisionPairs( CPhysicsObject *pObject );
	void DisablePair( uint64_t nPairKey );
	void EnablePair( uint64_t nPairKey );
	int GetContactMaterial( const Contact_t &contact, int nSide, int nTriangle, const Vector &point, const Vector &outwardNormal, bool bFaceFeature ) const;
	static int GetContactLedge( const Contact_t &contact, int nTriangle );
	static int GetPointVertexOwner( const Contact_t &contact, const b3Manifold &manifold, const b3ManifoldPoint &point );

	b3WorldId m_World = b3_nullWorldId;
	IVPhysicsDebugOverlay *m_pDebugOverlay = nullptr;

	struct FilterQueryResult_t
	{
		int m_nThread;
		size_t m_nBegin;
		size_t m_nEnd;
	};

	std::vector<CPhysicsObject *> m_FilterQueryObjects;
	std::vector<FilterQueryResult_t> m_FilterQueryResults;
	std::vector<std::vector<CPhysicsObject *>> m_ThreadCandidates;
	std::vector<std::vector<CPhysicsObject *>> m_ThreadUndecided;
	CFilterDecisionTable m_FilterDecisions;
	std::vector<IPhysicsObject *> m_Objects;
	std::vector<CPhysicsObject *> m_DeadObjects;
	std::vector<CPhysicsObject *> m_ActiveObjects;
	std::vector<CPhysicsObject *> m_PendingVelocityObjects;
	std::vector<CPhysicsObject *> m_VelocityFlushObjects;
	std::vector<CPhysicsObject *> m_WokenObjects;
	std::vector<CPhysicsObject *> m_SimulatedObjects;
	std::vector<CPhysicsObject *> m_SteppedObjects;
	std::vector<CPhysicsObject *> m_FrozenObjects;
	std::vector<CPhysicsObject *> m_UnitObjects;
	std::vector<CPhysicsObject *> m_IslandObjects;
	std::vector<SimulationUnit_t> m_SimulationUnits;
	CGravityController m_GravityController;
	CDragController m_DragController;
	std::vector<CPhysicsFluidController *> m_FluidControllers;
	std::vector<CPhysicsPlayerController *> m_PlayerControllers;
	std::vector<IPhysicsConstraint *> m_DeadConstraints;
	std::vector<CPhysicsConstraint *> m_BrokenConstraints;
	std::vector<CPhysicsObject *> m_RestoredCollisions;
	std::vector<CPhysicsConstraintGroup *> m_RestoredGroups;

	std::unordered_map<uint64_t, Contact_t> m_Contacts;
	std::unordered_map<uint64_t, Contact_t> m_ProxyResetContacts;
	std::unordered_map<uint64_t, uint64_t> m_DetachedContacts;
	std::vector<uint64_t> m_ExpiredContacts;
	std::vector<DisabledContact_t> m_DisabledContacts;
	std::unordered_map<uint64_t, int> m_DisabledPairs;
	std::unordered_map<uint64_t, PairTime_t> m_PairTimes;
	std::unordered_map<uint64_t, double> m_PairTimeCache;
	std::unordered_map<uint64_t, TriggerOverlap_t> m_TriggerOverlaps;
	std::vector<uint64_t> m_PendingTriggerEnters;
	std::vector<std::pair<CPhysicsObject *, CPhysicsObject *>> m_PendingFilterPairs;
	std::mutex m_PendingFilterMutex;
	std::vector<uint64_t> m_PenetrationPairs;
	std::vector<uint64_t> m_ScannedContacts;
	std::vector<Contact_t *> m_CompoundContacts;

	struct ManifoldPoint_t
	{
		const b3Manifold *m_pManifold;
		const b3ManifoldPoint *m_pPoint;
		b3Vec3 m_WorldPoint;
		FrictionContact_t *m_pFriction;
		int m_nLedge;
	};

	struct LedgeContact_t
	{
		int m_nLedge;
		ptrdiff_t m_nDeepest;
	};

	struct ContactImpact_t
	{
		ptrdiff_t m_nPoint;
		float m_flSpeed;
	};

	std::vector<ManifoldPoint_t> m_ManifoldPoints;
	std::vector<LedgeContact_t> m_LedgeContacts;
	std::vector<ContactImpact_t> m_ContactImpacts;
	std::vector<int> m_PenetrationLedges;

	struct PenetrationHold_t
	{
		CPhysicsObject *m_pObject;
		b3ShapeId m_Shape;
		b3ShapeId m_MeshShape;
	};

	std::vector<CPhysicsObject *> m_PendingHoldChecks;
	std::vector<CPhysicsObject *> m_PenetrationResets;
	std::vector<PenetrationHold_t> m_PenetrationHolds;
	std::unordered_set<uint64_t> m_HeldShapePairs;
	std::vector<int> m_HeldLedges;
	std::vector<std::pair<int, bool>> m_LedgeStates;
	std::vector<int> m_MeshTriangles;
	std::unordered_map<uint64_t, float> m_PairSpanCapacities;
	std::vector<ContactEvent_t> m_ContactEvents;
	std::vector<FrictionDispatch_t> m_FrictionDispatches;
	std::vector<ContactNormal_t> m_ContactNormals;
	uint64_t m_nContactSequence = 0;
	uint32_t m_nLastObjectId = 0;
	uint32_t m_nStepCount = 0;
	uint32_t m_nReviveSequence = 0;
	uint32_t m_nUnitStamp = 0;
	short m_nSleepCountdown = SLEEP_COUNTDOWN;
	float m_flLastFrictionTime = 0.0f;
	vcollisionevent_t m_CollisionEvent = {};

	IPhysicsCollisionSolver *m_pCollisionSolver = nullptr;
	IPhysicsCollisionEvent *m_pCollisionEvents = nullptr;
	IPhysicsObjectEvent *m_pObjectEvents = nullptr;
	IPhysicsConstraintEvent *m_pConstraintEvents = nullptr;

	physics_performanceparams_t m_PerformanceSettings;
	physics_stats_t m_Stats = {};
	float m_flMaxSpeed = 0.0f;
	float m_flMaxRotationPerStep = 0.0f;

	RealVec3_t m_IVPGravity;
	CTaskPool m_TaskPool;
	float m_flAirDensity = 2.0f;

	double m_flCurrentTime = 0.0;
	double m_flLastStepTime = 0.0;
	double m_flNextStepTime = 0.0;
	float m_flTimestep = 0.0f;
	float m_flInverseTimestep = 0.0f;

	bool m_bInSimulation = false;
	bool m_bDispatchingFriction = false;
	bool m_bDeleteQueue = false;
	bool m_bClockAligned = true;
	bool m_bFirstStep = true;
	bool m_bConstraintNotify = false;
	bool m_bQuickDelete = false;
};
