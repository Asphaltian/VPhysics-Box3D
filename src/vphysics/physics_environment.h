#pragma once

#include "vphysics_interface.h"
#include "vphysics/performance.h"
#include "vphysics/stats.h"
#include "tier0/valve_minmax_off.h"

#include "compact_builder/precision.h"
#include "physics_controller.h"

#include "box3d/collision.h"
#include "box3d/id.h"

#include <mutex>
#include <unordered_map>
#include <vector>

class CPhysConvex;
class CPhysicsObject;
class CPhysicsFluidController;
class CPhysicsPlayerController;

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

	void SetQuickDelete( bool ) override {}

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

	void DebugCheckContacts() override;

	b3WorldId GetWorld() const { return m_World; }

	float GetMaxSpeed() const { return m_flMaxSpeed; }

	float GetMaxRotationPerStep() const { return m_flMaxRotationPerStep; }

	RealVec3_t GetIVPGravity() const { return m_IVPGravity; }

	uint32_t GetStepCount() const { return m_nStepCount; }

	uint32_t AllocateObjectId() { return ++m_nLastObjectId; }

	CGravityController *GetGravityController() { return &m_GravityController; }

	CDragController *GetDragController() { return &m_DragController; }

	void QueuePendingVelocity( CPhysicsObject *pObject ) { m_PendingVelocityObjects.push_back( pObject ); }

	void RemovePendingVelocity( CPhysicsObject *pObject );

	void QueueVelocityFlush( CPhysicsObject *pObject ) { m_VelocityFlushObjects.push_back( pObject ); }

	void RemoveVelocityFlush( CPhysicsObject *pObject );
	void NotifyWoken( CPhysicsObject *pObject );
	void RemoveWoken( CPhysicsObject *pObject );
	void SyncSleepingObjects();
	void RemovePlayerGround( const CPhysicsObject *pObject );

	bool IsCollisionAllowed( CPhysicsObject *pObject0, CPhysicsObject *pObject1 );
	bool IsContactEnabled( const CPhysicsObject *pObject0, const CPhysicsObject *pObject1 ) const;
	void ResolveFilterDecisions( CPhysicsObject *pObject, bool bForce );
	void RecheckCollisionFilter( CPhysicsObject *pObject );
	void RecheckContactPoints( CPhysicsObject *pObject );
	void RemoveContacts( CPhysicsObject *pObject, CPhysicsObject *pPartner );
	void RemoveAllContacts( CPhysicsObject *pObject );
	void RemoveObjectContacts( CPhysicsObject *pObject );
	void PrepareShapeReset( CPhysicsObject *pObject );
	void RemoveTriggerOverlaps( CPhysicsObject *pTrigger );
	void RemoveVisitorOverlaps( CPhysicsObject *pObject );
	IPhysicsFrictionSnapshot *CreateFrictionSnapshot( CPhysicsObject *pObject );
	bool RefreshFrictionContact( const CPhysicsObject *pObject, uint64_t nSequence, Vector &point, Vector &normal ) const;
	const std::vector<ContactNormal_t> &GetContactNormals( const CPhysicsObject *pObject );
	bool GetContactPoint( const CPhysicsObject *pObject, Vector *pContactPoint, IPhysicsObject **ppContactObject ) const;

private:
	static constexpr int SUBSTEP_COUNT = 4;

	struct FrictionContact_t
	{
		uint64_t m_nSequence;
		uint32_t m_nFeatureId;
		int m_nTriangle;
		int m_nSide;
		int m_Materials[2];
		Vector m_Point;
		Vector m_Normal;
		float m_flFriction;
		float m_flSeparation;
		float m_flNormalImpulse;
		float m_flEnergy;
		bool m_bSeparated;
		bool m_bRemoved;
	};

	struct Contact_t
	{
		CPhysicsObject *m_pObjects[2];
		const CPhysConvex *m_pConvexes[2];
		b3ShapeId m_Shapes[2];
		b3ContactId m_ContactId;
		uint32_t m_nKeyIndex[2];
		uint32_t m_nScanStep;
		int m_nFirstSide;
		int m_nDisableCount;
		bool m_bPenetrationRefused;
		std::vector<FrictionContact_t> m_Friction;
	};

	struct FrictionRef_t
	{
		const Contact_t *m_pContact;
		const FrictionContact_t *m_pFriction;
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

	struct FrictionDispatch_t
	{
		CPhysicsObject *m_pObject;
		float m_flEnergy;
		int m_nSurfaceHit;
		Vector m_Point;
		Vector m_Normal;
	};

	Vector GetGravityVector() const;
	void UpdateContactTuning();
	void RemoveActiveObject( CPhysicsObject *pObject );
	void Step( double flTime );
	void SweepSleepingObjects();
	void ResolvePendingFilterPairs();
	bool EvaluateCollision( CPhysicsObject *pObject0, CPhysicsObject *pObject1 ) const;
	bool ShouldCollide( CPhysicsObject *pObject0, CPhysicsObject *pObject1 );
	void ProcessContactEvents();
	void ProcessSensorEvents();
	void ProcessPenetration();
	void ReleaseDisabledContacts();
	void DispatchFriction();
	void PurgePairTimes();
	void AddFrictionPair( uint64_t nPairKey );
	void RemoveFrictionPair( uint64_t nPairKey );
	const std::vector<FrictionRef_t> &GetSortedContacts( const CPhysicsObject *pObject ) const;
	void InsertContact( uint64_t nKey, Contact_t &&contact );
	void EraseContact( std::unordered_map<uint64_t, Contact_t>::iterator it );
	void RetireContact( const Contact_t &contact, bool bReport );
	void UpdateFrictionContacts( Contact_t &contact );
	void ReleaseFriction( const Contact_t &contact, const FrictionContact_t &friction, bool bReport );
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
	int GetContactMaterial( const CPhysicsObject *pObject, const CPhysConvex *pConvex, const Vector &point, const Vector &outwardNormal, bool bFaceFeature ) const;

	b3WorldId m_World = b3_nullWorldId;
	IVPhysicsDebugOverlay *m_pDebugOverlay = nullptr;

	std::vector<IPhysicsObject *> m_Objects;
	std::vector<CPhysicsObject *> m_DeadObjects;
	std::vector<CPhysicsObject *> m_ActiveObjects;
	std::vector<CPhysicsObject *> m_PendingVelocityObjects;
	std::vector<CPhysicsObject *> m_VelocityFlushObjects;
	std::vector<CPhysicsObject *> m_WokenObjects;
	std::vector<CPhysicsObject *> m_SimulatedObjects;
	std::vector<CPhysicsObject *> m_SteppedObjects;
	CGravityController m_GravityController;
	CDragController m_DragController;
	std::vector<CPhysicsFluidController *> m_FluidControllers;
	std::vector<CPhysicsPlayerController *> m_PlayerControllers;
	std::vector<IPhysicsConstraint *> m_DeadConstraints;

	std::unordered_map<uint64_t, Contact_t> m_Contacts;
	std::unordered_map<uint64_t, Contact_t> m_ProxyResetContacts;
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
	std::vector<ContactEvent_t> m_ContactEvents;
	std::vector<FrictionDispatch_t> m_FrictionDispatches;
	std::vector<ContactNormal_t> m_ContactNormals;
	mutable std::vector<FrictionRef_t> m_SortedContacts;
	uint64_t m_nContactSequence = 0;
	uint32_t m_nLastObjectId = 0;
	uint32_t m_nStepCount = 0;
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
};
