#pragma once

#include "bspflags.h"
#include "vphysics_interface.h"
#include "tier0/valve_minmax_off.h"

#include "compact_builder/math_util.h"

#include "box3d/collision.h"
#include "box3d/id.h"
#include "box3d/math_functions.h"

#include <cfloat>
#include <string>
#include <vector>

class CPhysCollide;
class CPhysicsConstraint;
class CPhysicsEnvironment;
class CPhysicsSpring;
class CPhysicsVehicleController;
class IController;
struct FrictionContact_t;
struct vphysics_save_cphysicsobject_t;

inline constexpr uint64_t MOVING_SHAPE_BIT = 1;
inline constexpr uint64_t STATIC_SHAPE_BIT = 2;
inline constexpr uint64_t PINNED_SHAPE_BIT = 4;

class CPhysicsObject final : public IPhysicsObject
{
public:
	enum SleepState_t : unsigned char
	{
		AWAKE = 0,
		FELL_ASLEEP = 1,
		ASLEEP = 2,
	};

	enum RestState_t : unsigned char
	{
		REST_MOVING = 1,
		REST_RESTING = 2,
		REST_READY = 3,
		REST_FROZEN = 8,
	};

	static CPhysicsObject *CreatePolyObject( CPhysicsEnvironment *pEnvironment, const CPhysCollide *pCollide, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic );
	static CPhysicsObject *CreateSphereObject( CPhysicsEnvironment *pEnvironment, float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic );
	static CPhysicsObject *CreateFromRecord( CPhysicsEnvironment *pEnvironment, const vphysics_save_cphysicsobject_t &record, void *pGameData );

	~CPhysicsObject() override;

	bool IsStatic() const override { return m_bStatic; }

	bool IsAsleep() const override;

	bool IsTrigger() const override { return m_bTrigger; }

	bool IsFluid() const override { return m_pFluidController != nullptr; }

	bool IsHinged() const override { return m_nHingedAxis >= 0; }

	bool IsCollisionEnabled() const override { return m_bCollisionEnabled; }

	bool IsGravityEnabled() const override;
	bool IsDragEnabled() const override;

	bool IsMotionEnabled() const override { return m_bMotionEnabled; }

	bool IsMoveable() const override { return !m_bStatic && m_bMotionEnabled; }

	bool IsAttachedToConstraint( bool bExternalOnly ) const override;

	void EnableCollisions( bool enable ) override;
	void EnableGravity( bool enable ) override;
	void EnableDrag( bool enable ) override;
	void EnableMotion( bool enable ) override;

	void SetGameData( void *pGameData ) override { m_pGameData = pGameData; }

	void *GetGameData() const override { return m_pGameData; }

	void SetGameFlags( unsigned short userFlags ) override { m_nGameFlags = userFlags; }

	unsigned short GetGameFlags() const override { return m_nGameFlags; }

	void SetGameIndex( unsigned short gameIndex ) override { m_nGameIndex = gameIndex; }

	unsigned short GetGameIndex() const override { return m_nGameIndex; }

	void SetCallbackFlags( unsigned short callbackflags ) override { m_nCallbackFlags = callbackflags; }

	unsigned short GetCallbackFlags() const override { return m_nCallbackFlags; }

	void Wake() override;
	void Sleep() override;
	void RecheckCollisionFilter() override;
	void RecheckContactPoints() override;

	void SetMass( float mass ) override;

	float GetMass() const override { return m_flMass; }

	float GetInvMass() const override { return m_flInvMass; }

	float GetFrictionInvMass( b3Vec3 localOffset ) const;

	Vector GetInertia() const override;
	Vector GetInvInertia() const override;
	void SetInertia( const Vector &inertia ) override;
	void SetDamping( const float *speed, const float *rot ) override;
	void GetDamping( float *speed, float *rot ) const override;
	void SetDragCoefficient( float *pDrag, float *pAngularDrag ) override;

	void SetBuoyancyRatio( float ratio ) override { m_flBuoyancyRatio = ratio; }

	float GetBuoyancyRatio() const override { return m_flBuoyancyRatio; }

	int GetMaterialIndex() const override { return m_nMaterialIndex; }

	void SetMaterialIndex( int materialIndex ) override;

	unsigned int GetContents() const override { return m_nContents; }

	void SetContents( unsigned int contents ) override { m_nContents = contents; }

	float GetSphereRadius() const override;
	float GetEnergy() const override;

	Vector GetMassCenterLocalSpace() const override { return m_MassCenter; }

	void SetPosition( const Vector &worldPosition, const QAngle &angles, bool isTeleport ) override;
	void SetPositionMatrix( const matrix3x4_t &matrix, bool isTeleport ) override;
	void GetPosition( Vector *worldPosition, QAngle *angles ) const override;
	void GetPositionMatrix( matrix3x4_t *positionMatrix ) const override;

	void SetVelocity( const Vector *velocity, const AngularImpulse *angularVelocity ) override;
	void SetVelocityInstantaneous( const Vector *velocity, const AngularImpulse *angularVelocity ) override;
	void GetVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const override;
	void AddVelocity( const Vector *velocity, const AngularImpulse *angularVelocity ) override;
	void GetVelocityAtPoint( const Vector &worldPosition, Vector *pVelocity ) const override;
	void GetImplicitVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const override;

	void LocalToWorld( Vector *worldPosition, const Vector &localPosition ) const override;
	void WorldToLocal( Vector *localPosition, const Vector &worldPosition ) const override;
	void LocalToWorldVector( Vector *worldVector, const Vector &localVector ) const override;
	void WorldToLocalVector( Vector *localVector, const Vector &worldVector ) const override;

	void ApplyForceCenter( const Vector &forceVector ) override;
	void ApplyForceOffset( const Vector &forceVector, const Vector &worldPosition ) override;
	void ApplyTorqueCenter( const AngularImpulse &torque ) override;
	void CalculateForceOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerForce, AngularImpulse *centerTorque ) const override;
	void CalculateVelocityOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerVelocity, AngularImpulse *centerAngularVelocity ) const override;
	float CalculateLinearDrag( const Vector &unitDirection ) const override;
	float CalculateAngularDrag( const Vector &objectSpaceRotationAxis ) const override;

	bool GetContactPoint( Vector *contactPoint, IPhysicsObject **contactObject ) const override;

	void SetShadow( float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation ) override;
	void UpdateShadow( const Vector &targetPosition, const QAngle &targetAngles, bool tempDisableGravity, float timeOffset ) override;
	int GetShadowPosition( Vector *position, QAngle *angles ) const override;

	IPhysicsShadowController *GetShadowController() const override { return m_pShadow; }

	void RemoveShadowController() override;
	float ComputeShadowControl( const hlshadowcontrol_params_t &params, float secondsToArrival, float dt ) override;

	const CPhysCollide *GetCollide() const override { return m_pCollide; }

	const char *GetName() const override { return m_bHasName ? m_Name.c_str() : nullptr; }

	void BecomeTrigger() override;
	void RemoveTrigger() override;
	void BecomeHinged( int localAxis ) override;
	void RemoveHinged() override;

	IPhysicsFrictionSnapshot *CreateFrictionSnapshot() override;
	void DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot ) override;

	void OutputDebugInfo() const override;

	void WriteRecord( vphysics_save_cphysicsobject_t &record ) const;
	void MoveToEnvironment( CPhysicsEnvironment *pDestination, const vphysics_save_cphysicsobject_t &record, void *pGameData );

	void SetShadowController( IPhysicsShadowController *pShadow ) { m_pShadow = pShadow; }

	void SetVehicleController( CPhysicsVehicleController *pController ) { m_pVehicleController = pController; }

	void SetCarWheel() { m_bCarWheel = true; }

	void SetCarBody() { m_bCarBody = true; }

	bool IsCarCore() const { return m_bCarWheel || m_bCarBody; }

	float GetRestRadius() const { return m_flRestRadius; }

	float GetRadiusDeviation() const { return m_flRadiusDeviation; }

	void SetTouchedDynamic() { m_bHasTouchedDynamic = true; }

	void ClearAsleepSinceCreation() { m_bAsleepSinceCreation = false; }

	b3BodyId GetBody() const { return m_Body; }

	uint32_t GetId() const { return m_nId; }

	IPhysicsFluidController *GetFluidController() const { return m_pFluidController; }

	bool FindFilterDecision( const CPhysicsObject *pPartner, bool &bCollide ) const;
	void StoreFilterDecision( CPhysicsObject *pPartner, bool bCollide );
	void ClearFilterDecisions();

	bool HasFilterBounds() const { return m_bHasFilterBounds; }

	const b3AABB &GetFilterBounds() const { return m_FilterBounds; }

	void SetFilterBounds( const b3AABB &bounds )
	{
		m_FilterBounds = bounds;
		m_bHasFilterBounds = true;
	}

	void ResetShapeProxies();
	void UpdateWorldMaterials();

	void CapturePreStepVelocity( uint32_t nStepCount );

	void SetReportPreStepVelocity( bool bReport ) { m_bReportPreStepVelocity = bReport; }

	bool IsReportingPreStepVelocity() const { return m_bReportPreStepVelocity; }

	void SetReportedImpactVelocity( b3Vec3 velocity, b3Vec3 angularVelocity );

	void ClearReportedImpactVelocity()
	{
		m_bReportImpactVelocity = false;
		m_bReportImpactAngularVelocity = false;
	}

	bool IsReportingImpactVelocity() const { return m_bReportImpactVelocity || m_bReportImpactAngularVelocity; }

	b3Vec3 GetReportedTotalVelocity() const { return b3Add( GetReportedVelocity(), m_PendingVelocity ); }

	b3Vec3 GetReportedTotalAngularVelocity() const { return b3Add( GetReportedAngularVelocity(), m_PendingAngularVelocity ); }

	int GetImpactCount( uint32_t nStep ) const { return m_nImpactStep == nStep ? m_nImpactCount : 0; }

	bool IsImpactFrozen( uint32_t nStep ) const { return m_nImpactStep == nStep && m_bImpactFrozen; }

	void SetImpactState( uint32_t nStep, int nCount, bool bFrozen )
	{
		m_nImpactStep = nStep;
		m_nImpactCount = nCount;
		m_bImpactFrozen = bFrozen;
	}

	Vector GetPreStepVelocityAtPoint( const Vector &worldPosition ) const;

	b3Quat GetPreStepRotation() const;

	CPhysicsEnvironment *GetEnvironment() const { return m_pEnvironment; }

	float GetIVPSphereRadius() const { return m_flSphereRadius; }

	SleepState_t GetSleepState() const { return m_nSleepState; }

	void SetSleepState( SleepState_t nState ) { m_nSleepState = nState; }

	void NotifySleep( const b3Transform &transform );
	SleepState_t NotifyWake();
	void ResetClock( double flLastStepTime );

	double GetLastStepTime() const { return m_flLastStepTime; }

	RestState_t GetRestState() const { return m_nRestState; }

	bool IsRestFast() const { return m_bRestFast; }

	void SetRestFast( bool bFast ) { m_bRestFast = bFast; }

	uint32_t GetReviveSequence() const { return m_nReviveSequence; }

	uint32_t GetUnitStamp() const { return m_nUnitStamp; }

	void SetUnitStamp( uint32_t nStamp ) { m_nUnitStamp = nStamp; }

	void Revive( double flTime, uint32_t nSequence );
	void ResetRestTimers( double flTime );
	RestState_t CheckRest( double flTime, bool bInStep );
	bool IsReadyToRest( double flTime ) const;
	void PrepareForcedRest( double flTime );
	void Freeze( bool bInStep );

	bool IsQueuedAsWoken() const { return m_bQueuedAsWoken; }

	void SetQueuedAsWoken( bool bQueued ) { m_bQueuedAsWoken = bQueued; }

	unsigned short GetActiveIndex() const { return m_nActiveIndex; }

	void SetActiveIndex( unsigned short nIndex ) { m_nActiveIndex = nIndex; }

	void RebuildCoreMatrix( double flTime );
	void UpdateTransform( const b3Transform &transform );
	void ApplyPendingVelocity();
	void FlushVelocity();
	void ApplyImpulse( DoubleVec3_t worldPosition, b3Vec3 impulse );
	void ApplyDirectImpulse( DoubleVec3_t worldPosition, b3Vec3 impulse );
	void AddPendingAngularVelocity( b3Vec3 change );
	void ApplyGravity( float flTimestep, DoubleVec3_t gravity, bool bWorldGravity );
	void ApplyDrag( float flTimestep, float flAirDensity );
	void ClampStepVelocity();
	void UpdateFastRotation();
	void UpdateFastMotion( float flTimestep );
	void UpdateBullet( float flTimestep );

	void AttachController( IController *pController );
	void DetachController( IController *pController );
	void AttachConstraint( CPhysicsConstraint *pConstraint );
	void DetachConstraint( CPhysicsConstraint *pConstraint );
	void AttachSpring( CPhysicsSpring *pSpring );
	void DetachSpring( CPhysicsSpring *pSpring );

	const std::vector<CPhysicsConstraint *> &GetConstraints() const { return m_Constraints; }

	void QueueStiffnessUpdate();

	void ClearStiffnessQueued() { m_bStiffnessQueued = false; }

	uint32_t GetStiffnessPass() const { return m_nStiffnessPass; }

	void SetStiffnessPass( uint32_t nPass ) { m_nStiffnessPass = nPass; }

	float GetStiffnessScale() const { return m_flStiffnessScale; }

	void SetStiffnessScale( float flScale ) { m_flStiffnessScale = flScale; }

	std::vector<uint64_t> &GetContactKeys() { return m_ContactKeys; }

	const std::vector<uint64_t> &GetContactKeys() const { return m_ContactKeys; }

	FrictionContact_t *GetFrictionHead() const { return m_pFrictionHead; }

	bool IsBullet() const { return m_bBullet; }

	float GetFastDistance() const { return m_flFastDistance; }

	float GetMaxExtent() const { return m_flMaxExtent; }

	void SetFrictionHead( FrictionContact_t *pFriction ) { m_pFrictionHead = pFriction; }

	uint32_t GetPlacedStep() const { return m_nPlacedStep; }

	void SetPlacedStep( uint32_t nStep ) { m_nPlacedStep = nStep; }

	void ResetControllerCursor( uint32_t nStep )
	{
		m_nControllerCursor = static_cast<int>( m_ControllerOrder.size() ) - 1;
		m_nControllerStep = nStep;
	}

	uint32_t GetControllerStep() const { return m_nControllerStep; }

	int GetHighestControllerPriority() const;

	void RunControllers( float flTimestep, int nPriorityFloor );

	struct ShadowControl_t
	{
		DoubleVec3_t m_TargetPosition;
		DoubleQuat_t m_TargetRotation;
		float m_flMaxSpeed;
		float m_flMaxDampSpeed;
		float m_flMaxAngular;
		float m_flMaxDampAngular;
		float m_flDampFactor;
		float m_flTeleportDistance;
		DoubleVec3_t m_LastPosition;
		b3Vec3 m_LastImpulse;
	};

	void ApplyShadowControl( ShadowControl_t &control, float flSecondsToArrival, float flTimestep );
	void GetPose( double flTime, DoubleMatrix3_t &rotation, DoubleVec3_t &origin ) const;
	void MoveTo( DoubleVec3_t origin, DoubleQuat_t rotation, bool isTeleport );
	void StepUp( float height );

	DoubleQuat_t GetNextRotation() const { return m_NextRotation; }

	DoubleQuat_t GetRotationAt( double flTime ) const;

	float GetLastStepMotion() const { return m_flLastStepMotion; }

	const DoubleMatrix3_t &GetCoreRotation() const { return m_CoreRotation; }

	DoubleVec3_t GetCorePosition() const { return m_CorePosition; }

	DoubleVec3_t GetCoreOrigin() const;
	b3Vec3 GetCoreVelocity() const;
	void SetCoreVelocity( b3Vec3 velocity );
	b3Vec3 GetCoreAngularVelocity() const;
	void SetCoreAngularVelocity( b3Vec3 angular );
	b3Vec3 GetCoreInverseInertia() const;
	b3Vec3 IVPToCore( b3Vec3 worldIVP ) const;
	b3Vec3 CoreToIVP( b3Vec3 local ) const;
	b3Vec3 GetCoreSurfaceVelocity( b3Vec3 localPoint ) const;
	DoubleVec3_t TransformCorePoint( b3Vec3 localPoint ) const;

	b3Vec3 GetPendingVelocity() const { return m_PendingVelocity; }

	void SetPendingVelocity( b3Vec3 velocity );

	b3Vec3 GetPendingAngularVelocity() const { return m_PendingAngularVelocity; }

	void SetPendingAngularVelocity( b3Vec3 angular );

	b3Vec3 GetCoreImplicitVelocity() const { return m_ImplicitVelocity; }

	b3Vec3 GetShift() const { return m_Shift; }

	b3Vec3 GetReportedVelocity() const;
	b3Vec3 GetReportedAngularVelocity() const;

	void ClampVelocity( bool bExemptShadow );
	void ChangeMass( float mass );

	float GetRotDamping() const { return m_flRotDamping; }

	void SetRotDamping( float flRotDamping ) { m_flRotDamping = flRotDamping; }

	void SetFluidController( IPhysicsFluidController *pController ) { m_pFluidController = pController; }

private:
	CPhysicsObject( CPhysicsEnvironment *pEnvironment, const objectparams_t &params, bool isStatic );

	void Initialize( const objectparams_t &params, int materialIndex, DoubleVec3_t origin, DoubleQuat_t rotation, b3Vec3 massCenter, float flVolume );
	void InitializeFromRecord( const objectparams_t &params, const vphysics_save_cphysicsobject_t &record, void *pGameData );
	void InitializePose( DoubleVec3_t origin, DoubleQuat_t rotation, b3Vec3 massCenter );
	void ComputeRestRadius( b3Vec3 massCenter );
	void SetContactGravity( bool bContactGravity );
	double GetRotationDistance( DoubleQuat_t a, DoubleQuat_t b ) const;
	void GetRestPose( bool bInStep, DoubleVec3_t &position, DoubleQuat_t &nextRotation, DoubleQuat_t &lastRotation ) const;
	RestState_t EvaluateRest( double flTime, bool bInStep, bool &bMoved, bool &bCoarseMoved ) const;
	DoubleVec3_t GetCorePositionAt( double flTime ) const;
	b3Transform GetBodyTransform() const;
	bool HasController( const IController *pController ) const;
	void InvalidateVelocityCache();
	void QueueVelocityFlush();
	float GetLinearDrag( b3Vec3 coreDirection ) const;
	float GetAngularDrag( b3Vec3 coreAxis ) const;
	b3Vec3 GetCoreOffset( const Vector &worldPosition ) const;
	b3Vec3 GetBodyAngularVelocity( b3Vec3 coreAngular ) const;
	void ComputeMassProperties( const objectparams_t &params, b3Vec3 massCenter, b3Vec3 unitInertia );
	bool IsSolverIsolated() const;
	void CreateShapes( bool bSensor );
	b3Filter GetShapeFilter( bool bSensor ) const;
	void UpdateShapeFilters( bool bInvokeContacts );
	void DestroyShapes();
	void SetVolume( float volume );
	void ComputeDragBases();
	void UpdateInverseMass();
	void UpdateMassData();
	void ApplyMassData();
	void QueuePendingVelocity();

	CPhysicsEnvironment *m_pEnvironment;
	b3BodyId m_Body = b3_nullBodyId;
	uint32_t m_nId = 0;
	uint32_t m_nPlacedStep = 0;
	b3AABB m_FilterBounds = {};
	std::vector<IController *> m_Controllers;
	std::vector<IController *> m_ControllerOrder;
	std::vector<CPhysicsConstraint *> m_Constraints;
	std::vector<CPhysicsSpring *> m_Springs;
	std::vector<uint64_t> m_ContactKeys;
	FrictionContact_t *m_pFrictionHead = nullptr;
	b3Vec3 m_PreStepVelocity = b3Vec3_zero;
	b3Vec3 m_PreStepAngularVelocity = b3Vec3_zero;
	b3Quat m_PreStepRotation = b3Quat_identity;
	b3Vec3 m_ImpactVelocity = b3Vec3_zero;
	b3Vec3 m_ImpactAngularVelocity = b3Vec3_zero;
	float m_flFastDistance = 0.0f;
	float m_flStiffnessScale = 1.0f;
	uint32_t m_nStiffnessPass = 0;
	float m_flMaxExtent = 0.0f;
	uint32_t m_nPreStepCount = 0;
	int m_nControllerCursor = -1;
	uint32_t m_nControllerStep = UINT32_MAX;
	uint32_t m_nImpactStep = UINT32_MAX;
	int m_nImpactCount = 0;
	const CPhysCollide *m_pCollide = nullptr;
	void *m_pGameData = nullptr;
	IPhysicsShadowController *m_pShadow = nullptr;
	IPhysicsFluidController *m_pFluidController = nullptr;
	CPhysicsVehicleController *m_pVehicleController = nullptr;
	std::string m_Name;

	b3Vec3 m_PendingVelocity = b3Vec3_zero;
	b3Vec3 m_PendingAngularVelocity = b3Vec3_zero;
	b3Vec3 m_CoreVelocity = b3Vec3_zero;
	b3Vec3 m_CoreAngularVelocity = b3Vec3_zero;

	DoubleVec3_t m_CorePosition = {};
	DoubleMatrix3_t m_CoreRotation = {};
	DoubleQuat_t m_LastRotation = { 0, 0, 0, 1 };
	DoubleQuat_t m_NextRotation = { 0, 0, 0, 1 };
	b3Vec3 m_ImplicitVelocity = b3Vec3_zero;
	b3Vec3 m_Shift = b3Vec3_zero;
	double m_flLastStepTime = 0.0;
	float m_flInverseStepInterval = 0.0f;

	float m_flMass = 0.0f;
	float m_flInvMass = 0.0f;
	Vector m_Inertia = Vector( 0.0f, 0.0f, 0.0f );
	Vector m_InvInertia = Vector( 0.0f, 0.0f, 0.0f );
	Vector m_MassCenter = Vector( 0.0f, 0.0f, 0.0f );
	float m_flSpeedDamping = 0.0f;
	float m_flRotDamping = 0.0f;

	float m_LinearDragBasis[3] = {};
	float m_AngularDragBasis[3] = {};
	float m_flDragCoefficient = 0.0f;
	float m_flAngularDragCoefficient = 0.0f;

	float m_flVolume = 0.0f;
	float m_flBuoyancyRatio = 0.0f;
	float m_flSphereRadius = 0.0f;

	unsigned int m_nContents = CONTENTS_SOLID;
	int m_nHingedAxis = -1;
	unsigned short m_nMaterialIndex = 0;
	unsigned short m_nGameIndex = 0;
	unsigned short m_nGameFlags = 0;
	unsigned short m_nCallbackFlags = 0;
	unsigned short m_nActiveIndex = 0xFFFF;
	RestState_t m_nRestState = REST_FROZEN;
	uint32_t m_nReviveSequence = 0;
	uint32_t m_nUnitStamp = 0;
	DoubleVec3_t m_PreviousCorePosition = {};
	DoubleQuat_t m_PreviousRotation = { 0, 0, 0, 1 };
	DoubleVec3_t m_FineRestPosition = {};
	DoubleQuat_t m_FineRestRotation = {};
	DoubleVec3_t m_CoarseRestPosition = {};
	DoubleQuat_t m_CoarseRestRotation = {};
	double m_flFineRestTime = 0.0;
	double m_flCoarseRestTime = 0.0;
	float m_flRestRadius = 0.0f;
	float m_flRadiusDeviation = 0.0f;
	float m_flLastStepMotion = 0.0f;
	SleepState_t m_nSleepState = ASLEEP;

	bool m_bStatic = false;
	bool m_bMotionEnabled = true;
	bool m_bCollisionEnabled = false;
	bool m_bTrigger = false;
	bool m_bShadowTempGravityDisable = false;
	bool m_bPendingVelocity = false;
	bool m_bVelocityFlushQueued = false;
	bool m_bStiffnessQueued = false;
	bool m_bQueuedAsWoken = false;
	bool m_bHasFilterBounds = false;
	bool m_bCoreVelocityValid = false;
	bool m_bCoreAngularVelocityValid = false;
	bool m_bNoShift = true;
	bool m_bReportPreStepVelocity = false;
	bool m_bPreStepVelocityValid = true;
	bool m_bPreStepAngularVelocityValid = true;
	bool m_bAsleepSinceCreation = false;
	bool m_bHasName = false;
	bool m_bHasTouchedDynamic = false;
	bool m_bCarWheel = false;
	bool m_bCarBody = false;
	bool m_bReportImpactVelocity = false;
	bool m_bReportImpactAngularVelocity = false;
	bool m_bImpactFrozen = false;
	bool m_bGravityApplied = false;
	bool m_bContactGravity = false;
	bool m_bRestFast = false;
	bool m_bFastMotion = false;
	bool m_bBullet = false;
};
