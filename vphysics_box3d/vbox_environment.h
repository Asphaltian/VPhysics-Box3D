//=================================================================================================
//
// Interface to a physics scene
//
//=================================================================================================

#pragma once

#include "vbox_interface.h"
#include "vphysics/performance.h"

#include <functional>

class Box3DPhysicsObject;
class Box3DPhysicsShadowController;
class Box3DPhysicsMotionController;
class Box3DPhysicsPlayerController;
class Box3DPhysicsFluidController;
class Box3DPhysicsConstraint;
class Box3DPhysicsConstraintGroup;
class Box3DPhysicsSpring;

class Box3DPhysicsEnvironment final : public IPhysicsEnvironment
{
public:
    Box3DPhysicsEnvironment();
    ~Box3DPhysicsEnvironment() override;

    void SetDebugOverlay(CreateInterfaceFn debugOverlayFactory) override;
    IVPhysicsDebugOverlay* GetDebugOverlay(void) override;

    void SetGravity(const Vector& gravityVector) override;
    void GetGravity(Vector* pGravityVector) const override;

    void SetAirDensity(float density) override;
    float GetAirDensity() const override;

    IPhysicsObject* CreatePolyObject(
        const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles,
        objectparams_t* pParams) override;
    IPhysicsObject* CreatePolyObjectStatic(
        const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles,
        objectparams_t* pParams) override;
    IPhysicsObject* CreateSphereObject(
        float radius, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams,
        bool isStatic) override;
    void DestroyObject(IPhysicsObject*) override;

    IPhysicsFluidController* CreateFluidController(IPhysicsObject* pFluidObject, fluidparams_t* pParams) override;
    void DestroyFluidController(IPhysicsFluidController*) override;

    IPhysicsSpring* CreateSpring(IPhysicsObject* pObjectStart, IPhysicsObject* pObjectEnd, springparams_t* pParams) override;
    void DestroySpring(IPhysicsSpring*) override;

    IPhysicsConstraint* CreateRagdollConstraint(
        IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
        const constraint_ragdollparams_t& ragdoll) override;
    IPhysicsConstraint* CreateHingeConstraint(
        IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
        const constraint_hingeparams_t& hinge) override;
    IPhysicsConstraint* CreateFixedConstraint(
        IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
        const constraint_fixedparams_t& fixed) override;
    IPhysicsConstraint* CreateSlidingConstraint(
        IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
        const constraint_slidingparams_t& sliding) override;
    IPhysicsConstraint* CreateBallsocketConstraint(
        IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
        const constraint_ballsocketparams_t& ballsocket) override;
    IPhysicsConstraint* CreatePulleyConstraint(
        IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
        const constraint_pulleyparams_t& pulley) override;
    IPhysicsConstraint* CreateLengthConstraint(
        IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
        const constraint_lengthparams_t& length) override;

    void DestroyConstraint(IPhysicsConstraint*) override;

    IPhysicsConstraintGroup* CreateConstraintGroup(const constraint_groupparams_t& groupParams) override;
    void DestroyConstraintGroup(IPhysicsConstraintGroup* pGroup) override;

    IPhysicsShadowController* CreateShadowController(
        IPhysicsObject* pObject, bool allowTranslation, bool allowRotation) override;
    void DestroyShadowController(IPhysicsShadowController*) override;

    IPhysicsPlayerController* CreatePlayerController(IPhysicsObject* pObject) override;
    void DestroyPlayerController(IPhysicsPlayerController*) override;

    IPhysicsMotionController* CreateMotionController(IMotionEvent* pHandler) override;
    void DestroyMotionController(IPhysicsMotionController* pController) override;

    IPhysicsVehicleController* CreateVehicleController(
        IPhysicsObject* pVehicleBodyObject, const vehicleparams_t& params, unsigned int nVehicleType,
        IPhysicsGameTrace* pGameTrace) override;
    void DestroyVehicleController(IPhysicsVehicleController*) override;

    void SetCollisionSolver(IPhysicsCollisionSolver* pSolver) override;

    void Simulate(float deltaTime) override;
    bool IsInSimulation() const override;

    float GetSimulationTimestep() const override;
    void SetSimulationTimestep(float timestep) override;

    float GetSimulationTime() const override;
    void ResetSimulationClock() override;
    float GetNextFrameTime() const override;

    void SetCollisionEventHandler(IPhysicsCollisionEvent* pCollisionEvents) override;
    void SetObjectEventHandler(IPhysicsObjectEvent* pObjectEvents) override;
    virtual void SetConstraintEventHandler(IPhysicsConstraintEvent* pConstraintEvents) override;

    void SetQuickDelete(bool bQuick) override;

    int GetActiveObjectCount() const override;
    void GetActiveObjects(IPhysicsObject** pOutputObjectList) const override;
    const IPhysicsObject** GetObjectList(int* pOutputObjectCount) const override;
    bool TransferObject(IPhysicsObject* pObject, IPhysicsEnvironment* pDestinationEnvironment) override;

    void CleanupDeleteList() override;
    void EnableDeleteQueue(bool enable) override;

    bool Save(const physsaveparams_t& params) override;
    void PreRestore(const physprerestoreparams_t& params) override;
    bool Restore(const physrestoreparams_t& params) override;
    void PostRestore() override;
#if defined(GAME_GMOD_64X)
    void PreSave(const physprerestoreparams_t& params) override;
    void PostSave() override;
#endif

    bool IsCollisionModelUsed(CPhysCollide* pCollide) const override;

    void TraceRay(const Ray_t& ray, unsigned int fMask, IPhysicsTraceFilter* pTraceFilter, trace_t* pTrace) override;
    void SweepCollideable(
        const CPhysCollide* pCollide, const Vector& vecAbsStart, const Vector& vecAbsEnd, const QAngle& vecAngles,
        unsigned int fMask, IPhysicsTraceFilter* pTraceFilter, trace_t* pTrace) override;

    void GetPerformanceSettings(physics_performanceparams_t* pOutput) const override;
    void SetPerformanceSettings(const physics_performanceparams_t* pSettings) override;

    void ReadStats(physics_stats_t* pOutput) override;
    void ClearStats() override;

    unsigned int GetObjectSerializeSize(IPhysicsObject* pObject) const override;
    void SerializeObjectToBuffer(IPhysicsObject* pObject, unsigned char* pBuffer, unsigned int bufferSize) override;
    IPhysicsObject* UnserializeObjectFromBuffer(
        void* pGameData, unsigned char* pBuffer, unsigned int bufferSize, bool enableCollisions) override;

    void EnableConstraintNotify(bool bEnable) override;
    void DebugCheckContacts() override;

    void SetAlternateGravity(const Vector& gravityVector) override_asw;
    void GetAlternateGravity(Vector* pGravityVector) const override_asw;

    float GetDeltaFrameTime(int maxTicks) const override_asw;
    void ForceObjectsToSleep(IPhysicsObject** pList, int listCount) override_asw;

    void SetPredicted(bool bPredicted) override_portal2;
    bool IsPredicted() override_portal2;
    void SetPredictionCommandNum(int iCommandNum) override_portal2;
    int GetPredictionCommandNum() override_portal2;
    void DoneReferencingPreviousCommands(int iCommandNum) override_portal2;
    void RestorePredictedSimulation() override_portal2;

    void DestroyCollideOnDeadObjectFlush(CPhysCollide*) override_portal2;

public:
    b3WorldId GetWorldId() const
    {
        return m_WorldId;
    }
    // IVP's PI/2 rad/tick angular cap, recomputed each step from the tick length. Objects clamp their
    // read-back velocity to the same value so the game never sees a "crazy angular velocity".
    float GetMaxAngularVelocity() const
    {
        return m_flMaxAngularVelocity;
    }
    float GetLastStepTime() const
    {
        return m_flLastStepTime;
    }
    IPhysicsCollisionEvent* GetCollisionEvent() const
    {
        return m_pCollisionEvent;
    }
    IPhysicsCollisionSolver* GetCollisionSolver() const
    {
        return m_pCollisionSolver;
    }
    IPhysicsObject* CreateObject(
        const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles,
        objectparams_t* pParams, bool bStatic);

#if GAME_GMOD
    void SetGModObjectEventHandler(IGModPhysicsObjectEvent* pGModObjectEvent) override
    {
        m_pGModObjectEvent = pGModObjectEvent;
    }
#endif

private:
    // Drain Box3D's post-step contact events into the game's collision callbacks.
    void DrainContactEvents();

    void DeleteObject(Box3DPhysicsObject* pObject);

    // Track a new constraint, wire it to its group, and build its joint (unless deferred to the group).
    IPhysicsConstraint* FinishConstraint(
        Box3DPhysicsConstraint* pConstraint, IPhysicsConstraintGroup* pGroup, bool bActive,
        const std::function<b3JointId()>& buildFn);

    b3WorldId m_WorldId;

    Vector m_vecGravity = vec3_origin;
    float m_flAirDensity = 2.0f;
    float m_flSimulationTimestep = 1.0f / 60.0f;
    float m_flLastStepTime = 1.0f / 60.0f;
    float m_flSimulationClock = 0.0f;
    float m_flMaxAngularVelocity = 104.0f; // PI/2 rad at ~66.7Hz; recomputed each step for the real tick
    bool m_bInSimulation = false;

    IPhysicsCollisionEvent* m_pCollisionEvent = nullptr;
    IPhysicsObjectEvent* m_pObjectEvent = nullptr;
    IPhysicsCollisionSolver* m_pCollisionSolver = nullptr;
#if GAME_GMOD
    IGModPhysicsObjectEvent* m_pGModObjectEvent = nullptr;
#endif

    CUtlVector<Box3DPhysicsObject*> m_Objects;
    mutable CUtlVector<Box3DPhysicsObject*> m_ActiveObjects;
    CUtlVector<Box3DPhysicsObject*> m_DeadObjects;
    CUtlVector<CPhysCollide*> m_DeadObjectCollides;
    bool m_bDeleteQueueEnabled = false;

    CUtlVector<Box3DPhysicsShadowController*> m_ShadowControllers;
    CUtlVector<Box3DPhysicsMotionController*> m_MotionControllers;
    CUtlVector<Box3DPhysicsPlayerController*> m_PlayerControllers;
    CUtlVector<Box3DPhysicsFluidController*> m_FluidControllers;
    CUtlVector<Box3DPhysicsConstraint*> m_Constraints;
    CUtlVector<Box3DPhysicsSpring*> m_Springs;
    physics_performanceparams_t m_PerformanceParams;
};
