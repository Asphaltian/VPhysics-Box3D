//=================================================================================================
//
// Physics constraints: the 7 Source constraint types mapped onto Box3D joints.
//
//=================================================================================================

#pragma once

#include "vbox_interface.h"

#include <cstring>
#include <functional>

class Box3DPhysicsObject;
class Box3DPhysicsEnvironment;
class Box3DPhysicsConstraintGroup;

// Source constraint type + original params, so save/restore rebuilds via the same CreateXConstraint path.
enum Box3DConstraintKind
{
    kBox3DConstraint_None = 0,
    kBox3DConstraint_Ragdoll,
    kBox3DConstraint_Hinge,
    kBox3DConstraint_Fixed,
    kBox3DConstraint_Sliding,
    kBox3DConstraint_Ballsocket,
    kBox3DConstraint_Pulley,
    kBox3DConstraint_Length,
};

union Box3DConstraintParams
{
    // Members carry default initializers, so the union needs explicit no-op special members; it is only ever
    // filled as a byte blob.
    Box3DConstraintParams()
    {
    }
    ~Box3DConstraintParams()
    {
    }
    constraint_ragdollparams_t ragdoll;
    constraint_hingeparams_t hinge;
    constraint_fixedparams_t fixed;
    constraint_slidingparams_t sliding;
    constraint_ballsocketparams_t ballsocket;
    constraint_pulleyparams_t pulley;
    constraint_lengthparams_t length;
};

// Box3D joints have no enable flag, so Activate/Deactivate rebuild/destroy the joint via a stored closure.
class Box3DPhysicsConstraint final : public IPhysicsConstraint
{
public:
    Box3DPhysicsConstraint(
        Box3DPhysicsEnvironment* pEnvironment, Box3DPhysicsObject* pReference, Box3DPhysicsObject* pAttached);
    ~Box3DPhysicsConstraint() override;

    void Activate() override;
    void Deactivate() override;
    void SetGameData(void* gameData) override
    {
        m_pGameData = gameData;
    }
    void* GetGameData() const override
    {
        return m_pGameData;
    }
    IPhysicsObject* GetReferenceObject() const override;
    IPhysicsObject* GetAttachedObject() const override;
    void SetLinearMotor(float speed, float maxLinearImpulse) override;
    void SetAngularMotor(float rotSpeed, float maxAngularImpulse) override;
    void UpdateRagdollTransforms(const matrix3x4_t&, const matrix3x4_t&) override
    {
    }
    bool GetConstraintTransform(matrix3x4_t* pConstraintToReference, matrix3x4_t* pConstraintToAttached) const override;
    bool GetConstraintParams(constraint_breakableparams_t* pParams) const override;
    void OutputDebugInfo() override
    {
    }

    // Store the joint builder and create it now if bActive (grouped constraints defer to the group's Activate).
    void Init(const std::function<b3JointId()>& buildFn, bool bActive);
    void SetGroup(Box3DPhysicsConstraintGroup* pGroup)
    {
        m_pGroup = pGroup;
    }
    Box3DPhysicsConstraintGroup* GetGroup() const
    {
        return m_pGroup;
    }
    b3JointId GetJointId() const
    {
        return m_JointId;
    }

    // A constrained object is being destroyed: break the joint and null the stale pointer. Returns true the
    // first time (so the environment fires ConstraintBroken once).
    bool NotifyObjectDestroyed(Box3DPhysicsObject* pObject);

    // The joint exceeded its force/torque limit: destroy it and mark the constraint broken (it stays broken).
    void OnBroken();
    void SetBreakParams(const constraint_breakableparams_t& params)
    {
        m_BreakParams = params;
    }

    // Remember the source constraint type + params so save/restore can rebuild it via CreateXConstraint.
    void SetSaveInfo(Box3DConstraintKind kind, const void* pParams, int nSize)
    {
        m_SaveKind = kind;
        memcpy(&m_SaveParams, pParams, (size_t)nSize);
    }
    Box3DConstraintKind GetSaveKind() const
    {
        return m_SaveKind;
    }
    bool IsBroken() const
    {
        return m_bBroken;
    }
    const Box3DConstraintParams& GetSaveParams() const
    {
        return m_SaveParams;
    }

    // Pulley: Box3D has no pulley joint, so it is solved as a per-step impulse constraint coupling the two
    // rope segments (|A-pulleyA| + gearRatio*|B-pulleyB| = totalLength). SolvePulley runs each step.
    void SetupPulley(const b3Vec3 pulleyWorld[2], const b3Vec3 localAttach[2], float totalLength, float gearRatio, bool rigid);
    void SolvePulley(float dt);
    bool IsPulley() const
    {
        return m_bPulley;
    }

    // Ragdoll angular limits + joint friction, solved per step like Havana's hk_Constraint_Limit.
    void SetupAngularLimits(
        const b3Transform& frameRef, const b3Transform& frameAtt, bool bCone, float coneAngle, bool bTwist, float twistMin,
        float twistMax, float flFriction, bool bHasJoint);
    bool SolveAngularLimits(float dt, bool bApplyFriction);
    bool IsAngularLimits() const
    {
        return m_bAngularLimits;
    }
    void SetAngularFriction(float flFriction)
    {
        m_flAngFriction = flFriction;
    }

private:
    void DestroyJoint();
    // Apply the breakable params to the live joint: force/torque break thresholds and constraint strength.
    void ApplyConstraintTuning();

    Box3DPhysicsEnvironment* m_pEnvironment;
    Box3DPhysicsObject* m_pReference;
    Box3DPhysicsObject* m_pAttached;
    Box3DPhysicsConstraintGroup* m_pGroup = nullptr;
    void* m_pGameData = nullptr;
    b3JointId m_JointId = b3_nullJointId;
    std::function<b3JointId()> m_BuildFn;
    constraint_breakableparams_t m_BreakParams = {};
    bool m_bBroken = false;

    bool m_bPulley = false;
    b3Vec3 m_PulleyWorld[2] = {}; // pulley pivots, world space (metres)
    b3Vec3 m_PulleyLocal[2] = {}; // attach points, body-origin-local (metres)
    float m_flPulleyTotalLength = 0.0f;
    float m_flPulleyGearRatio = 1.0f;
    bool m_bPulleyRigid = false;

    bool m_bAngularLimits = false;
    b3Quat m_AngFrameRef = {}; // constraint frame in each body's local space (twist axis = frame X)
    b3Quat m_AngFrameAtt = {};
    bool m_bAngCone = false;
    float m_flAngCone = 0.0f; // max swing angle (rad)
    bool m_bAngTwist = false;
    float m_flAngTwistMin = 0.0f;
    float m_flAngTwistMax = 0.0f;
    float m_flTwistUnwrapped = 0.0f; // continuously tracked twist (Havana keeps limits turn-aware)
    float m_flTwistLastRaw = 0.0f;
    bool m_bTwistInit = false;
    float m_flAngFriction = 0.0f;      // joint friction torque (Havana m_joint_friction)
    float m_flFrictionRefTwist = 0.0f; // friction servo reference angles (Havana m_ref_position)
    float m_flFrictionRefSwing = 0.0f;
    bool m_bFrictionInit = false;
    bool m_bAngHasJoint = false; // re-pin the ball anchor after angular impulses
    b3Vec3 m_AngAnchorRef = {};  // joint anchor, body-local
    b3Vec3 m_AngAnchorAtt = {};

    Box3DConstraintKind m_SaveKind = kBox3DConstraint_None;
    Box3DConstraintParams m_SaveParams{};
};

// IVP_Actuator_Spring, ported per step. Force-based (F = k * stretch): a hertz-based joint spring
// bakes the endpoint masses in at creation and detunes when a frozen endpoint is later unfrozen.
class Box3DPhysicsSpring final : public IPhysicsSpring
{
public:
    Box3DPhysicsSpring(
        Box3DPhysicsEnvironment* pEnvironment, Box3DPhysicsObject* pStart, Box3DPhysicsObject* pEnd,
        const springparams_t* pParams);
    ~Box3DPhysicsSpring() override;

    void GetEndpoints(Vector* worldPositionStart, Vector* worldPositionEnd) override;
    void SetSpringConstant(float flSpringConstant) override;
    void SetSpringDamping(float flSpringDamping) override;
    void SetSpringLength(float flSpringLength) override;
    IPhysicsObject* GetStartObject() override;
    IPhysicsObject* GetEndObject() override;

    void NotifyObjectDestroyed(Box3DPhysicsObject* pObject);

    void SetSaveParams(const springparams_t& params)
    {
        m_SaveParams = params;
    }
    const springparams_t& GetSaveParams() const
    {
        return m_SaveParams;
    }

    void Simulate(float dt);

private:
    Box3DPhysicsEnvironment* m_pEnvironment;
    Box3DPhysicsObject* m_pStart;
    Box3DPhysicsObject* m_pEnd;
    springparams_t m_SaveParams = {};
    b3Vec3 m_AnchorStart; // body-local anchor points, for GetEndpoints
    b3Vec3 m_AnchorEnd;
    float m_flNaturalLen; // metres
    float m_flConstant;
    float m_flDamping;
    float m_flRelativeDamping = 0.0f;
    bool m_bOnlyStretch = false;
};

class Box3DPhysicsConstraintGroup final : public IPhysicsConstraintGroup
{
public:
    void Activate() override;
    bool IsInErrorState() override
    {
        return false;
    }
    void ClearErrorState() override
    {
    }
    void GetErrorParams(constraint_groupparams_t* pParams) override
    {
        if (pParams)
            *pParams = m_Params;
    }
    void SetErrorParams(const constraint_groupparams_t& params) override
    {
        m_Params = params;
    }
    void SolvePenetration(IPhysicsObject*, IPhysicsObject*) override
    {
    }

    void AddConstraint(Box3DPhysicsConstraint* pConstraint)
    {
        m_Constraints.AddToTail(pConstraint);
    }
    void RemoveConstraint(Box3DPhysicsConstraint* pConstraint)
    {
        m_Constraints.FindAndRemove(pConstraint);
    }

private:
    CUtlVector<Box3DPhysicsConstraint*> m_Constraints;
    constraint_groupparams_t m_Params = {};
};
