//=================================================================================================
//
// Physics constraints: the 7 Source constraint types mapped onto Box3D joints.
//
//=================================================================================================

#pragma once

#include "vbox_interface.h"

#include <functional>

class Box3DPhysicsObject;
class Box3DPhysicsEnvironment;
class Box3DPhysicsConstraintGroup;

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
    b3JointId GetJointId() const
    {
        return m_JointId;
    }

    // A constrained object is being destroyed: break the joint and null the stale pointer.
    void NotifyObjectDestroyed(Box3DPhysicsObject* pObject);

private:
    void DestroyJoint();

    Box3DPhysicsEnvironment* m_pEnvironment;
    Box3DPhysicsObject* m_pReference;
    Box3DPhysicsObject* m_pAttached;
    Box3DPhysicsConstraintGroup* m_pGroup = nullptr;
    void* m_pGameData = nullptr;
    b3JointId m_JointId = b3_nullJointId;
    std::function<b3JointId()> m_BuildFn;
};

// A Source spring on a Box3D distance joint with a soft spring.
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

private:
    void PushSpringSettings();

    Box3DPhysicsEnvironment* m_pEnvironment;
    Box3DPhysicsObject* m_pStart;
    Box3DPhysicsObject* m_pEnd;
    b3JointId m_JointId = b3_nullJointId;
    b3Vec3 m_AnchorStart; // body-local anchor points, for GetEndpoints
    b3Vec3 m_AnchorEnd;
    float m_flConstant;
    float m_flDamping;
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
