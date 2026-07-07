//=================================================================================================
//
// Physics constraints: the 7 Source constraint types mapped onto Box3D joints.
//
//=================================================================================================

#include "vbox_constraints.h"

#include "cbase.h"
#include "vbox_environment.h"
#include "vbox_object.h"
#include "vbox_util.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

namespace
{
    b3Vec3 SafeNormalize(b3Vec3 v)
    {
        const float flLen = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
        return flLen > 1e-9f ? b3Vec3{ v.x / flLen, v.y / flLen, v.z / flLen } : b3Vec3{ 0.0f, 0.0f, 1.0f };
    }

    // World point in a body's local (origin-relative) frame.
    b3Vec3 WorldToLocalPoint(b3BodyId body, b3Vec3 worldPoint)
    {
        const b3WorldTransform wt = b3Body_GetTransform(body);
        return b3InvRotateVector(wt.q, b3Sub(worldPoint, b3ToVec3(wt.p)));
    }

    b3Quat BodyRotation(b3BodyId body)
    {
        return b3Body_GetTransform(body).q;
    }
    b3Vec3 BodyOrigin(b3BodyId body)
    {
        return b3ToVec3(b3Body_GetTransform(body).p);
    }

    // Body-local frame that maps fromAxis (revolute Z / prismatic X) onto worldAxis.
    b3Quat LocalFrameForAxis(b3BodyId body, b3Vec3 fromAxis, b3Vec3 worldAxis)
    {
        const b3Quat qWorld = b3ComputeQuatBetweenUnitVectors(fromAxis, SafeNormalize(worldAxis));
        return b3InvMulQuat(BodyRotation(body), qWorld);
    }

    float ClampAngle(float radians, float limit)
    {
        return clamp(radians, -limit, limit);
    }
} // namespace

//-------------------------------------------------------------------------------------------------
// Box3DPhysicsConstraint
//-------------------------------------------------------------------------------------------------

Box3DPhysicsConstraint::Box3DPhysicsConstraint(
    Box3DPhysicsEnvironment* pEnvironment, Box3DPhysicsObject* pReference, Box3DPhysicsObject* pAttached)
    : m_pEnvironment(pEnvironment)
    , m_pReference(pReference)
    , m_pAttached(pAttached)
{
}

Box3DPhysicsConstraint::~Box3DPhysicsConstraint()
{
    if (m_pGroup)
        m_pGroup->RemoveConstraint(this);
    DestroyJoint();
}

void Box3DPhysicsConstraint::Init(const std::function<b3JointId()>& buildFn, bool bActive)
{
    m_BuildFn = buildFn;
    if (bActive)
        Activate();
}

void Box3DPhysicsConstraint::DestroyJoint()
{
    if (b3Joint_IsValid(m_JointId))
        b3DestroyJoint(m_JointId, true);
    m_JointId = b3_nullJointId;
}

void Box3DPhysicsConstraint::Activate()
{
    if (m_bBroken)
        return;
    if (!b3Joint_IsValid(m_JointId) && m_BuildFn)
    {
        m_JointId = m_BuildFn();
        if (b3Joint_IsValid(m_JointId))
        {
            b3Joint_SetUserData(m_JointId, this);
            ApplyConstraintTuning();
        }
    }
}

void Box3DPhysicsConstraint::ApplyConstraintTuning()
{
    // Break limits arrive in kilogram-force (game does lbs2kg); break force = rated mass * sim gravity, so a
    // weld rated for N kg holds an N-kg prop. 0 = never break, which Box3D's default FLT_MAX already means.
    Vector vecGravity;
    m_pEnvironment->GetGravity(&vecGravity);
    float flGravity = SourceToBox::Distance(vecGravity.Length());
    if (flGravity < 1e-3f)
        flGravity = 9.80665f; // standard g if the game hasn't set gravity yet

    if (m_BreakParams.forceLimit > 0.0f)
        b3Joint_SetForceThreshold(m_JointId, m_BreakParams.forceLimit * flGravity);
    if (m_BreakParams.torqueLimit > 0.0f)
        b3Joint_SetTorqueThreshold(m_JointId, m_BreakParams.torqueLimit * flGravity * InchesToMetres);

    // strength (0..1) softens the constraint below full stiffness; 1 keeps Box3D's default tuning.
    if (m_BreakParams.strength > 0.0f && m_BreakParams.strength < 0.999f)
    {
        float flHertz, flDamping;
        b3Joint_GetConstraintTuning(m_JointId, &flHertz, &flDamping);
        b3Joint_SetConstraintTuning(m_JointId, flHertz * m_BreakParams.strength, flDamping);
    }
}

void Box3DPhysicsConstraint::OnBroken()
{
    DestroyJoint();
    m_bBroken = true;
}

void Box3DPhysicsConstraint::SetupPulley(
    const b3Vec3 pulleyWorld[2], const b3Vec3 localAttach[2], float totalLength, float gearRatio, bool rigid)
{
    m_bPulley = true;
    m_PulleyWorld[0] = pulleyWorld[0];
    m_PulleyWorld[1] = pulleyWorld[1];
    m_PulleyLocal[0] = localAttach[0];
    m_PulleyLocal[1] = localAttach[1];
    m_flPulleyTotalLength = totalLength;
    m_flPulleyGearRatio = gearRatio > 1e-4f ? gearRatio : 1.0f;
    m_bPulleyRigid = rigid;
}

// No pulley joint in Box3D: solve lenA + gear*lenB = totalLength (rope may slack) as a post-step impulse
// constraint with the full effective mass, a few iterations, and a Baumgarte position bias.
void Box3DPhysicsConstraint::SolvePulley(float dt)
{
    if (!m_bPulley || !m_pReference || !m_pAttached || dt <= 0.0f)
        return;

    const b3BodyId ref = m_pReference->GetBodyID();
    const b3BodyId att = m_pAttached->GetBodyID();
    if (!b3Body_IsAwake(ref) && !b3Body_IsAwake(att))
        return; // both settled; don't keep them awake

    const b3WorldTransform xfRef = b3Body_GetTransform(ref);
    const b3WorldTransform xfAtt = b3Body_GetTransform(att);
    const b3Pos worldA = b3TransformWorldPoint(xfRef, m_PulleyLocal[0]);
    const b3Pos worldB = b3TransformWorldPoint(xfAtt, m_PulleyLocal[1]);

    const b3Vec3 dA = b3Sub(b3ToVec3(worldA), m_PulleyWorld[0]);
    const b3Vec3 dB = b3Sub(b3ToVec3(worldB), m_PulleyWorld[1]);
    const float lenA = b3Length(dA), lenB = b3Length(dB);
    if (lenA < 1e-6f || lenB < 1e-6f)
        return;
    const b3Vec3 uA = b3MulSV(1.0f / lenA, dA);
    const b3Vec3 uB = b3MulSV(1.0f / lenB, dB);

    const float gear = m_flPulleyGearRatio;
    const float C = lenA + gear * lenB - m_flPulleyTotalLength;
    if (!m_bPulleyRigid && C < 0.0f)
        return; // rope slack

    const b3Pos comA = b3TransformWorldPoint(xfRef, b3Body_GetMassData(ref).center);
    const b3Pos comB = b3TransformWorldPoint(xfAtt, b3Body_GetMassData(att).center);
    const b3Vec3 crossA = b3Cross(b3SubPos(worldA, comA), uA);
    const b3Vec3 crossB = b3Cross(b3SubPos(worldB, comB), uB);
    const b3Matrix3 invIA = b3Body_GetWorldInverseRotationalInertia(ref);
    const b3Matrix3 invIB = b3Body_GetWorldInverseRotationalInertia(att);
    const float kA = b3Body_GetInverseMass(ref) + b3Dot(crossA, b3MulMV(invIA, crossA));
    const float kB = b3Body_GetInverseMass(att) + b3Dot(crossB, b3MulMV(invIB, crossB));
    const float K = kA + gear * gear * kB;
    if (K <= 1e-9f)
        return; // both effectively immovable

    const float flClampC = SourceToBox::Distance(24.0f);
    const float bias = (0.2f / dt) * clamp(C, -flClampC, flClampC);

    for (int i = 0; i < 4; i++)
    {
        const b3Vec3 vA = b3Body_GetWorldPointVelocity(ref, worldA);
        const b3Vec3 vB = b3Body_GetWorldPointVelocity(att, worldB);
        const float Cdot = b3Dot(uA, vA) + gear * b3Dot(uB, vB);
        float impulse = -(Cdot + bias) / K;
        if (!m_bPulleyRigid && impulse > 0.0f)
            impulse = 0.0f; // a rope only pulls
        b3Body_ApplyLinearImpulse(ref, b3MulSV(impulse, uA), worldA, true);
        b3Body_ApplyLinearImpulse(att, b3MulSV(gear * impulse, uB), worldB, true);
    }
}

void Box3DPhysicsConstraint::Deactivate()
{
    DestroyJoint();
}

IPhysicsObject* Box3DPhysicsConstraint::GetReferenceObject() const
{
    return m_pReference;
}
IPhysicsObject* Box3DPhysicsConstraint::GetAttachedObject() const
{
    return m_pAttached;
}

bool Box3DPhysicsConstraint::NotifyObjectDestroyed(Box3DPhysicsObject* pObject)
{
    if (m_pReference != pObject && m_pAttached != pObject)
        return false;
    const bool bFireBroken = !m_bBroken; // report the break once, on the first constrained object to die
    DestroyJoint();
    m_bBroken = true;
    if (m_pReference == pObject)
        m_pReference = nullptr;
    if (m_pAttached == pObject)
        m_pAttached = nullptr;
    return bFireBroken;
}

void Box3DPhysicsConstraint::SetLinearMotor(float speed, float maxLinearImpulse)
{
    if (!b3Joint_IsValid(m_JointId) || b3Joint_GetType(m_JointId) != b3_prismaticJoint)
        return;

    b3PrismaticJoint_EnableMotor(m_JointId, speed != 0.0f);
    b3PrismaticJoint_SetMotorSpeed(m_JointId, SourceToBox::Distance(speed));
    b3PrismaticJoint_SetMaxMotorForce(m_JointId, fabsf(SourceToBox::Distance(maxLinearImpulse)));
}

void Box3DPhysicsConstraint::SetAngularMotor(float rotSpeed, float maxAngularImpulse)
{
    if (!b3Joint_IsValid(m_JointId))
        return;

    const float flSpeed = DEG2RAD(rotSpeed);
    const float flMaxTorque = fabsf(maxAngularImpulse);
    switch (b3Joint_GetType(m_JointId))
    {
        case b3_revoluteJoint:
            b3RevoluteJoint_EnableMotor(m_JointId, rotSpeed != 0.0f);
            b3RevoluteJoint_SetMotorSpeed(m_JointId, flSpeed);
            b3RevoluteJoint_SetMaxMotorTorque(m_JointId, flMaxTorque);
            break;
        case b3_sphericalJoint:
            b3SphericalJoint_EnableMotor(m_JointId, rotSpeed != 0.0f);
            b3SphericalJoint_SetMotorVelocity(m_JointId, b3Vec3{ flSpeed, flSpeed, flSpeed });
            b3SphericalJoint_SetMaxMotorTorque(m_JointId, flMaxTorque);
            break;
        default:
            break;
    }
}

bool Box3DPhysicsConstraint::GetConstraintTransform(
    matrix3x4_t* pConstraintToReference, matrix3x4_t* pConstraintToAttached) const
{
    if (m_pReference && pConstraintToReference)
        m_pReference->GetPositionMatrix(pConstraintToReference);
    if (m_pAttached && pConstraintToAttached)
        m_pAttached->GetPositionMatrix(pConstraintToAttached);
    return true;
}

bool Box3DPhysicsConstraint::GetConstraintParams(constraint_breakableparams_t* pParams) const
{
    if (pParams)
        *pParams = m_BreakParams;
    return true;
}

//-------------------------------------------------------------------------------------------------
// Box3DPhysicsConstraintGroup
//-------------------------------------------------------------------------------------------------

void Box3DPhysicsConstraintGroup::Activate()
{
    for (int i = 0; i < m_Constraints.Count(); i++)
        m_Constraints[i]->Activate();
}

//-------------------------------------------------------------------------------------------------
// Environment factories
//-------------------------------------------------------------------------------------------------

IPhysicsConstraint* Box3DPhysicsEnvironment::FinishConstraint(
    Box3DPhysicsConstraint* pConstraint, IPhysicsConstraintGroup* pGroup, const constraint_breakableparams_t& breakParams,
    const std::function<b3JointId()>& buildFn)
{
    pConstraint->SetBreakParams(breakParams);
    m_Constraints.AddToTail(pConstraint);
    if (Box3DPhysicsConstraintGroup* pGrp = static_cast<Box3DPhysicsConstraintGroup*>(pGroup))
    {
        pGrp->AddConstraint(pConstraint);
        pConstraint->SetGroup(pGrp);
    }
    // Grouped constraints stay dormant until the group's Activate().
    pConstraint->Init(buildFn, !pGroup && breakParams.isActive);
    return pConstraint;
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateFixedConstraint(
    IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
    const constraint_fixedparams_t& fixed)
{
    Box3DPhysicsObject* pRef = static_cast<Box3DPhysicsObject*>(pReferenceObject);
    Box3DPhysicsObject* pAtt = static_cast<Box3DPhysicsObject*>(pAttachedObject);
    const b3WorldId world = m_WorldId;
    const b3BodyId ref = pRef->GetBodyID(), att = pAtt->GetBodyID();

	// Weld at the game's designed attached->ref pose (attachedRefXform), not the live pose. frameA is the
    // reference origin; frameB is its inverse.
    const b3Transform relative = SourceToBox::Transform(fixed.attachedRefXform);
    b3Transform frameA = b3Transform_identity;
    b3Transform frameB = b3InvertTransform(relative);
    if (pRef->IsStatic() && !pAtt->IsStatic())
    {
        frameA = relative;
        frameB = b3Transform_identity;
    }

    auto build = [=]() {
        b3WeldJointDef def = b3DefaultWeldJointDef();
        def.base.bodyIdA = ref;
        def.base.bodyIdB = att;
        def.base.localFrameA = frameA;
        def.base.localFrameB = frameB;
        return b3CreateWeldJoint(world, &def);
    };
    Box3DPhysicsConstraint* pConstraint = new Box3DPhysicsConstraint(this, pRef, pAtt);
    pConstraint->SetSaveInfo(kBox3DConstraint_Fixed, &fixed, sizeof(fixed));
    return FinishConstraint(pConstraint, pGroup, fixed.constraint, build);
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateHingeConstraint(
    IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
    const constraint_hingeparams_t& hinge)
{
    Box3DPhysicsObject* pRef = static_cast<Box3DPhysicsObject*>(pReferenceObject);
    Box3DPhysicsObject* pAtt = static_cast<Box3DPhysicsObject*>(pAttachedObject);
    const b3WorldId world = m_WorldId;
    const b3BodyId ref = pRef->GetBodyID(), att = pAtt->GetBodyID();

    const b3Vec3 worldPos = SourceToBox::Distance(hinge.worldPosition);
    const b3Vec3 worldAxis = SourceToBox::Unitless(hinge.worldAxisDirection);
    b3Transform frameA, frameB;
    frameA.p = WorldToLocalPoint(ref, worldPos);
    frameA.q = LocalFrameForAxis(ref, b3Vec3_axisZ, worldAxis);
    frameB.p = WorldToLocalPoint(att, worldPos);
    frameB.q = LocalFrameForAxis(att, b3Vec3_axisZ, worldAxis);

    const bool bLimit = hinge.hingeAxis.minRotation != hinge.hingeAxis.maxRotation;
    const float flLower = ClampAngle(DEG2RAD(-hinge.hingeAxis.maxRotation), 0.99f * M_PI_F);
    const float flUpper = ClampAngle(DEG2RAD(-hinge.hingeAxis.minRotation), 0.99f * M_PI_F);

    // hingeAxis.torque is friction (velocity 0) or a motor; it's HL units (kg*in^2/s^2), so scale to N-m by
    // (in/m)^2 or the hinge barely swings. Negate the speed to match the limits' clockwise flip.
    const bool bMotor = hinge.hingeAxis.angularVelocity != 0.0f || hinge.hingeAxis.torque != 0.0f;
    const float flMotorSpeed = DEG2RAD(-hinge.hingeAxis.angularVelocity);
    const float flMaxTorque = fabsf(hinge.hingeAxis.torque) * (InchesToMetres * InchesToMetres);

    auto build = [=]() {
        b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
        def.base.bodyIdA = ref;
        def.base.bodyIdB = att;
        def.base.localFrameA = frameA;
        def.base.localFrameB = frameB;
        if (bLimit)
        {
            def.enableLimit = true;
            def.lowerAngle = flLower;
            def.upperAngle = flUpper;
        }
        if (bMotor)
        {
            def.enableMotor = true;
            def.motorSpeed = flMotorSpeed;
            def.maxMotorTorque = flMaxTorque;
        }
        return b3CreateRevoluteJoint(world, &def);
    };
    Box3DPhysicsConstraint* pConstraint = new Box3DPhysicsConstraint(this, pRef, pAtt);
    pConstraint->SetSaveInfo(kBox3DConstraint_Hinge, &hinge, sizeof(hinge));
    return FinishConstraint(pConstraint, pGroup, hinge.constraint, build);
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateBallsocketConstraint(
    IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
    const constraint_ballsocketparams_t& ballsocket)
{
    Box3DPhysicsObject* pRef = static_cast<Box3DPhysicsObject*>(pReferenceObject);
    Box3DPhysicsObject* pAtt = static_cast<Box3DPhysicsObject*>(pAttachedObject);
    const b3WorldId world = m_WorldId;
    const b3BodyId ref = pRef->GetBodyID(), att = pAtt->GetBodyID();

    // constraintPosition is already object-local; convert to metres.
    const b3Vec3 posA = SourceToBox::Distance(ballsocket.constraintPosition[0]);
    const b3Vec3 posB = SourceToBox::Distance(ballsocket.constraintPosition[1]);

    auto build = [=]() {
        b3SphericalJointDef def = b3DefaultSphericalJointDef();
        def.base.bodyIdA = ref;
        def.base.bodyIdB = att;
        def.base.localFrameA.p = posA;
        def.base.localFrameB.p = posB;
        return b3CreateSphericalJoint(world, &def);
    };
    Box3DPhysicsConstraint* pConstraint = new Box3DPhysicsConstraint(this, pRef, pAtt);
    pConstraint->SetSaveInfo(kBox3DConstraint_Ballsocket, &ballsocket, sizeof(ballsocket));
    return FinishConstraint(pConstraint, pGroup, ballsocket.constraint, build);
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateSlidingConstraint(
    IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
    const constraint_slidingparams_t& sliding)
{
    Box3DPhysicsObject* pRef = static_cast<Box3DPhysicsObject*>(pReferenceObject);
    Box3DPhysicsObject* pAtt = static_cast<Box3DPhysicsObject*>(pAttachedObject);
    const b3WorldId world = m_WorldId;
    const b3BodyId ref = pRef->GetBodyID(), att = pAtt->GetBodyID();

    // Build from the designed attachedRefXform (not the live pose); slideAxisRef is reference-local, and the
    // prismatic frees only frame X.
    const b3Transform xAttToRef = SourceToBox::Transform(sliding.attachedRefXform);
    const b3Vec3 slideAxis = SafeNormalize(SourceToBox::Unitless(sliding.slideAxisRef));
    b3Transform frameA;
    frameA.p = xAttToRef.p; // attached origin expressed in reference space = the slide anchor
    frameA.q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisX, slideAxis);
    const b3Transform frameB = b3InvMulTransforms(xAttToRef, frameA);

    const bool bLimit = sliding.limitMin != sliding.limitMax;
    const float flLo = SourceToBox::Distance(sliding.limitMin);
    const float flHi = SourceToBox::Distance(sliding.limitMax);
    const bool bMotor = sliding.friction != 0.0f || sliding.velocity != 0.0f;
    const float flMotorSpeed = SourceToBox::Distance(sliding.velocity);
    const float flMaxForce = sliding.friction;

    auto build = [=]() {
        b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
        def.base.bodyIdA = ref;
        def.base.bodyIdB = att;
        def.base.localFrameA = frameA;
        def.base.localFrameB = frameB;
        if (bLimit)
        {
            def.enableLimit = true;
            def.lowerTranslation = flLo;
            def.upperTranslation = flHi;
        }
        if (bMotor)
        {
            def.enableMotor = true;
            def.motorSpeed = flMotorSpeed;
            def.maxMotorForce = flMaxForce;
        }
        return b3CreatePrismaticJoint(world, &def);
    };
    Box3DPhysicsConstraint* pConstraint = new Box3DPhysicsConstraint(this, pRef, pAtt);
    pConstraint->SetSaveInfo(kBox3DConstraint_Sliding, &sliding, sizeof(sliding));
    return FinishConstraint(pConstraint, pGroup, sliding.constraint, build);
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateLengthConstraint(
    IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
    const constraint_lengthparams_t& length)
{
    Box3DPhysicsObject* pRef = static_cast<Box3DPhysicsObject*>(pReferenceObject);
    Box3DPhysicsObject* pAtt = static_cast<Box3DPhysicsObject*>(pAttachedObject);
    const b3WorldId world = m_WorldId;
    const b3BodyId ref = pRef->GetBodyID(), att = pAtt->GetBodyID();

    const b3Vec3 posA = SourceToBox::Distance(length.objectPosition[0]);
    const b3Vec3 posB = SourceToBox::Distance(length.objectPosition[1]);
    const float flTotal = SourceToBox::Distance(length.totalLength);
    const float flMin = SourceToBox::Distance(length.minLength);
    const bool bRigid = length.minLength >= length.totalLength;

    auto build = [=]() {
        b3DistanceJointDef def = b3DefaultDistanceJointDef();
        def.base.bodyIdA = ref;
        def.base.bodyIdB = att;
        def.base.localFrameA.p = posA;
        def.base.localFrameB.p = posB;
        def.length = flTotal;
        if (bRigid)
        {
            def.enableSpring = false; // rigid rod at totalLength
        }
        else
        {
            // Rope: no spring pull (hertz 0), hard stop at [minLength, totalLength].
            def.enableSpring = true;
            def.hertz = 0.0f;
            def.dampingRatio = 0.0f;
            def.enableLimit = true;
            def.minLength = flMin;
            def.maxLength = flTotal;
        }
        return b3CreateDistanceJoint(world, &def);
    };
    Box3DPhysicsConstraint* pConstraint = new Box3DPhysicsConstraint(this, pRef, pAtt);
    pConstraint->SetSaveInfo(kBox3DConstraint_Length, &length, sizeof(length));
    return FinishConstraint(pConstraint, pGroup, length.constraint, build);
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreateRagdollConstraint(
    IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
    const constraint_ragdollparams_t& ragdoll)
{
    Box3DPhysicsObject* pRef = static_cast<Box3DPhysicsObject*>(pReferenceObject);
    Box3DPhysicsObject* pAtt = static_cast<Box3DPhysicsObject*>(pAttachedObject);
    const b3WorldId world = m_WorldId;
    const b3BodyId ref = pRef->GetBodyID(), att = pAtt->GetBodyID();

    const b3Transform frameRef = SourceToBox::Transform(ragdoll.constraintToReference);
    const b3Transform frameAtt = SourceToBox::Transform(ragdoll.constraintToAttached);

    // Per-axis limits (radians) with Source's clockwise flip; an axis is a DOF if its range exceeds 5deg.
    // Axis 0 is twist, 1/2 are swing.
    float flMin[3], flMax[3];
    int nDOF = 0, nDofAxis = 0;
    for (int i = 0; i < 3; i++)
    {
        if (ragdoll.useClockwiseRotations)
        {
            flMin[i] = DEG2RAD(-ragdoll.axes[i].maxRotation);
            flMax[i] = DEG2RAD(-ragdoll.axes[i].minRotation);
        }
        else
        {
            flMin[i] = DEG2RAD(ragdoll.axes[i].minRotation);
            flMax[i] = DEG2RAD(ragdoll.axes[i].maxRotation);
        }
        if (flMax[i] - flMin[i] > DEG2RAD(5.0f))
        {
            nDOF++;
            nDofAxis = i;
        }
    }

    const float flLimit = 0.99f * M_PI_F;
    const float flCone = clamp(Max(0.5f * (flMax[1] - flMin[1]), 0.5f * (flMax[2] - flMin[2])), 0.0f, M_PI_F);
    // One isotropic motor torque, so use the strongest axis (the average dilutes a 1-DOF joint); axis torque
    // is HL units (kg*in^2/s^2) -> N-m by (in/m)^2.
    const float flRawTorque = Max(ragdoll.axes[0].torque, Max(ragdoll.axes[1].torque, ragdoll.axes[2].torque));
    const float flFriction = Max(0.05f, flRawTorque * (InchesToMetres * InchesToMetres));

    auto build = [=]() -> b3JointId {
        if (nDOF == 0)
        {
            b3WeldJointDef def = b3DefaultWeldJointDef();
            def.base.bodyIdA = ref;
            def.base.bodyIdB = att;
            def.base.localFrameA = frameRef;
            def.base.localFrameB = frameAtt;
            return b3CreateWeldJoint(world, &def);
        }
        if (nDOF == 1)
        {
            // One hinge axis: rotate the frame so the revolute Z sits on that constraint axis.
            const b3Vec3 axes[3] = { b3Vec3_axisX, b3Vec3{ 0.0f, 1.0f, 0.0f }, b3Vec3_axisZ };
            const b3Quat qRemap = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, axes[nDofAxis]);
            b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
            def.base.bodyIdA = ref;
            def.base.bodyIdB = att;
            def.base.localFrameA.p = frameRef.p;
            def.base.localFrameA.q = b3MulQuat(frameRef.q, qRemap);
            def.base.localFrameB.p = frameAtt.p;
            def.base.localFrameB.q = b3MulQuat(frameAtt.q, qRemap);
            def.enableLimit = true;
            def.lowerAngle = ClampAngle(flMin[nDofAxis], flLimit);
            def.upperAngle = ClampAngle(flMax[nDofAxis], flLimit);
            def.enableMotor = true;
            def.maxMotorTorque = flFriction;
            return b3CreateRevoluteJoint(world, &def);
        }

        // 2+ DOF: spherical swing cone + twist. Box3D's cone/twist axis is frame Z; Source twist is the
        // constraint X axis, so rotate the frame to put Z there.
        const b3Quat qZtoX = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, b3Vec3_axisX);
        b3SphericalJointDef def = b3DefaultSphericalJointDef();
        def.base.bodyIdA = ref;
        def.base.bodyIdB = att;
        def.base.localFrameA.p = frameRef.p;
        def.base.localFrameA.q = b3MulQuat(frameRef.q, qZtoX);
        def.base.localFrameB.p = frameAtt.p;
        def.base.localFrameB.q = b3MulQuat(frameAtt.q, qZtoX);
        def.enableConeLimit = true;
        def.coneAngle = flCone;
        def.enableTwistLimit = true;
        def.lowerTwistAngle = ClampAngle(flMin[0], flLimit);
        def.upperTwistAngle = ClampAngle(flMax[0], flLimit);
        def.enableMotor = true;
        def.maxMotorTorque = flFriction;
        return b3CreateSphericalJoint(world, &def);
    };
    Box3DPhysicsConstraint* pConstraint = new Box3DPhysicsConstraint(this, pRef, pAtt);
    pConstraint->SetSaveInfo(kBox3DConstraint_Ragdoll, &ragdoll, sizeof(ragdoll));
    return FinishConstraint(pConstraint, pGroup, ragdoll.constraint, build);
}

IPhysicsConstraint* Box3DPhysicsEnvironment::CreatePulleyConstraint(
    IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup,
    const constraint_pulleyparams_t& pulley)
{
    Box3DPhysicsObject* pRef = static_cast<Box3DPhysicsObject*>(pReferenceObject);
    Box3DPhysicsObject* pAtt = static_cast<Box3DPhysicsObject*>(pAttachedObject);
    Box3DPhysicsConstraint* pConstraint = new Box3DPhysicsConstraint(this, pRef, pAtt);

    // pulleyPosition is world space; objectPosition is object-local. Box3D has no pulley joint, so this is
    // solved each step (SolvePulleys) instead of via a b3Joint builder.
    const b3Vec3 pulleyWorld[2] = { SourceToBox::Distance(pulley.pulleyPosition[0]),
                                    SourceToBox::Distance(pulley.pulleyPosition[1]) };
    const b3Vec3 localAttach[2] = { SourceToBox::Distance(pulley.objectPosition[0]),
                                    SourceToBox::Distance(pulley.objectPosition[1]) };
    pConstraint->SetupPulley(
        pulleyWorld, localAttach, SourceToBox::Distance(pulley.totalLength), pulley.gearRatio, pulley.isRigid);
    pConstraint->SetSaveInfo(kBox3DConstraint_Pulley, &pulley, sizeof(pulley));
    m_Pulleys.AddToTail(pConstraint);

    return FinishConstraint(pConstraint, pGroup, pulley.constraint, std::function<b3JointId()>());
}

void Box3DPhysicsEnvironment::DestroyConstraint(IPhysicsConstraint* pConstraint)
{
    if (!pConstraint)
        return;
    Box3DPhysicsConstraint* pBoxConstraint = static_cast<Box3DPhysicsConstraint*>(pConstraint);
    m_Constraints.FindAndRemove(pBoxConstraint);
    m_Pulleys.FindAndRemove(pBoxConstraint);
    delete pBoxConstraint;
}

void Box3DPhysicsEnvironment::SolvePulleys(float dt)
{
    for (int i = 0; i < m_Pulleys.Count(); i++)
        m_Pulleys[i]->SolvePulley(dt);
}

IPhysicsConstraintGroup* Box3DPhysicsEnvironment::CreateConstraintGroup(const constraint_groupparams_t& groupParams)
{
    Box3DPhysicsConstraintGroup* pGroup = new Box3DPhysicsConstraintGroup;
    pGroup->SetErrorParams(groupParams);
    return pGroup;
}

void Box3DPhysicsEnvironment::DestroyConstraintGroup(IPhysicsConstraintGroup* pGroup)
{
    delete static_cast<Box3DPhysicsConstraintGroup*>(pGroup);
}

//-------------------------------------------------------------------------------------------------
// Box3DPhysicsSpring
//-------------------------------------------------------------------------------------------------

Box3DPhysicsSpring::Box3DPhysicsSpring(
    Box3DPhysicsEnvironment* pEnvironment, Box3DPhysicsObject* pStart, Box3DPhysicsObject* pEnd, const springparams_t* pParams)
    : m_pEnvironment(pEnvironment)
    , m_pStart(pStart)
    , m_pEnd(pEnd)
    , m_flConstant(pParams->constant)
    , m_flDamping(pParams->damping)
{
    const b3BodyId a = pStart->GetBodyID(), b = pEnd->GetBodyID();
    if (pParams->useLocalPositions)
    {
        m_AnchorStart = SourceToBox::Distance(pParams->startPosition);
        m_AnchorEnd = SourceToBox::Distance(pParams->endPosition);
    }
    else
    {
        m_AnchorStart = WorldToLocalPoint(a, SourceToBox::Distance(pParams->startPosition));
        m_AnchorEnd = WorldToLocalPoint(b, SourceToBox::Distance(pParams->endPosition));
    }

    b3DistanceJointDef def = b3DefaultDistanceJointDef();
    def.base.bodyIdA = a;
    def.base.bodyIdB = b;
    def.base.localFrameA.p = m_AnchorStart;
    def.base.localFrameB.p = m_AnchorEnd;
    def.length = SourceToBox::Distance(pParams->naturalLength);
    def.enableSpring = true;
    m_JointId = b3CreateDistanceJoint(m_pEnvironment->GetWorldId(), &def);
    PushSpringSettings();
}

Box3DPhysicsSpring::~Box3DPhysicsSpring()
{
    if (b3Joint_IsValid(m_JointId))
        b3DestroyJoint(m_JointId, true);
}

// Box3D springs are frequency-based: convert Source's stiffness/damping to hertz + ratio via the
// effective mass (hertz = sqrt(k/m)/2pi).
void Box3DPhysicsSpring::PushSpringSettings()
{
    if (!b3Joint_IsValid(m_JointId) || !m_pStart || !m_pEnd)
        return;
    const float flInvSum = b3Body_GetInverseMass(m_pStart->GetBodyID()) + b3Body_GetInverseMass(m_pEnd->GetBodyID());
    const float flMass = flInvSum > 1e-9f ? 1.0f / flInvSum : 1.0f;
    const float flK = Max(m_flConstant, 0.0f);
    const float flHertz = flK > 0.0f ? sqrtf(flK / flMass) / (2.0f * M_PI_F) : 0.0f;
    const float flDampingRatio = flK > 0.0f ? m_flDamping / (2.0f * sqrtf(flK * flMass)) : 1.0f;
    b3DistanceJoint_SetSpringHertz(m_JointId, flHertz);
    b3DistanceJoint_SetSpringDampingRatio(m_JointId, flDampingRatio);
}

void Box3DPhysicsSpring::GetEndpoints(Vector* worldPositionStart, Vector* worldPositionEnd)
{
    if (worldPositionStart)
    {
        const b3WorldTransform wt = b3Body_GetTransform(m_pStart->GetBodyID());
        *worldPositionStart = BoxToSource::Distance(b3Add(b3ToVec3(wt.p), b3RotateVector(wt.q, m_AnchorStart)));
    }
    if (worldPositionEnd)
    {
        const b3WorldTransform wt = b3Body_GetTransform(m_pEnd->GetBodyID());
        *worldPositionEnd = BoxToSource::Distance(b3Add(b3ToVec3(wt.p), b3RotateVector(wt.q, m_AnchorEnd)));
    }
}

void Box3DPhysicsSpring::SetSpringConstant(float flSpringConstant)
{
    m_flConstant = flSpringConstant;
    PushSpringSettings();
    if (m_pStart)
        m_pStart->Wake();
    if (m_pEnd)
        m_pEnd->Wake();
}

void Box3DPhysicsSpring::SetSpringDamping(float flSpringDamping)
{
    m_flDamping = flSpringDamping;
    PushSpringSettings();
    if (m_pStart)
        m_pStart->Wake();
    if (m_pEnd)
        m_pEnd->Wake();
}

void Box3DPhysicsSpring::SetSpringLength(float flSpringLength)
{
    if (b3Joint_IsValid(m_JointId))
        b3DistanceJoint_SetLength(m_JointId, SourceToBox::Distance(flSpringLength));
    if (m_pStart)
        m_pStart->Wake();
    if (m_pEnd)
        m_pEnd->Wake();
}

IPhysicsObject* Box3DPhysicsSpring::GetStartObject()
{
    return m_pStart;
}
IPhysicsObject* Box3DPhysicsSpring::GetEndObject()
{
    return m_pEnd;
}

void Box3DPhysicsSpring::NotifyObjectDestroyed(Box3DPhysicsObject* pObject)
{
    if (m_pStart != pObject && m_pEnd != pObject)
        return;
    if (b3Joint_IsValid(m_JointId))
        b3DestroyJoint(m_JointId, true);
    m_JointId = b3_nullJointId;
    if (m_pStart == pObject)
        m_pStart = nullptr;
    if (m_pEnd == pObject)
        m_pEnd = nullptr;
}

IPhysicsSpring* Box3DPhysicsEnvironment::CreateSpring(
    IPhysicsObject* pObjectStart, IPhysicsObject* pObjectEnd, springparams_t* pParams)
{
    if (!pObjectStart || !pObjectEnd || !pParams)
        return nullptr;
    Box3DPhysicsSpring* pSpring = new Box3DPhysicsSpring(
        this, static_cast<Box3DPhysicsObject*>(pObjectStart), static_cast<Box3DPhysicsObject*>(pObjectEnd), pParams);
    pSpring->SetSaveParams(*pParams);
    m_Springs.AddToTail(pSpring);
    return pSpring;
}

void Box3DPhysicsEnvironment::DestroySpring(IPhysicsSpring* pSpring)
{
    if (!pSpring)
        return;
    Box3DPhysicsSpring* pBoxSpring = static_cast<Box3DPhysicsSpring*>(pSpring);
    m_Springs.FindAndRemove(pBoxSpring);
    delete pBoxSpring;
}
