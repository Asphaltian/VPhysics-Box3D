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

void Box3DPhysicsConstraint::SetupAngularLimits(
    const b3Transform& frameRef, const b3Transform& frameAtt, bool bCone, float coneAngle, bool bTwist, float twistMin,
    float twistMax, float flFriction, bool bHasJoint)
{
    m_bAngularLimits = true;
    m_AngFrameRef = frameRef.q;
    m_AngFrameAtt = frameAtt.q;
    m_AngAnchorRef = frameRef.p;
    m_AngAnchorAtt = frameAtt.p;
    m_bAngHasJoint = bHasJoint;
    m_bAngCone = bCone;
    m_flAngCone = coneAngle;
    m_bAngTwist = bTwist;
    m_flAngTwistMin = twistMin;
    m_flAngTwistMax = twistMax;
    m_flAngFriction = flFriction;
}

static b3Vec3 AnchorRelativeVelocity(b3BodyId ref, b3BodyId att, const b3Vec3& anchorRefLocal, const b3Vec3& anchorAttLocal)
{
    const b3Pos worldRef = b3TransformWorldPoint(b3Body_GetTransform(ref), anchorRefLocal);
    const b3Pos worldAtt = b3TransformWorldPoint(b3Body_GetTransform(att), anchorAttLocal);
    return b3Sub(b3Body_GetWorldPointVelocity(att, worldAtt), b3Body_GetWorldPointVelocity(ref, worldRef));
}

// Restore the anchor velocity to its pre-impulse value so angular corrections become rotation about
// the joint instead of being cancelled by the island's point-lock (Havana solves these jointly).
static void RepinAnchorVelocity(
    b3BodyId ref, b3BodyId att, const b3Vec3& anchorRefLocal, const b3Vec3& anchorAttLocal, const b3Vec3& vRelTarget)
{
    const b3WorldTransform xfRef = b3Body_GetTransform(ref);
    const b3WorldTransform xfAtt = b3Body_GetTransform(att);
    const b3Pos worldRef = b3TransformWorldPoint(xfRef, anchorRefLocal);
    const b3Pos worldAtt = b3TransformWorldPoint(xfAtt, anchorAttLocal);
    const b3Matrix3 invIRef = b3Body_GetWorldInverseRotationalInertia(ref);
    const b3Matrix3 invIAtt = b3Body_GetWorldInverseRotationalInertia(att);
    const b3Vec3 rRef = b3SubPos(worldRef, b3Body_GetWorldCenter(ref));
    const b3Vec3 rAtt = b3SubPos(worldAtt, b3Body_GetWorldCenter(att));
    const float flInvMass = b3Body_GetInverseMass(ref) + b3Body_GetInverseMass(att);

    for (int i = 0; i < 4; i++)
    {
        const b3Vec3 vRel = b3Sub(
            b3Sub(b3Body_GetWorldPointVelocity(att, worldAtt), b3Body_GetWorldPointVelocity(ref, worldRef)), vRelTarget);
        const float flLen = b3Length(vRel);
        if (flLen < 1e-4f)
            break;
        const b3Vec3 dir = b3MulSV(1.0f / flLen, vRel);
        const b3Vec3 crossRef = b3Cross(rRef, dir);
        const b3Vec3 crossAtt = b3Cross(rAtt, dir);
        const float flK = flInvMass + b3Dot(crossRef, b3MulMV(invIRef, crossRef)) + b3Dot(crossAtt, b3MulMV(invIAtt, crossAtt));
        if (flK <= 1e-9f)
            break;
        const b3Vec3 impulse = b3MulSV(-flLen / flK, dir);
        b3Body_ApplyLinearImpulse(att, impulse, worldAtt, false);
        b3Body_ApplyLinearImpulse(ref, b3MulSV(-1.0f, impulse), worldRef, false);
    }
}

// Havana joint friction: a position servo on a reference angle that slips when the clamp engages.
static float SolveAngularFrictionImpulse(
    b3BodyId ref, b3BodyId att, const b3Vec3& axis, float flAlpha, float flRefPos, float flFriction, float dt, bool& bApplied)
{
    const b3Matrix3 invIRef = b3Body_GetWorldInverseRotationalInertia(ref);
    const b3Matrix3 invIAtt = b3Body_GetWorldInverseRotationalInertia(att);
    const float flK = b3Dot(axis, b3MulMV(invIRef, axis)) + b3Dot(axis, b3MulMV(invIAtt, axis));
    if (flK <= 1e-9f)
        return flRefPos;

    const float flVel = b3Dot(axis, b3Sub(b3Body_GetAngularVelocity(att), b3Body_GetAngularVelocity(ref)));
    const float flDAlpha = flRefPos - flAlpha;
    const float flCorrection = flDAlpha * 0.8f / dt - flVel; // friction_tau 0.8, friction_damp 1.0
    // Deadband: sub-threshold corrections ring forever and reset b3's velocity-based sleep timer.
    if (fabsf(flCorrection) < 0.03f)
        return flRefPos;
    float flImpulse = flCorrection / flK;

    const float flMaxImpulse = flFriction * dt;
    if (fabsf(flImpulse) > flMaxImpulse)
    {
        const float flFactor = flMaxImpulse / fabsf(flImpulse);
        flImpulse *= flFactor;
        flRefPos -= (1.0f - flFactor) * flDAlpha; // slip toward the actual angle
    }

    if (fabsf(flImpulse) < 1e-6f)
        return flRefPos;

    bApplied = true;
    b3Body_ApplyAngularImpulse(att, b3MulSV(flImpulse, axis), false);
    b3Body_ApplyAngularImpulse(ref, b3MulSV(-flImpulse, axis), false);
    return flRefPos;
}

// Havana limit: one predictive impulse, ((alpha + vel*dt) - limit) / (dt * K); never brakes inward motion.
static void SolveAngularLimitImpulse(
    b3BodyId ref, b3BodyId att, const b3Vec3& axis, float flAlpha, float flMin, float flMax, float flTau, float dt,
    bool& bApplied)
{
    const b3Matrix3 invIRef = b3Body_GetWorldInverseRotationalInertia(ref);
    const b3Matrix3 invIAtt = b3Body_GetWorldInverseRotationalInertia(att);
    const float flK = b3Dot(axis, b3MulMV(invIRef, axis)) + b3Dot(axis, b3MulMV(invIAtt, axis));
    if (flK <= 1e-9f)
        return;

    // Slop: resting violations settle inside it with zero impulse, or piles never sleep.
    const float flSlop = 0.015f;
    const float flVel = b3Dot(axis, b3Sub(b3Body_GetAngularVelocity(att), b3Body_GetAngularVelocity(ref)));
    const float flNext = flAlpha + flVel * dt;
    float flImpulse = 0.0f; // about +axis
    if (flNext > flMax + flSlop)
        flImpulse = -flTau * (flNext - (flMax + flSlop)) / (dt * flK);
    else if (flNext < flMin - flSlop)
        flImpulse = -flTau * (flNext - (flMin - flSlop)) / (dt * flK);
    if (fabsf(flImpulse) > 1e-6f)
    {
        bApplied = true;
        b3Body_ApplyAngularImpulse(att, b3MulSV(flImpulse, axis), false);
        b3Body_ApplyAngularImpulse(ref, b3MulSV(-flImpulse, axis), false);
    }
}

// Friction servo first (once per tick), then limits; twist unwrapped so forcing past pi never flips.
bool Box3DPhysicsConstraint::SolveAngularLimits(float dt, bool bApplyFriction)
{
    if (!m_bAngularLimits || m_bBroken || !m_pReference || !m_pAttached || dt <= 0.0f)
        return false;

    const b3BodyId ref = m_pReference->GetBodyID();
    const b3BodyId att = m_pAttached->GetBodyID();
    if (!b3Body_IsAwake(ref) && !b3Body_IsAwake(att))
        return false;

    bool bApplied = false;

    const b3Quat qRef = b3MulQuat(BodyRotation(ref), m_AngFrameRef);
    const b3Quat qAtt = b3MulQuat(BodyRotation(att), m_AngFrameAtt);
    const b3Vec3 xRef = b3RotateVector(qRef, b3Vec3_axisX); // twist axes
    const b3Vec3 xAtt = b3RotateVector(qAtt, b3Vec3_axisX);

    const bool bFriction = bApplyFriction && m_flAngFriction > 0.0f;

    const b3Vec3 vAnchorBefore = m_bAngHasJoint ? AnchorRelativeVelocity(ref, att, m_AngAnchorRef, m_AngAnchorAtt) : b3Vec3{};

    if (m_bAngCone)
    {
        // Swing: angle between the twist axes, growing about +cross(xRef, xAtt). Cone is a max-only limit.
        const b3Vec3 crossRA = b3Cross(xRef, xAtt);
        const float flCrossLen = b3Length(crossRA);
        if (flCrossLen > 1e-6f)
        {
            const b3Vec3 axis = b3MulSV(1.0f / flCrossLen, crossRA);
            const float flSwing = atan2f(flCrossLen, b3Dot(xAtt, xRef));
            if (bFriction)
            {
                if (!m_bFrictionInit)
                    m_flFrictionRefSwing = flSwing;
                m_flFrictionRefSwing = SolveAngularFrictionImpulse(
                    ref, att, axis, flSwing, m_flFrictionRefSwing, m_flAngFriction, dt, bApplied);
            }
            SolveAngularLimitImpulse(ref, att, axis, flSwing, -M_PI_F, m_flAngCone, 1.0f, dt, bApplied);
        }
    }

    // Twist about the bisector with tau = cos(swing/2), fading out at the pole (Havana ragdoll twist).
    const b3Vec3 axisSum = b3Add(xRef, xAtt);
    const float flSumLenSq = b3Dot(axisSum, axisSum);
    if (m_bAngTwist && flSumLenSq > 1e-4f)
    {
        b3Quat qRel = b3InvMulQuat(qRef, qAtt);
        if (qRel.s < 0.0f)
        {
            // Canonicalize: q and -q are the same rotation, and 2*atan2 spans (-2pi, 2pi] otherwise.
            qRel.s = -qRel.s;
            qRel.v = b3MulSV(-1.0f, qRel.v);
        }
        const float flRaw = 2.0f * atan2f(qRel.v.x, qRel.s); // wrapped [-pi, pi]
        if (!m_bTwistInit)
        {
            m_bTwistInit = true;
            m_flTwistUnwrapped = flRaw;
        }
        else
        {
            float flDelta = flRaw - m_flTwistLastRaw;
            if (flDelta > M_PI_F)
                flDelta -= 2.0f * M_PI_F;
            else if (flDelta < -M_PI_F)
                flDelta += 2.0f * M_PI_F;
            m_flTwistUnwrapped += flDelta;
        }
        m_flTwistLastRaw = flRaw;

        const float flSumLen = sqrtf(flSumLenSq);
        const b3Vec3 twistAxis = b3MulSV(1.0f / flSumLen, axisSum);
        const float flTwistTau = 0.5f * flSumLen; // cos(swing/2)
        if (bFriction)
        {
            if (!m_bFrictionInit)
                m_flFrictionRefTwist = m_flTwistUnwrapped;
            m_flFrictionRefTwist = SolveAngularFrictionImpulse(
                ref, att, twistAxis, m_flTwistUnwrapped, m_flFrictionRefTwist, m_flAngFriction, dt, bApplied);
        }
        SolveAngularLimitImpulse(
            ref, att, twistAxis, m_flTwistUnwrapped, m_flAngTwistMin, m_flAngTwistMax, flTwistTau, dt, bApplied);
    }

    if (bFriction)
        m_bFrictionInit = true;

    if (bApplied && m_bAngHasJoint)
        RepinAnchorVelocity(ref, att, m_AngAnchorRef, m_AngAnchorAtt, vAnchorBefore);
    return bApplied;
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
    // Retail: update_friction(ConvertAngleToIVP(f)), speed-0 path only.
    if (m_bAngularLimits)
    {
        if (rotSpeed == 0.0f)
            m_flAngFriction = DEG2RAD(fabsf(maxAngularImpulse));
        return;
    }

    if (!b3Joint_IsValid(m_JointId))
        return;

    switch (b3Joint_GetType(m_JointId))
    {
        case b3_revoluteJoint:
            // Speed 0 with torque is a friction brake; speed negated for the clockwise flip.
            b3RevoluteJoint_EnableMotor(m_JointId, maxAngularImpulse != 0.0f);
            b3RevoluteJoint_SetMotorSpeed(m_JointId, DEG2RAD(-rotSpeed));
            b3RevoluteJoint_SetMaxMotorTorque(m_JointId, fabsf(DEG2RAD(maxAngularImpulse)));
            break;
        case b3_sphericalJoint:
            // Retail only updates ragdoll friction here, and only for speed 0.
            if (rotSpeed == 0.0f)
            {
                b3SphericalJoint_EnableMotor(m_JointId, maxAngularImpulse != 0.0f);
                b3SphericalJoint_SetMotorVelocity(m_JointId, b3Vec3{ 0.0f, 0.0f, 0.0f });
                b3SphericalJoint_SetMaxMotorTorque(m_JointId, fabsf(DEG2RAD(maxAngularImpulse)));
            }
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

    // Clockwise flip; a DOF when min != max (retail), FREE when full circle. Axis 0 twist, 1/2 swing.
    float flMin[3], flMax[3];
    bool bFree[3];
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
        bFree[i] = (ragdoll.axes[i].maxRotation - ragdoll.axes[i].minRotation) >= 359.0f;
        if (ragdoll.axes[i].minRotation != ragdoll.axes[i].maxRotation)
        {
            nDOF++;
            nDofAxis = i;
        }
    }

    const float flSwing1 = bFree[1] ? M_PI_F : 0.5f * (flMax[1] - flMin[1]);
    const float flSwing2 = bFree[2] ? M_PI_F : 0.5f * (flMax[2] - flMin[2]);
    const float flCone = clamp(Max(flSwing1, flSwing2), 0.0f, M_PI_F);
    // Strongest axis (average dilutes 1-DOF joints); retail scales friction by reference mass.
    const float flRawTorque = Max(ragdoll.axes[0].torque, Max(ragdoll.axes[1].torque, ragdoll.axes[2].torque));
    const float flFriction = flRawTorque * pRef->GetMass();

    // Retail gates activation on ragdoll.isActive, not constraint.isActive.
    constraint_breakableparams_t breakParams = ragdoll.constraint;
    breakParams.isActive = ragdoll.isActive;

    const bool bCone = flCone < M_PI_F - 1e-3f;

    // onlyAngularLimits (AdvBallsocket onlyrotation): rotation-only, translation stays free. Rigid
    // swing + free twist = parallel joint; all axes rigid = motor joint (angular spring only); real
    // cones/twist limits = no joint, per-step limit solver.
    if (ragdoll.onlyAngularLimits)
    {
        const bool bTwistFree = bFree[0];
        const bool bTwistRigid = !bFree[0] && fabsf(ragdoll.axes[0].maxRotation - ragdoll.axes[0].minRotation) <= 2.0f;
        if (flCone <= DEG2RAD(2.0f) && bTwistFree)
        {
            // Parallel joint aligns the frame Z axes; Source twist is the constraint X axis.
            const b3Quat qZtoX = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, b3Vec3_axisX);
            auto buildBearing = [=]() {
                b3ParallelJointDef def = b3DefaultParallelJointDef();
                def.base.bodyIdA = ref;
                def.base.bodyIdB = att;
                def.base.localFrameA.p = frameRef.p;
                def.base.localFrameA.q = b3MulQuat(frameRef.q, qZtoX);
                def.base.localFrameB.p = frameAtt.p;
                def.base.localFrameB.q = b3MulQuat(frameAtt.q, qZtoX);
                def.hertz = 120.0f;
                def.dampingRatio = 2.0f;
                return b3CreateParallelJoint(world, &def);
            };
            Box3DPhysicsConstraint* pBearing = new Box3DPhysicsConstraint(this, pRef, pAtt);
            pBearing->SetSaveInfo(kBox3DConstraint_Ragdoll, &ragdoll, sizeof(ragdoll));
            return FinishConstraint(pBearing, pGroup, breakParams, buildBearing);
        }
        if (flCone <= DEG2RAD(2.0f) && bTwistRigid)
        {
            auto buildLock = [=]() {
                b3MotorJointDef def = b3DefaultMotorJointDef();
                def.base.bodyIdA = ref;
                def.base.bodyIdB = att;
                def.base.localFrameA = frameRef;
                def.base.localFrameB = frameAtt;
                // Linear spring stays off (hertz 0): rotation-only, like IVP.
                def.angularHertz = 120.0f;
                def.angularDampingRatio = 2.0f;
                def.maxSpringTorque = FLT_MAX;
                return b3CreateMotorJoint(world, &def);
            };
            Box3DPhysicsConstraint* pLock = new Box3DPhysicsConstraint(this, pRef, pAtt);
            pLock->SetSaveInfo(kBox3DConstraint_Ragdoll, &ragdoll, sizeof(ragdoll));
            return FinishConstraint(pLock, pGroup, breakParams, buildLock);
        }

        // Real cones/twist limits: no joint (translation must stay free), per-step limits only.
        Box3DPhysicsConstraint* pAngular = new Box3DPhysicsConstraint(this, pRef, pAtt);
        pAngular->SetSaveInfo(kBox3DConstraint_Ragdoll, &ragdoll, sizeof(ragdoll));
        pAngular->SetupAngularLimits(frameRef, frameAtt, bCone, flCone, !bFree[0], flMin[0], flMax[0], flFriction, false);
        m_Pulleys.AddToTail(pAngular);
        return FinishConstraint(pAngular, pGroup, breakParams, std::function<b3JointId()>());
    }

    // In-island limits: solved with the contacts so piles reach equilibrium and sleep.
    const float flLimit = 0.99f * M_PI_F;

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
            if (!bFree[nDofAxis]) // a full-circle axis spins continuously, no hard stop
            {
                def.enableLimit = true;
                def.lowerAngle = ClampAngle(flMin[nDofAxis], flLimit);
                def.upperAngle = ClampAngle(flMax[nDofAxis], flLimit);
            }
            def.enableMotor = true;
            def.maxMotorTorque = flFriction;
            return b3CreateRevoluteJoint(world, &def);
        }

        // 2+ DOF: spherical swing cone + twist; Box3D's cone/twist axis is frame Z, Source twist is X.
        const b3Quat qZtoX = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, b3Vec3_axisX);
        b3SphericalJointDef def = b3DefaultSphericalJointDef();
        def.base.bodyIdA = ref;
        def.base.bodyIdB = att;
        def.base.localFrameA.p = frameRef.p;
        def.base.localFrameA.q = b3MulQuat(frameRef.q, qZtoX);
        def.base.localFrameB.p = frameAtt.p;
        def.base.localFrameB.q = b3MulQuat(frameAtt.q, qZtoX);
        if (bCone)
        {
            def.enableConeLimit = true;
            def.coneAngle = Min(flCone, 0.99f * 0.5f * M_PI_F); // b3 cone cap is pi/2
        }
        if (!bFree[0]) // full-circle twist must spin continuously
        {
            def.enableTwistLimit = true;
            def.lowerTwistAngle = ClampAngle(flMin[0], flLimit);
            def.upperTwistAngle = ClampAngle(flMax[0], flLimit);
        }
        def.enableMotor = true;
        def.maxMotorTorque = flFriction;
        return b3CreateSphericalJoint(world, &def);
    };
    Box3DPhysicsConstraint* pConstraint = new Box3DPhysicsConstraint(this, pRef, pAtt);
    pConstraint->SetSaveInfo(kBox3DConstraint_Ragdoll, &ragdoll, sizeof(ragdoll));
    return FinishConstraint(pConstraint, pGroup, breakParams, build);
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
    , m_flNaturalLen(SourceToBox::Distance(pParams->naturalLength))
    , m_flConstant(pParams->constant)
    , m_flDamping(pParams->damping)
    , m_flRelativeDamping(pParams->relativeDamping)
    , m_bOnlyStretch(pParams->onlyStretch)
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
}

Box3DPhysicsSpring::~Box3DPhysicsSpring()
{
}

// IVP_Actuator_Spring::do_simulation_controller: axial force (dlen - rest) * k - damp * closingSpeed,
// plus rel_pos_damp on the full anchor relative velocity, applied at the anchors; onlyStretch gates it.
void Box3DPhysicsSpring::Simulate(float dt)
{
    if (!m_pStart || !m_pEnd || dt <= 0.0f)
        return;

    const b3BodyId start = m_pStart->GetBodyID();
    const b3BodyId end = m_pEnd->GetBodyID();
    if (!b3Body_IsAwake(start) && !b3Body_IsAwake(end))
        return;

    const b3WorldTransform xfStart = b3Body_GetTransform(start);
    const b3WorldTransform xfEnd = b3Body_GetTransform(end);
    const b3Vec3 posStart = b3Add(b3ToVec3(xfStart.p), b3RotateVector(xfStart.q, m_AnchorStart));
    const b3Vec3 posEnd = b3Add(b3ToVec3(xfEnd.p), b3RotateVector(xfEnd.q, m_AnchorEnd));

    b3Vec3 dir = b3Sub(posStart, posEnd); // IVP: pos0 - pos1
    const float flLen = b3Length(dir);
    if (flLen < 1e-6f)
        return;
    dir = b3MulSV(1.0f / flLen, dir);

    if (m_bOnlyStretch && flLen <= m_flNaturalLen)
        return;

    // Positive when the anchors approach each other.
    const b3Vec3 vRel = b3Sub(
        b3Body_GetWorldPointVelocity(end, b3ToPos(posEnd)), b3Body_GetWorldPointVelocity(start, b3ToPos(posStart)));
    const float flDampSpeed = b3Dot(dir, vRel);
    const float flForce = (flLen - m_flNaturalLen) * m_flConstant - m_flDamping * flDampSpeed;

    // wake=false: IVP's async pushes only reach simulated (awake, unpinned) cores.
    b3Vec3 impulse = b3MulSV(flForce * dt, dir);
    impulse = b3Add(impulse, b3MulSV(-dt * m_flRelativeDamping, vRel));
    b3Body_ApplyLinearImpulse(end, impulse, b3ToPos(posEnd), false);
    b3Body_ApplyLinearImpulse(start, b3MulSV(-1.0f, impulse), b3ToPos(posStart), false);
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
    if (m_pStart)
        m_pStart->Wake();
    if (m_pEnd)
        m_pEnd->Wake();
}

void Box3DPhysicsSpring::SetSpringDamping(float flSpringDamping)
{
    m_flDamping = flSpringDamping;
    if (m_pStart)
        m_pStart->Wake();
    if (m_pEnd)
        m_pEnd->Wake();
}

void Box3DPhysicsSpring::SetSpringLength(float flSpringLength)
{
    m_flNaturalLen = SourceToBox::Distance(flSpringLength);
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
