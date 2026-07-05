
#include "vbox_controller_player.h"

#include "cbase.h"
#include "vbox_environment.h"
#include "vbox_object.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------
// Player controller. The shadow is a real dynamic object;
// the controller servos its velocity to the game's position and limits how hard it may drive
// into contacts (that limit is what makes pushing respect mass).
//-------------------------------------------------------------------------------------------------
// A contact normal.z below this (pointing down out of the surface the body rests on) counts as ground.
static constexpr float kGroundNormalZ = -0.7f;

// The other object in a body's contact; also reports whether pSelf is side A (manifold normals point A -> B).
static Box3DPhysicsObject* ContactOther(const b3ContactData& contact, Box3DPhysicsObject* pSelf, bool* pSelfIsA = nullptr)
{
    Box3DPhysicsObject* pA = static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(b3Shape_GetBody(contact.shapeIdA)));
    const bool bSelfIsA = pA == pSelf;
    if (pSelfIsA)
        *pSelfIsA = bSelfIsA;
    return bSelfIsA ? static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(b3Shape_GetBody(contact.shapeIdB))) : pA;
}

static void ComputeController(
    Vector& vCurrentSpeed, const Vector& vDelta, const Vector& vMaxSpeed, float flScaleDelta, float flDamping,
    Vector* pOutImpulse)
{
    Vector vAcceleration = vDelta * flScaleDelta;

    if (vCurrentSpeed.LengthSqr() < 1e-6f)
        vCurrentSpeed = vec3_origin;

    vAcceleration += vCurrentSpeed * -flDamping;

    for (int i = 0; i < 3; i++)
    {
        if (fabsf(vAcceleration[i]) < vMaxSpeed[i])
            continue;
        vAcceleration[i] = vAcceleration[i] < 0.0f ? -vMaxSpeed[i] : vMaxSpeed[i];
    }

    vCurrentSpeed += vAcceleration;
    if (pOutImpulse)
        *pOutImpulse = vAcceleration;
}

// Contact normals the controller may not drive into faster than the push speed limit.
class CNormalList
{
public:
    void AddNormal(const Vector& vecNormal)
    {
        if (m_nCount == MAX_NORMALS)
            return;
        for (int i = 0; i < m_nCount; i++)
        {
            if (DotProduct(m_Normals[i], vecNormal) > 0.99f)
                return;
        }
        m_Normals[m_nCount++] = vecNormal;
    }

    Vector ClampVector(const Vector& vecIn, float flLimitVel)
    {
        if (m_nCount > 2)
        {
            for (int i = 0; i < m_nCount; i++)
            {
                if (DotProduct(vecIn, m_Normals[i]) > 0.0f)
                    return vec3_origin;
            }
        }
        else if (m_nCount == 2)
        {
            Vector vecCrease;
            CrossProduct(m_Normals[0], m_Normals[1], vecCrease);
            return vecCrease * DotProduct(vecIn, vecCrease);
        }
        else if (m_nCount == 1)
        {
            const float flDot = DotProduct(vecIn, m_Normals[0]);
            if (flDot > flLimitVel)
                return vecIn + m_Normals[0] * (flLimitVel - flDot);
        }
        return vecIn;
    }

private:
    static constexpr int MAX_NORMALS = 8;
    Vector m_Normals[MAX_NORMALS];
    int m_nCount = 0;
};

Box3DPhysicsPlayerController::Box3DPhysicsPlayerController(Box3DPhysicsObject* pObject)
{
    SetObjectInternal(pObject);
}

Box3DPhysicsPlayerController::~Box3DPhysicsPlayerController()
{
    SetObjectInternal(nullptr);
}

void Box3DPhysicsPlayerController::SetObjectInternal(Box3DPhysicsObject* pObject)
{
    if (m_pObject == pObject)
        return;

    if (m_pObject)
    {
        const b3BodyId oldId = m_pObject->GetBodyID();
        if (b3Body_IsValid(oldId))
            b3Body_SetAngularDamping(oldId, m_flSavedAngularDamping);
        m_pObject->SetCallbackFlags(m_pObject->GetCallbackFlags() & ~CALLBACK_IS_PLAYER_CONTROLLER);
    }

    m_pObject = pObject;
    SetGround(nullptr);

    if (m_pObject)
    {
        const b3BodyId bodyId = m_pObject->GetBodyID();

        // IVP replaces the controlled core's rot_speed_damp_factor with (100,100,100); heavy
        // angular damping is the box3d equivalent.
        // Which hull collides is the game's business (EnableCollisions via SetVCollisionState).
        m_pObject->EnableDrag(false);
        m_flSavedAngularDamping = b3Body_GetAngularDamping(bodyId);
        b3Body_SetAngularDamping(bodyId, 100.0f);

        m_pObject->SetCallbackFlags(m_pObject->GetCallbackFlags() | CALLBACK_IS_PLAYER_CONTROLLER);
    }
}

void Box3DPhysicsPlayerController::SetGround(Box3DPhysicsObject* pGround)
{
    m_pGround = pGround;
}

void Box3DPhysicsPlayerController::ClearGround(Box3DPhysicsObject* pObject)
{
    if (m_pGround == pObject)
        m_pGround = nullptr;
}

void Box3DPhysicsPlayerController::Update(
    const Vector& position, const Vector& velocity, float secondsToArrival, bool onground, IPhysicsObject* ground)
{
    m_updatedSinceLast = true;

    // If the object hasn't moved, abort.
    if ((velocity - m_currentSpeed).LengthSqr() < 1e-6f && (position - m_targetPosition).LengthSqr() < 1e-6f)
        return;

    m_targetPosition = position;
    m_currentSpeed = velocity;
    m_secondsToArrival = secondsToArrival < 0.0f ? 0.0f : secondsToArrival;

    if (m_pObject)
        b3Body_SetAwake(m_pObject->GetBodyID(), true);

    m_enable = true;
    if (velocity.LengthSqr() <= 0.1f)
    {
        // No input velocity, just go where physics takes you.
        m_enable = false;
        ground = nullptr;
    }
    else
    {
        MaxSpeed(velocity);
    }

    SetGround(static_cast<Box3DPhysicsObject*>(ground));
    if (m_pGround)
        m_pGround->WorldToLocal(&m_groundPosition, m_targetPosition);
}

void Box3DPhysicsPlayerController::SetEventHandler(IPhysicsPlayerControllerEvent* handler)
{
    m_pHandler = handler;
}

static bool IsControlledByGame(Box3DPhysicsObject* pObject)
{
    IPhysicsShadowController* pShadow = pObject->GetShadowController();
    if (pShadow && !pShadow->IsPhysicallyControlled())
        return true;

    return (pObject->GetCallbackFlags() & CALLBACK_IS_PLAYER_CONTROLLER) != 0;
}

bool Box3DPhysicsPlayerController::IsInContact()
{
    if (!m_pObject || !m_pObject->IsCollisionEnabled())
        return false;

    b3ContactData contacts[32];
    const int nCount = b3Body_GetContactData(m_pObject->GetBodyID(), contacts, 32);
    for (int i = 0; i < nCount; i++)
    {
        bool bTouching = false;
        for (int j = 0; j < contacts[i].manifoldCount; j++)
            bTouching |= contacts[i].manifolds[j].pointCount > 0;
        if (!bTouching)
            continue;

        Box3DPhysicsObject* pOther = ContactOther(contacts[i], m_pObject);
        if (!pOther || !pOther->IsCollisionEnabled() || !pOther->IsMoveable())
            continue;

        // Skip game-controlled shadow objects; we want physically simulated contact.
        if (IsControlledByGame(pOther))
            continue;

        return true;
    }
    return false;
}

void Box3DPhysicsPlayerController::MaxSpeed(const Vector& maxVelocity)
{
    if (!m_pObject)
        return;

    Vector vCurrentVelocity;
    m_pObject->GetVelocity(&vCurrentVelocity, nullptr);

    Vector vDirection = maxVelocity;
    const float flLength = VectorNormalize(vDirection);

    Vector vAvailable = maxVelocity;
    const float flDot = DotProduct(vDirection, vCurrentVelocity);
    if (flDot > 0.0f)
        vAvailable -= vDirection * (flDot * flLength);

    m_maxSpeed = Vector(fabsf(vAvailable.x), fabsf(vAvailable.y), fabsf(vAvailable.z));
}

void Box3DPhysicsPlayerController::SetObject(IPhysicsObject* pObject)
{
    SetObjectInternal(static_cast<Box3DPhysicsObject*>(pObject));
}

int Box3DPhysicsPlayerController::GetShadowPosition(Vector* position, QAngle* angles)
{
    if (m_pObject)
        m_pObject->GetPosition(position, angles);
    return 1;
}

void Box3DPhysicsPlayerController::StepUp(float height)
{
    if (height == 0.0f || !m_pObject)
        return;

    Vector vPos;
    QAngle qAngles;
    m_pObject->GetPosition(&vPos, &qAngles);
    vPos.z += height;
    m_pObject->SetPosition(vPos, qAngles, true);
}

void Box3DPhysicsPlayerController::Jump()
{
}

void Box3DPhysicsPlayerController::GetShadowVelocity(Vector* velocity)
{
    if (!velocity || !m_pObject)
        return;

    m_pObject->GetVelocity(velocity, nullptr);

    if (m_pGround)
    {
        Vector vGroundPoint;
        m_pGround->LocalToWorld(&vGroundPoint, m_groundPosition);
        Vector vBaseVelocity;
        m_pGround->GetVelocityAtPoint(vGroundPoint, &vBaseVelocity);
        *velocity -= vBaseVelocity;
    }
}

IPhysicsObject* Box3DPhysicsPlayerController::GetObject()
{
    return m_pObject;
}

void Box3DPhysicsPlayerController::GetLastImpulse(Vector* pOut)
{
    if (pOut)
        *pOut = m_lastImpulse;
}

void Box3DPhysicsPlayerController::SetPushMassLimit(float maxPushMass)
{
    m_pushableMassLimit = maxPushMass;
}

void Box3DPhysicsPlayerController::SetPushSpeedLimit(float maxPushSpeed)
{
    m_pushableSpeedLimit = maxPushSpeed;
}

float Box3DPhysicsPlayerController::GetPushMassLimit()
{
    return m_pushableMassLimit;
}

float Box3DPhysicsPlayerController::GetPushSpeedLimit()
{
    return m_pushableSpeedLimit;
}

bool Box3DPhysicsPlayerController::WasFrozen()
{
    return false;
}

int Box3DPhysicsPlayerController::TryTeleportObject()
{
    if (m_pHandler && !m_pHandler->ShouldMoveTo(m_pObject, m_targetPosition))
        return 0;

    QAngle qAngles;
    m_pObject->GetPosition(nullptr, &qAngles);

    if (m_pObject->IsCollisionEnabled())
    {
        m_pObject->EnableCollisions(false);
        m_pObject->SetPosition(m_targetPosition, qAngles, true);
        m_pObject->EnableCollisions(true);
    }
    else
    {
        m_pObject->SetPosition(m_targetPosition, qAngles, true);
    }
    return 1;
}

void Box3DPhysicsPlayerController::OnPreSimulate(float flDeltaTime)
{
    if (!m_pObject || !m_enable || flDeltaTime <= 0.0f)
        return;

    const b3BodyId bodyId = m_pObject->GetBodyID();
    if (!b3Body_IsAwake(bodyId))
        return;

    Vector vPosition, vSpeed;
    m_pObject->GetPosition(&vPosition, nullptr);
    m_pObject->GetVelocity(&vSpeed, nullptr);

    // Ride a moving ground object: keep the target at the stored ground-local point and servo in
    // the ground's reference frame.
    Vector vBaseVelocity = vec3_origin;
    if (m_pGround)
    {
        m_pGround->LocalToWorld(&m_targetPosition, m_groundPosition);
        m_pGround->GetVelocityAtPoint(m_targetPosition, &vBaseVelocity);
        vSpeed -= vBaseVelocity;
    }

    const Vector vDeltaPos = m_targetPosition - vPosition;

    if (vDeltaPos.LengthSqr() > m_maxDeltaPosition * m_maxDeltaPosition)
    {
        if (TryTeleportObject())
        {
            // IVP modifies core speed in place, so the ground base-velocity subtraction
            // persists across the teleport.
            b3Body_SetLinearVelocity(bodyId, SourceToBox::Distance(vSpeed));
            return;
        }
    }

    float flFraction = 1.0f;
    if (m_secondsToArrival > 0.0f)
        flFraction = Min(flDeltaTime / m_secondsToArrival, 1.0f);

    if (!m_updatedSinceLast)
    {
        // No game update since the last step: the error estimate is stale, so cap the impulse to
        // the last known good one.
        const float flLen = m_lastImpulse.Length();
        ComputeController(vSpeed, vDeltaPos, Vector(flLen, flLen, flLen), flFraction / flDeltaTime, m_dampFactor, nullptr);
    }
    else
    {
        ComputeController(vSpeed, vDeltaPos, m_maxSpeed, flFraction / flDeltaTime, m_dampFactor, &m_lastImpulse);
    }
    vSpeed += vBaseVelocity;
    m_updatedSinceLast = false;

    // Limit how fast the controller drives into its contacts: unmoveable or too-heavy objects can't
    // be pushed at all, everything else at most at the push speed limit.
    const float flMass = m_pObject->GetMass();
    const float flInvMass = flMass > 0.0f ? 1.0f / flMass : 0.0f;
    float flLimitVel = m_pushableSpeedLimit;
    bool bGround = false;
    CNormalList normalList;

    b3ContactData contacts[32];
    const int nCount = b3Body_GetContactData(bodyId, contacts, 32);
    for (int i = 0; i < nCount; i++)
    {
        bool bSelfIsA;
        Box3DPhysicsObject* pOther = ContactOther(contacts[i], m_pObject, &bSelfIsA);

        for (int j = 0; j < contacts[i].manifoldCount; j++)
        {
            const b3Manifold& manifold = contacts[i].manifolds[j];
            if (manifold.pointCount <= 0)
                continue;

            // The manifold normal points A -> B; orient it from the player into the other object.
            Vector vecNormal = BoxToSource::Unitless(manifold.normal);
            if (!bSelfIsA)
                vecNormal = -vecNormal;

            if (vecNormal.z < kGroundNormalZ)
                bGround = true;

            if (vecNormal.z > -0.99f)
            {
                if (!pOther || !pOther->IsMoveable() || pOther->GetMass() > m_pushableMassLimit)
                    flLimitVel = 0.0f;

                float flImpulse = 0.0f;
                for (int p = 0; p < manifold.pointCount; p++)
                    flImpulse = Max(flImpulse, manifold.points[p].totalNormalImpulse);

                const float flPushSpeed = DotProduct(vSpeed, vecNormal);
                const float flContactVel = BoxToSource::Distance(flImpulse * flInvMass);
                if (flPushSpeed + flContactVel > flLimitVel)
                    normalList.AddNormal(vecNormal);
            }
        }
    }

    const Vector vLimit = normalList.ClampVector(vSpeed, flLimitVel) - vSpeed;
    vSpeed += vLimit;
    m_lastImpulse += vLimit;

    if (bGround)
    {
        // On the ground, press down with exactly one step of gravity and no more.
        const b3Vec3 vGravity = b3World_GetGravity(m_pObject->GetEnvironment()->GetWorldId());
        const float flGravDt = BoxToSource::Distance(sqrtf(b3Dot(vGravity, vGravity))) * flDeltaTime;
        if (m_lastImpulse.z <= 0.0f)
        {
            const float flDelta = -flGravDt - m_lastImpulse.z;
            vSpeed.z += flDelta;
            m_lastImpulse.z += flDelta;
        }
    }

    b3Body_SetLinearVelocity(bodyId, SourceToBox::Distance(vSpeed));

    m_secondsToArrival = Max(m_secondsToArrival - flDeltaTime, 0.0f);
}
