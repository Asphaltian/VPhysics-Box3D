
#include "vbox_controller_shadow.h"

#include "cbase.h"
#include "vbox_object.h"
#include "vbox_surfaceprops.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------
// Shadow controller. The object is driven kinematically toward the game's target, so dynamic
// bodies can never push it off course (doors, lifts, NPC-driven objects).
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

Box3DPhysicsShadowController::Box3DPhysicsShadowController(
    Box3DPhysicsObject* pObject, bool allowTranslation, bool allowRotation)
    : m_pObject(pObject)
    , m_allowTranslation(allowTranslation)
    , m_allowRotation(allowRotation)
{
    const b3BodyId bodyId = m_pObject->GetBodyID();

    m_savedBodyType = b3Body_GetType(bodyId);
    b3Body_SetType(bodyId, b3_kinematicBody);

    m_savedMaterialIndex = m_pObject->GetMaterialIndex();
    UseShadowMaterial(true);

    m_savedCallbackFlags = m_pObject->GetCallbackFlags();
    unsigned short uFlags = m_savedCallbackFlags | CALLBACK_SHADOW_COLLISION;
    uFlags &= ~CALLBACK_GLOBAL_FRICTION;
    uFlags &= ~CALLBACK_GLOBAL_COLLIDE_STATIC;
    m_pObject->SetCallbackFlags(uFlags);
    m_pObject->EnableDrag(false);

    m_pObject->GetPosition(&m_targetPosition, &m_targetAngles);
}

Box3DPhysicsShadowController::~Box3DPhysicsShadowController()
{
    const b3BodyId bodyId = m_pObject->GetBodyID();
    const bool bMarkedForDelete = (m_pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) != 0;

    if (!bMarkedForDelete)
    {
        m_pObject->SetCallbackFlags(m_savedCallbackFlags);
        m_pObject->EnableDrag(true);
        UseShadowMaterial(false);

        if (b3Body_IsValid(bodyId))
        {
            b3Body_SetType(bodyId, m_savedBodyType);
            b3Body_SetAwake(bodyId, true);
        }
    }
}

void Box3DPhysicsShadowController::Update(const Vector& position, const QAngle& angles, float timeOffset)
{
    const Vector vecOldTarget = m_targetPosition;
    const QAngle angOldTarget = m_targetAngles;

    m_targetPosition = position;
    m_targetAngles = angles;
    m_secondsToArrival = Max(timeOffset, 0.0f);
    m_enabled = true;

    if ((position - vecOldTarget).LengthSqr() < 1e-8f
        && (Vector(angles.x, angles.y, angles.z) - Vector(angOldTarget.x, angOldTarget.y, angOldTarget.z)).LengthSqr() < 1e-8f)
        return;

    m_pObject->Wake();
}

void Box3DPhysicsShadowController::MaxSpeed(float maxSpeed, float maxAngularSpeed)
{
    m_maxSpeed = maxSpeed;
    m_maxDampSpeed = maxSpeed;
    m_maxAngular = maxAngularSpeed;
    m_maxDampAngular = maxAngularSpeed;
}

void Box3DPhysicsShadowController::StepUp(float height)
{
    if (height == 0.0f)
        return;

    Vector vecPos;
    QAngle angPos;
    m_pObject->GetPosition(&vecPos, &angPos);
    vecPos.z += height;
    m_pObject->SetPosition(vecPos, angPos, true);
}

void Box3DPhysicsShadowController::SetTeleportDistance(float teleportDistance)
{
    m_teleportDistance = teleportDistance;
}

bool Box3DPhysicsShadowController::AllowsTranslation()
{
    return m_allowTranslation;
}

bool Box3DPhysicsShadowController::AllowsRotation()
{
    return m_allowRotation;
}

void Box3DPhysicsShadowController::SetPhysicallyControlled(bool isPhysicallyControlled)
{
    m_isPhysicallyControlled = isPhysicallyControlled;
}

bool Box3DPhysicsShadowController::IsPhysicallyControlled()
{
    return m_isPhysicallyControlled;
}

void Box3DPhysicsShadowController::GetLastImpulse(Vector* pOut)
{
    if (pOut)
        *pOut = m_lastImpulse;
}

void Box3DPhysicsShadowController::UseShadowMaterial(bool bUseShadowMaterial)
{
    const int nCurrent = m_pObject->GetMaterialIndex();
    const int nTarget = bUseShadowMaterial ? MATERIAL_INDEX_SHADOW : m_savedMaterialIndex;
    if (nTarget != nCurrent)
        m_pObject->SetMaterialIndex(nTarget);
}

void Box3DPhysicsShadowController::ObjectMaterialChanged(int materialIndex)
{
    if (materialIndex != MATERIAL_INDEX_SHADOW)
        m_savedMaterialIndex = materialIndex;
}

float Box3DPhysicsShadowController::GetTargetPosition(Vector* pPositionOut, QAngle* pAnglesOut)
{
    if (pPositionOut)
        *pPositionOut = m_targetPosition;
    if (pAnglesOut)
        *pAnglesOut = m_targetAngles;

    return m_secondsToArrival;
}

float Box3DPhysicsShadowController::GetTeleportDistance()
{
    return m_teleportDistance;
}

void Box3DPhysicsShadowController::GetMaxSpeed(float* pMaxSpeedOut, float* pMaxAngularSpeedOut)
{
    if (pMaxSpeedOut)
        *pMaxSpeedOut = m_maxSpeed;
    if (pMaxAngularSpeedOut)
        *pMaxAngularSpeedOut = m_maxAngular;
}

void Box3DPhysicsShadowController::OnPreSimulate(float flDeltaTime)
{
    if (!m_enabled)
        return;

    const b3BodyId bodyId = m_pObject->GetBodyID();

    if (m_secondsToArrival > 0.0f)
    {
        b3WorldTransform target;
        target.p = SourceToBox::Distance(m_targetPosition);
        target.q = SourceToBox::Angle(m_targetAngles);
        b3Body_SetTargetTransform(bodyId, target, m_secondsToArrival, true);
    }
    else
    {
        b3Body_SetTransform(bodyId, SourceToBox::Distance(m_targetPosition), SourceToBox::Angle(m_targetAngles));
        b3Body_SetLinearVelocity(bodyId, b3Vec3{});
        b3Body_SetAngularVelocity(bodyId, b3Vec3{});
        b3Body_SetAwake(bodyId, true);
        m_enabled = false;
    }

    m_secondsToArrival = Max(m_secondsToArrival - flDeltaTime, 0.0f);
}
