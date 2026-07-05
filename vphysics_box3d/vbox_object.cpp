//=================================================================================================
//
// A physics object
//
//=================================================================================================

#include "vbox_object.h"

#include "cbase.h"
#include "vbox_collide.h"
#include "vbox_controller_shadow.h"
#include "vbox_environment.h"
#include "vbox_friction.h"
#include "vbox_surfaceprops.h"
#include "vphysics/friction.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

// Monotonic id stamped on every object at creation for dangling-proof identity comparison (never reused).
static uint64 s_nNextUniqueId = 1;

// Cap game-set inertia to this multiple of Box3D's native value; IVP's raw inertia destabilizes ragdolls.
static ConVar vbox_inertia_scale(
    "vbox_inertia_scale", "2", FCVAR_NONE, "Max multiple of Box3D's native inertia a SetInertia may apply.", true, 0.0f, true,
    1000.0f);

namespace
{
    // Run a callable over every shape on a body.
    template<typename Fn> void ForEachShape(b3BodyId bodyId, Fn fn)
    {
        const int nCount = b3Body_GetShapeCount(bodyId);
        CUtlVector<b3ShapeId> shapes;
        shapes.SetCount(nCount);
        b3Body_GetShapes(bodyId, shapes.Base(), nCount);
        for (int i = 0; i < nCount; i++)
            fn(shapes[i]);
    }

    // Integral of a differential drag area's torque over an OBB half-extent (IVP RecomputeDragBases).
    float AngDragIntegral(float flInvInertia, float l, float w, float h)
    {
        const float w2 = w * w, l2 = l * l, h2 = h * h;
        return flInvInertia * ((1.0f / 3.0f) * w2 * l * l2 + 0.5f * w2 * w2 * l + l * w2 * h2);
    }
} // namespace

Box3DPhysicsObject::Box3DPhysicsObject(
    b3BodyId bodyId, Box3DPhysicsEnvironment* pEnvironment, bool bStatic, int nMaterialIndex, const CPhysCollide* pCollide,
    const objectparams_t* pParams)
    : m_bStatic(bStatic)
    , m_materialIndex(nMaterialIndex)
    , m_pCollide(pCollide)
    , m_BodyId(bodyId)
    , m_pEnvironment(pEnvironment)
{
    m_WorldId = b3Body_GetWorld(bodyId);
    b3Body_SetUserData(bodyId, this);
    m_nUniqueId = s_nNextUniqueId++;

    if (pParams)
    {
        m_pGameData = pParams->pGameData;
        if (pParams->pName)
            m_pName = pParams->pName;

        m_flLinearDamping = pParams->damping;
        m_flAngularDamping = pParams->rotdamping;
        m_flVolume = pParams->volume;
        m_flDragCoefficient = pParams->dragCoefficient;
        m_flAngularDragCoefficient = pParams->dragCoefficient;
        m_bDragEnabled = pParams->dragCoefficient != 0.0f;
        if (!bStatic)
        {
            b3Body_SetLinearDamping(bodyId, pParams->damping);
            b3Body_SetAngularDamping(bodyId, pParams->rotdamping);
        }

        if (pParams->mass > 0.0f)
            SetMass(pParams->mass);
    }

    if (m_flCachedMass <= 0.0f)
    {
        m_flCachedMass = bStatic ? 0.0f : b3Body_GetMass(bodyId);
        m_flCachedInvMass = bStatic ? 0.0f : b3Body_GetInverseMass(bodyId);
    }

    if (surfacedata_t* pSurface = Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData(m_materialIndex))
        m_flMaterialDensity = pSurface->physics.density;
    CalculateBuoyancy();
    RecomputeDragBases();
}

// Buoyancy ratio = (mass/volume) / material density, so a prop sinks iff its material density > the water's,
// regardless of how loose its collision hull is.
void Box3DPhysicsObject::CalculateBuoyancy()
{
    if (m_flVolume > 0.0f && m_flMaterialDensity > 0.0f)
    {
        const float flVolume = SourceToBox::Volume(Max(m_flVolume, 5.0f));
        const float flActualDensity = m_flCachedMass / flVolume;
        m_flBuoyancyRatio = flActualDensity / m_flMaterialDensity;
    }
    else
    {
        m_flBuoyancyRatio = 1.0f;
    }
}

Box3DPhysicsObject::~Box3DPhysicsObject()
{
    // The environment owns the Box3D body lifetime (b3DestroyBody in DestroyObject).
}

//-------------------------------------------------------------------------------------------------

bool Box3DPhysicsObject::IsStatic() const
{
    return m_bStatic;
}
bool Box3DPhysicsObject::IsAsleep() const
{
    return m_bStatic || !b3Body_IsAwake(m_BodyId);
}
bool Box3DPhysicsObject::IsTrigger() const
{
    return m_bTrigger;
}
bool Box3DPhysicsObject::IsFluid() const
{
    return false;
}
bool Box3DPhysicsObject::IsHinged() const
{
    return false;
}
bool Box3DPhysicsObject::IsCollisionEnabled() const
{
    return m_bCollisionEnabled;
}
bool Box3DPhysicsObject::IsGravityEnabled() const
{
    return !m_bStatic && m_bGravityEnabled;
}
bool Box3DPhysicsObject::IsDragEnabled() const
{
    return m_bDragEnabled;
}
bool Box3DPhysicsObject::IsMotionEnabled() const
{
    return !m_bStatic && m_bMotionEnabled;
}
bool Box3DPhysicsObject::IsMoveable() const
{
    return IsMotionEnabled();
}
bool Box3DPhysicsObject::IsAttachedToConstraint(bool) const
{
    return false;
}

void Box3DPhysicsObject::EnableCollisions(bool enable)
{
    if (m_bCollisionEnabled == enable)
        return;

    m_bCollisionEnabled = enable;

    if (!b3Body_IsValid(m_BodyId))
        return;

    b3Filter filter = b3DefaultFilter();
    if (!enable)
        filter.maskBits = 0;

    ForEachShape(m_BodyId, [&](b3ShapeId shape) { b3Shape_SetFilter(shape, filter, true); });
}

void Box3DPhysicsObject::EnableGravity(bool enable)
{
    m_bGravityEnabled = enable;
    if (!m_bStatic)
        b3Body_SetGravityScale(m_BodyId, enable ? 1.0f : 0.0f);
}

void Box3DPhysicsObject::EnableDrag(bool enable)
{
    m_bDragEnabled = enable;
}

void Box3DPhysicsObject::EnableMotion(bool enable)
{
    if (m_bStatic || m_bMotionEnabled == enable)
        return;

    m_bMotionEnabled = enable;
    b3Body_SetType(m_BodyId, enable ? b3_dynamicBody : b3_staticBody);
    if (enable)
    {
        b3Body_ApplyMassFromShapes(m_BodyId);
        b3Body_SetAwake(m_BodyId, true);
    }
}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsObject::SetGameData(void* pGameData)
{
    m_pGameData = pGameData;
}
void* Box3DPhysicsObject::GetGameData() const
{
    return m_pGameData;
}
void Box3DPhysicsObject::SetGameFlags(unsigned short userFlags)
{
    m_gameFlags = userFlags;
}
unsigned short Box3DPhysicsObject::GetGameFlags() const
{
    return m_gameFlags;
}
void Box3DPhysicsObject::SetGameIndex(unsigned short gameIndex)
{
    m_gameIndex = gameIndex;
}
unsigned short Box3DPhysicsObject::GetGameIndex() const
{
    return m_gameIndex;
}
void Box3DPhysicsObject::SetCallbackFlags(unsigned short flags)
{
    m_callbackFlags = flags;
}
unsigned short Box3DPhysicsObject::GetCallbackFlags() const
{
    return m_callbackFlags;
}

void Box3DPhysicsObject::Wake()
{
    if (!m_bStatic)
        b3Body_SetAwake(m_BodyId, true);
}
void Box3DPhysicsObject::Sleep()
{
    if (!m_bStatic)
        b3Body_SetAwake(m_BodyId, false);
}
void Box3DPhysicsObject::RecheckCollisionFilter()
{ /* Not needed */
}
void Box3DPhysicsObject::RecheckContactPoints(bool)
{ /* Not needed */
}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsObject::SetMass(float mass)
{
    mass = clamp(mass, 1.0f, VPHYSICS_MAX_MASS);
    m_flCachedMass = mass;
    m_flCachedInvMass = 1.0f / mass;
    CalculateBuoyancy();

    if (m_bStatic)
        return;

    b3MassData massData = b3Body_GetMassData(m_BodyId);
    const float scale = massData.mass > 0.0f ? mass / massData.mass : 1.0f;
    massData.mass = mass;
    massData.inertia.cx.x *= scale;
    massData.inertia.cx.y *= scale;
    massData.inertia.cx.z *= scale;
    massData.inertia.cy.x *= scale;
    massData.inertia.cy.y *= scale;
    massData.inertia.cy.z *= scale;
    massData.inertia.cz.x *= scale;
    massData.inertia.cz.y *= scale;
    massData.inertia.cz.z *= scale;
    b3Body_SetMassData(m_BodyId, massData);
    RecomputeDragBases();
}

float Box3DPhysicsObject::GetMass() const
{
    return m_flCachedMass;
}
float Box3DPhysicsObject::GetInvMass() const
{
    return m_flCachedInvMass;
}

Vector Box3DPhysicsObject::GetInertia() const
{
    // IVP hands the game its raw kg m^2 diagonal (axis conversion only, no unit scaling); the game
    // feeds these values straight back into SetInertia.
    const b3Matrix3 inertia = b3Body_GetLocalRotationalInertia(m_BodyId);
    return Vector(fabsf(inertia.cx.x), fabsf(inertia.cy.y), fabsf(inertia.cz.z));
}

Vector Box3DPhysicsObject::GetInvInertia() const
{
    const Vector inertia = GetInertia();
    return Vector(
        inertia.x > 0.0f ? 1.0f / inertia.x : 0.0f, inertia.y > 0.0f ? 1.0f / inertia.y : 0.0f,
        inertia.z > 0.0f ? 1.0f / inertia.z : 0.0f);
}

void Box3DPhysicsObject::SetInertia(const Vector& inertia)
{
    if (m_bStatic)
        return;

    // Clamp to vbox_inertia_scale x Box3D's native inertia; IVP's raw value flails active ragdolls.
    const float flScale = vbox_inertia_scale.GetFloat();
    b3MassData massData = b3Body_GetMassData(m_BodyId);
    const float capX = fabsf(massData.inertia.cx.x) * flScale;
    const float capY = fabsf(massData.inertia.cy.y) * flScale;
    const float capZ = fabsf(massData.inertia.cz.z) * flScale;
    massData.inertia = b3Matrix3{};
    massData.inertia.cx.x = Min(fabsf(inertia.x), capX);
    massData.inertia.cy.y = Min(fabsf(inertia.y), capY);
    massData.inertia.cz.z = Min(fabsf(inertia.z), capZ);
    b3Body_SetMassData(m_BodyId, massData);
    RecomputeDragBases();
}

void Box3DPhysicsObject::SetDamping(const float* speed, const float* rot)
{
    if (speed)
    {
        m_flLinearDamping = *speed;
        if (!m_bStatic)
            b3Body_SetLinearDamping(m_BodyId, *speed);
    }
    if (rot)
    {
        m_flAngularDamping = *rot;
        if (!m_bStatic)
            b3Body_SetAngularDamping(m_BodyId, *rot);
    }
}

void Box3DPhysicsObject::GetDamping(float* speed, float* rot) const
{
    if (speed)
        *speed = m_flLinearDamping;
    if (rot)
        *rot = m_flAngularDamping;
}

void Box3DPhysicsObject::SetDragCoefficient(float* pDrag, float* pAngularDrag)
{
    if (pDrag)
        m_flDragCoefficient = *pDrag;
    if (pAngularDrag)
        m_flAngularDragCoefficient = *pAngularDrag;
    m_bDragEnabled = m_flDragCoefficient != 0.0f || m_flAngularDragCoefficient != 0.0f;
    RecomputeDragBases();
}

void Box3DPhysicsObject::RecomputeDragBases()
{
    m_dragBasis = vec3_origin;
    m_angDragBasis = vec3_origin;
    if (m_bStatic || !m_pCollide)
        return;

    // OBB drag basis: face areas (linear) and the swept-area integral (angular), over inverse mass/inertia.
    Vector mins, maxs;
    Box3DPhysicsCollision::GetInstance().CollideGetAABB(&mins, &maxs, m_pCollide, vec3_origin, vec3_angle);
    const Vector areaFractions = Box3DPhysicsCollision::GetInstance().CollideGetOrthographicAreas(m_pCollide);

    Vector delta = maxs - mins;
    delta.x = fabsf(SourceToBox::Distance(delta.x));
    delta.y = fabsf(SourceToBox::Distance(delta.y));
    delta.z = fabsf(SourceToBox::Distance(delta.z));

    m_dragBasis.x = delta.y * delta.z * areaFractions.x;
    m_dragBasis.y = delta.x * delta.z * areaFractions.y;
    m_dragBasis.z = delta.x * delta.y * areaFractions.z;
    m_dragBasis *= GetInvMass();

    const Vector invInertia = GetInvInertia();
    delta *= 0.5f; // half-extents
    m_angDragBasis.x = areaFractions.z * AngDragIntegral(invInertia.x, delta.x, delta.y, delta.z)
        + areaFractions.y * AngDragIntegral(invInertia.x, delta.x, delta.z, delta.y);
    m_angDragBasis.y = areaFractions.z * AngDragIntegral(invInertia.y, delta.y, delta.x, delta.z)
        + areaFractions.x * AngDragIntegral(invInertia.y, delta.y, delta.z, delta.x);
    m_angDragBasis.z = areaFractions.y * AngDragIntegral(invInertia.z, delta.z, delta.x, delta.y)
        + areaFractions.x * AngDragIntegral(invInertia.z, delta.z, delta.y, delta.x);
}

void Box3DPhysicsObject::ApplyAirDrag(float flAirDensity, float dt)
{
    if (!m_bDragEnabled || m_bStatic)
        return;

    const b3Vec3 vWorld = b3Body_GetLinearVelocity(m_BodyId);
    const b3Vec3 vLocal = b3Body_GetLocalVector(m_BodyId, vWorld);
    const float flDrag = m_flDragCoefficient
        * (fabsf(vLocal.x * m_dragBasis.x) + fabsf(vLocal.y * m_dragBasis.y) + fabsf(vLocal.z * m_dragBasis.z));
    const float flDragForce = -0.5f * flDrag * flAirDensity * dt;
    if (flDragForce < 0.0f)
        b3Body_SetLinearVelocity(m_BodyId, b3MulSV(1.0f + Max(flDragForce, -1.0f), vWorld));

    const b3Vec3 wWorld = b3Body_GetAngularVelocity(m_BodyId);
    const b3Vec3 wLocal = b3Body_GetLocalVector(m_BodyId, wWorld);
    const float flAngDrag = m_flAngularDragCoefficient
        * (fabsf(wLocal.x * m_angDragBasis.x) + fabsf(wLocal.y * m_angDragBasis.y) + fabsf(wLocal.z * m_angDragBasis.z));
    const float flAngDragForce = -flAngDrag * flAirDensity * dt;
    if (flAngDragForce < 0.0f)
        b3Body_SetAngularVelocity(m_BodyId, b3MulSV(1.0f + Max(flAngDragForce, -1.0f), wWorld));
}
void Box3DPhysicsObject::SetBuoyancyRatio(float ratio)
{
    m_flBuoyancyRatio = ratio;
}

int Box3DPhysicsObject::GetMaterialIndex() const
{
    return m_materialIndex;
}
void Box3DPhysicsObject::SetMaterialIndex(int materialIndex)
{
    if (m_materialIndex == materialIndex)
        return;

    m_materialIndex = materialIndex;

    surfacedata_t* pSurface = Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData(materialIndex);
    if (pSurface)
    {
        m_flMaterialDensity = pSurface->physics.density;
        CalculateBuoyancy();
    }
    if (!pSurface || !b3Body_IsValid(m_BodyId))
        return;

    const float flFriction = Max(pSurface->physics.friction, 0.0f);
    const float flRestitution = pSurface->physics.elasticity;
    ForEachShape(m_BodyId, [&](b3ShapeId shape) {
        b3Shape_SetFriction(shape, flFriction);
        b3Shape_SetRestitution(shape, flRestitution);
    });

    if (m_pShadowController)
        m_pShadowController->ObjectMaterialChanged(materialIndex);
}
unsigned int Box3DPhysicsObject::GetContents() const
{
    return m_contents;
}
void Box3DPhysicsObject::SetContents(unsigned int contents)
{
    m_contents = contents;
}

float Box3DPhysicsObject::GetSphereRadius() const
{
    return m_flSphereRadius;
}
void Box3DPhysicsObject::SetSphereRadius(float radius)
{
    m_flSphereRadius = radius;
}
float Box3DPhysicsObject::GetEnergy() const
{
    if (m_bStatic)
        return 0.0f;

    // 1/2 mvv + 1/2 wIw, converted like IVP's ConvertEnergyToHL.
    const b3Vec3 v = b3Body_GetLinearVelocity(m_BodyId);
    const b3Vec3 w = b3InvRotateVector(b3Body_GetTransform(m_BodyId).q, b3Body_GetAngularVelocity(m_BodyId));
    const b3MassData massData = b3Body_GetMassData(m_BodyId);

    const b3Vec3 Iw = { massData.inertia.cx.x * w.x + massData.inertia.cy.x * w.y + massData.inertia.cz.x * w.z,
                        massData.inertia.cx.y * w.x + massData.inertia.cy.y * w.y + massData.inertia.cz.y * w.z,
                        massData.inertia.cx.z * w.x + massData.inertia.cy.z * w.y + massData.inertia.cz.z * w.z };

    return BoxToSource::Energy(0.5f * massData.mass * b3Dot(v, v) + 0.5f * b3Dot(w, Iw));
}

Vector Box3DPhysicsObject::GetMassCenterLocalSpace() const
{
    return BoxToSource::Distance(b3Body_GetLocalCenter(m_BodyId));
}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsObject::SetPosition(const Vector& worldPosition, const QAngle& angles, bool)
{
    b3Body_SetTransform(m_BodyId, SourceToBox::Distance(worldPosition), SourceToBox::Angle(angles));
}

void Box3DPhysicsObject::SetPositionMatrix(const matrix3x4_t& matrix, bool)
{
    const b3Transform xf = SourceToBox::Transform(matrix);
    b3Body_SetTransform(m_BodyId, xf.p, xf.q);
}

void Box3DPhysicsObject::GetPosition(Vector* worldPosition, QAngle* angles) const
{
    const b3WorldTransform xf = b3Body_GetTransform(m_BodyId);
    if (worldPosition)
        *worldPosition = BoxToSource::Distance(xf.p);
    if (angles)
        *angles = BoxToSource::Angle(xf.q);
}

void Box3DPhysicsObject::GetPositionMatrix(matrix3x4_t* positionMatrix) const
{
    if (positionMatrix)
        *positionMatrix = BoxToSource::Matrix(b3Body_GetTransform(m_BodyId));
}

void Box3DPhysicsObject::SetVelocity(const Vector* velocity, const AngularImpulse* angularVelocity)
{
    if (m_bStatic)
        return;
    // Never feed NaN/Inf into the solver — a bad velocity propagates to a bad position and the engine
    // deletes the entity.
    const bool bVel = velocity && velocity->IsValid();
    const bool bAng = angularVelocity && angularVelocity->IsValid();
    if (bVel)
        b3Body_SetLinearVelocity(m_BodyId, SourceToBox::Distance(*velocity));
    if (bAng)
    {
        Vector vecWorldAngular;
        LocalToWorldVector(&vecWorldAngular, *angularVelocity);
        b3Body_SetAngularVelocity(m_BodyId, SourceToBox::AngularImpulse(vecWorldAngular));
    }
    if (bVel || bAng)
        b3Body_SetAwake(m_BodyId, true);
}

void Box3DPhysicsObject::SetVelocityInstantaneous(const Vector* velocity, const AngularImpulse* angularVelocity)
{
    SetVelocity(velocity, angularVelocity);
}

void Box3DPhysicsObject::GetVelocity(Vector* velocity, AngularImpulse* angularVelocity) const
{
    // Never hand the game a NaN/Inf: it fails the entity's IsValid check and deletes it.
    if (velocity)
    {
        *velocity = BoxToSource::Distance(b3Body_GetLinearVelocity(m_BodyId));
        if (!velocity->IsValid())
            *velocity = vec3_origin;
    }
    if (angularVelocity)
    {
        // Explosion gibs get extreme spin; IVP clamps the core to PI/2 rad/tick and the game reads that
        // clamped value. Clamp here (where the game reads) to the same per-tick limit the step used, and
        // write it back so the body stays sane too.
        const float flMaxAngular = m_pEnvironment->GetMaxAngularVelocity();
        b3Vec3 w = b3Body_GetAngularVelocity(m_BodyId);
        const float flLen = sqrtf(b3Dot(w, w));
        if (flLen > flMaxAngular)
        {
            w = b3MulSV(flMaxAngular / flLen, w);
            if (!m_bStatic)
                b3Body_SetAngularVelocity(m_BodyId, w);
        }
        const Vector vecWorldAngular = BoxToSource::AngularImpulse(w);
        WorldToLocalVector(angularVelocity, vecWorldAngular);
        if (!angularVelocity->IsValid())
            *angularVelocity = vec3_origin;
    }
}

void Box3DPhysicsObject::SnapshotPreStepVelocity()
{
    m_vecPreStepVelocity = m_bStatic ? vec3_origin : BoxToSource::Distance(b3Body_GetLinearVelocity(m_BodyId));
}

Vector Box3DPhysicsObject::FakeVelocity(const Vector& vecVelocity)
{
    const Vector vecOld = BoxToSource::Distance(b3Body_GetLinearVelocity(m_BodyId));
    if (!m_bStatic)
        b3Body_SetLinearVelocity(m_BodyId, SourceToBox::Distance(vecVelocity));
    return vecOld;
}

void Box3DPhysicsObject::RestoreVelocity(const Vector& vecVelocity)
{
    if (!m_bStatic)
        b3Body_SetLinearVelocity(m_BodyId, SourceToBox::Distance(vecVelocity));
}

void Box3DPhysicsObject::AddVelocity(const Vector* velocity, const AngularImpulse* angularVelocity)
{
    if (m_bStatic)
        return;
    if (velocity)
        b3Body_SetLinearVelocity(m_BodyId, b3Add(b3Body_GetLinearVelocity(m_BodyId), SourceToBox::Distance(*velocity)));
    if (angularVelocity)
    {
        Vector vecWorldAngular;
        LocalToWorldVector(&vecWorldAngular, *angularVelocity);
        b3Body_SetAngularVelocity(
            m_BodyId, b3Add(b3Body_GetAngularVelocity(m_BodyId), SourceToBox::AngularImpulse(vecWorldAngular)));
    }
    if (velocity || angularVelocity)
        b3Body_SetAwake(m_BodyId, true);
}

void Box3DPhysicsObject::GetVelocityAtPoint(const Vector& worldPosition, Vector* pVelocity) const
{
    if (pVelocity)
        *pVelocity = BoxToSource::Distance(b3Body_GetWorldPointVelocity(m_BodyId, SourceToBox::Distance(worldPosition)));
}

void Box3DPhysicsObject::GetImplicitVelocity(Vector* velocity, AngularImpulse* angularVelocity) const
{
    GetVelocity(velocity, angularVelocity);
}

void Box3DPhysicsObject::LocalToWorld(Vector* worldPosition, const Vector& localPosition) const
{
    if (worldPosition)
        *worldPosition = BoxToSource::Distance(b3Body_GetWorldPoint(m_BodyId, SourceToBox::Distance(localPosition)));
}

void Box3DPhysicsObject::WorldToLocal(Vector* localPosition, const Vector& worldPosition) const
{
    if (localPosition)
        *localPosition = BoxToSource::Distance(b3Body_GetLocalPoint(m_BodyId, SourceToBox::Distance(worldPosition)));
}

void Box3DPhysicsObject::LocalToWorldVector(Vector* worldVector, const Vector& localVector) const
{
    if (worldVector)
        *worldVector = BoxToSource::Unitless(b3Body_GetWorldVector(m_BodyId, SourceToBox::Unitless(localVector)));
}

void Box3DPhysicsObject::WorldToLocalVector(Vector* localVector, const Vector& worldVector) const
{
    if (localVector)
        *localVector = BoxToSource::Unitless(b3Body_GetLocalVector(m_BodyId, SourceToBox::Unitless(worldVector)));
}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsObject::ApplyForceCenter(const Vector& forceVector)
{
    if (!m_bStatic)
        b3Body_ApplyLinearImpulseToCenter(m_BodyId, SourceToBox::Distance(forceVector), true);
}

void Box3DPhysicsObject::ApplyForceOffset(const Vector& forceVector, const Vector& worldPosition)
{
    if (!m_bStatic)
        b3Body_ApplyLinearImpulse(m_BodyId, SourceToBox::Distance(forceVector), SourceToBox::Distance(worldPosition), true);
}

void Box3DPhysicsObject::ApplyTorqueCenter(const AngularImpulse& torque)
{
    if (m_bStatic)
        return;

    // IVP applies this torque impulse in world space (async_rot_push_core_multiple_ws).
    b3Body_ApplyAngularImpulse(m_BodyId, SourceToBox::AngularImpulse(torque), true);
}

void Box3DPhysicsObject::CalculateForceOffset(
    const Vector& forceVector, const Vector& worldPosition, Vector* centerForce, AngularImpulse* centerTorque) const
{
    if (centerForce)
        *centerForce = forceVector;
    if (centerTorque)
    {
        Vector com;
        com = BoxToSource::Distance(b3Body_GetWorldCenter(m_BodyId));
        const Vector worldTorque = CrossProduct(worldPosition - com, forceVector);
        WorldToLocalVector(centerTorque, worldTorque);
    }
}

void Box3DPhysicsObject::CalculateVelocityOffset(
    const Vector& forceVector, const Vector& worldPosition, Vector* centerVelocity, AngularImpulse* centerAngularVelocity) const
{
    if (centerVelocity)
        *centerVelocity = forceVector * GetInvMass();
    if (centerAngularVelocity)
    {
        Vector centerTorque;
        CalculateForceOffset(forceVector, worldPosition, nullptr, &centerTorque);
        const Vector invInertia = GetInvInertia();
        *centerAngularVelocity = Vector(
            centerTorque.x * invInertia.x, centerTorque.y * invInertia.y, centerTorque.z * invInertia.z);
    }
}

float Box3DPhysicsObject::CalculateLinearDrag(const Vector&) const
{
    return 0.0f;
}
float Box3DPhysicsObject::CalculateAngularDrag(const Vector&) const
{
    return 0.0f;
}

bool Box3DPhysicsObject::GetContactPoint(Vector*, IPhysicsObject**) const
{
    return false;
}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsObject::SetShadow(float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation)
{
    if (!m_pShadowController)
        m_pShadowController = static_cast<Box3DPhysicsShadowController*>(
            m_pEnvironment->CreateShadowController(this, allowPhysicsMovement, allowPhysicsRotation));
    m_pShadowController->MaxSpeed(maxSpeed, maxAngularSpeed);
}

void Box3DPhysicsObject::UpdateShadow(
    const Vector& targetPosition, const QAngle& targetAngles, bool tempDisableGravity, float timeOffset)
{
    if (m_pShadowController)
        m_pShadowController->Update(targetPosition, targetAngles, timeOffset);
}

int Box3DPhysicsObject::GetShadowPosition(Vector* position, QAngle* angles) const
{
    GetPosition(position, angles);
    return 1;
}
IPhysicsShadowController* Box3DPhysicsObject::GetShadowController() const
{
    return m_pShadowController;
}

void Box3DPhysicsObject::RemoveShadowController()
{
    if (m_pShadowController)
    {
        m_pEnvironment->DestroyShadowController(m_pShadowController);
        m_pShadowController = nullptr;
    }
}

// The engine's grab/shadow (physgun, +use pickup, doors) drives a held object toward a target through
// this every step: compute a velocity toward the target position/rotation and set it on the body.
// Remove the components of a shadow-driven velocity that point into static geometry. Box3D's solver
// can't fully cancel a hard-set velocity aimed at a contact under sustained drive, so without this the
// physgun/carry can force the held object through walls. Only static blockers are clamped, so pushing
// or lifting dynamic objects still works.
static void ClampShadowVelocityAgainstContacts(const Box3DPhysicsObject* pSelf, b3BodyId bodyId, Vector& velocity)
{
    if (velocity == vec3_origin)
        return;

    b3ContactData contacts[16];
    const int nCount = b3Body_GetContactData(bodyId, contacts, 16);
    for (int i = 0; i < nCount; i++)
    {
        Box3DPhysicsObject* pA = static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(b3Shape_GetBody(contacts[i].shapeIdA)));
        const bool bSelfIsA = (pA == pSelf);
        Box3DPhysicsObject* pOther = bSelfIsA
            ? static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(b3Shape_GetBody(contacts[i].shapeIdB)))
            : pA;
        if (!pOther || !pOther->IsStatic())
            continue;

        for (int j = 0; j < contacts[i].manifoldCount; j++)
        {
            const b3Manifold& manifold = contacts[i].manifolds[j];
            if (manifold.pointCount <= 0)
                continue;

            // Manifold normal points A -> B; orient it from the shadow object into the blocker.
            Vector vNormal = BoxToSource::Unitless(manifold.normal);
            if (!bSelfIsA)
                vNormal = -vNormal;

            const float flInto = DotProduct(velocity, vNormal);
            if (flInto > 0.0f)
                velocity -= vNormal * flInto;
        }
    }
}

float Box3DPhysicsObject::ComputeShadowControl(
    const hlshadowcontrol_params_t& params, float flSecondsToArrival, float flDeltaTime)
{
    Vector position;
    QAngle angles;
    GetPosition(&position, &angles);

    // The servo math below runs in world space; use raw body velocities rather than the
    // interface's object-space angular values.
    Vector linearVelocity = BoxToSource::Distance(b3Body_GetLinearVelocity(m_BodyId));
    AngularImpulse angularVelocity = BoxToSource::AngularImpulse(b3Body_GetAngularVelocity(m_BodyId));

    const float flFraction = flSecondsToArrival > 0.0f ? Min(flDeltaTime / flSecondsToArrival, 1.0f) : 1.0f;
    flSecondsToArrival = Max(flSecondsToArrival - flDeltaTime, 0.0f);

    if (flFraction <= 0.0f)
        return flSecondsToArrival;

    Vector deltaPosition = params.targetPosition - position;

    bool bTeleport = false;
    if (params.teleportDistance > 0.0f && deltaPosition.LengthSqr() > Square(params.teleportDistance))
    {
        position = params.targetPosition;
        angles = params.targetRotation;
        deltaPosition = vec3_origin;
        bTeleport = true;
    }

    const float flFractionTime = flFraction / flDeltaTime;

    ShadowComputeVelocity(
        linearVelocity, deltaPosition, params.maxSpeed, params.maxDampSpeed, flFractionTime, params.dampFactor);

    const Vector deltaAngles = ShadowRotationDeltaDegrees(angles, params.targetRotation);
    ShadowComputeVelocity(
        angularVelocity, deltaAngles, params.maxAngular, params.maxDampAngular, flFractionTime, params.dampFactor);

    if (bTeleport)
    {
        if (IsCollisionEnabled())
        {
            EnableCollisions(false);
            SetPosition(position, angles, true);
            EnableCollisions(true);
        }
        else
        {
            SetPosition(position, angles, true);
        }
    }

    if (!m_bStatic)
    {
        ClampShadowVelocityAgainstContacts(this, m_BodyId, linearVelocity);
        b3Body_SetLinearVelocity(m_BodyId, SourceToBox::Distance(linearVelocity));
        b3Body_SetAngularVelocity(m_BodyId, SourceToBox::AngularImpulse(angularVelocity));
        b3Body_SetAwake(m_BodyId, true);
    }

    return flSecondsToArrival;
}

const CPhysCollide* Box3DPhysicsObject::GetCollide() const
{
    return m_pCollide;
}
const char* Box3DPhysicsObject::GetName() const
{
    return m_pName;
}

// A trigger (e.g. water) is non-solid: drop collision response so bodies pass through and queries find them.
void Box3DPhysicsObject::BecomeTrigger()
{
    m_bTrigger = true;
    EnableCollisions(false);
}
void Box3DPhysicsObject::RemoveTrigger()
{
    m_bTrigger = false;
    EnableCollisions(true);
}
void Box3DPhysicsObject::BecomeHinged(int)
{
    Log_Stub(LOG_VBox3D);
}
void Box3DPhysicsObject::RemoveHinged()
{
    Log_Stub(LOG_VBox3D);
}

IPhysicsFrictionSnapshot* Box3DPhysicsObject::CreateFrictionSnapshot()
{
    return new Box3DFrictionSnapshot(this, m_pEnvironment->GetLastStepTime());
}
void Box3DPhysicsObject::DestroyFrictionSnapshot(IPhysicsFrictionSnapshot* pSnapshot)
{
    delete static_cast<Box3DFrictionSnapshot*>(pSnapshot);
}

void Box3DPhysicsObject::OutputDebugInfo() const
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsObject::SetUseAlternateGravity(bool)
{
}
void Box3DPhysicsObject::SetCollisionHints(uint32 collisionHints)
{
    m_collisionHints = collisionHints;
}
uint32 Box3DPhysicsObject::GetCollisionHints() const
{
    return m_collisionHints;
}

IPredictedPhysicsObject* Box3DPhysicsObject::GetPredictedInterface() const
{
    return nullptr;
}
void Box3DPhysicsObject::SyncWith(IPhysicsObject*)
{
}
