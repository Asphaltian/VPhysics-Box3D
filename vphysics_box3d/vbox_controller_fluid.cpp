
#include "vbox_controller_fluid.h"

#include "cbase.h"
#include "vbox_collide.h"
#include "vbox_environment.h"
#include "vbox_object.h"
#include "vbox_surfaceprops.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------
// Fluid controller: water buoyancy, drag and current. No fluid model in Box3D, so each step we find the
// bodies overlapping the fluid volume and apply buoyancy. Submerged volume from the body AABB.
//-------------------------------------------------------------------------------------------------

// Store the world-space surface plane in the fluid object's local space so it tracks the object.
static cplane_t PlaneToLocalSpace(Box3DPhysicsObject* pFluidObject, const Vector4D& worldSurfacePlane)
{
    const cplane_t worldPlane = { worldSurfacePlane.AsVector3D(), worldSurfacePlane[3] };

    matrix3x4_t objectToWorld;
    pFluidObject->GetPositionMatrix(&objectToWorld);

    cplane_t localPlane;
    MatrixITransformPlane(objectToWorld, worldPlane, localPlane);
    return localPlane;
}

// Object volume (m^3) and tight world AABB. Uses the pristine collision hulls, not the inflated sim hulls
// (which over-report volume), and builds the AABB here since b3Body_ComputeAABB returns the fat one.
static float ComputeBodyBuoyancy(Box3DPhysicsObject* pObject, b3AABB* pAABB)
{
    const b3BodyId bodyId = pObject->GetBodyID();
    const b3WorldTransform wt = b3Body_GetTransform(bodyId);
    const b3Transform xf = { b3ToVec3(wt.p), wt.q };

    float flVolume = 0.0f;
    bool bHasBounds = false;
    b3AABB bounds = {};

    const CPhysCollide* pCollide = pObject->GetCollide();
    if (pCollide)
    {
        for (int i = 0; i < pCollide->m_Convexes.Count(); i++)
        {
            const b3HullData* pHull = pCollide->m_Convexes[i]->m_pHull;
            if (!pHull)
                continue;
            flVolume += b3ComputeHullMass(pHull, 1.0f).mass;
            const b3AABB hullAABB = b3ComputeHullAABB(pHull, xf);
            bounds = bHasBounds ? b3AABB_Union(bounds, hullAABB) : hullAABB;
            bHasBounds = true;
        }
    }

    // Spheres (no CPhysCollide) fall back to the body's own shapes.
    if (!bHasBounds)
    {
        const int nCount = b3Body_GetShapeCount(bodyId);
        CUtlVector<b3ShapeId> shapes;
        shapes.SetCount(nCount);
        b3Body_GetShapes(bodyId, shapes.Base(), nCount);
        for (int i = 0; i < nCount; i++)
        {
            if (b3Shape_GetType(shapes[i]) != b3_sphereShape)
                continue;
            const b3Sphere sphere = b3Shape_GetSphere(shapes[i]);
            flVolume += (4.0f / 3.0f) * 3.14159265f * sphere.radius * sphere.radius * sphere.radius;
            const b3Vec3 center = b3TransformPoint(xf, sphere.center);
            const b3Vec3 rad = { sphere.radius, sphere.radius, sphere.radius };
            const b3AABB sphAABB = { b3Sub(center, rad), b3Add(center, rad) };
            bounds = bHasBounds ? b3AABB_Union(bounds, sphAABB) : sphAABB;
            bHasBounds = true;
        }
    }

    if (pAABB)
        *pAABB = bounds;
    return flVolume;
}

namespace
{
    struct FluidQuery
    {
        Box3DPhysicsObject* pFluidObject;
        CUtlVector<Box3DPhysicsObject*>* pOut;
    };

    // Collect the distinct dynamic bodies overlapping the fluid's AABB.
    bool FluidOverlapFcn(b3ShapeId shapeId, void* context)
    {
        FluidQuery* pQuery = static_cast<FluidQuery*>(context);
        Box3DPhysicsObject* pObject = static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(b3Shape_GetBody(shapeId)));
        if (pObject && pObject != pQuery->pFluidObject && !pObject->IsStatic()
            && pQuery->pOut->Find(pObject) == pQuery->pOut->InvalidIndex())
            pQuery->pOut->AddToTail(pObject);
        return true;
    }
} // namespace

Box3DPhysicsFluidController::Box3DPhysicsFluidController(Box3DPhysicsObject* pFluidObject, const fluidparams_t* pParams)
    : m_pFluidObject(pFluidObject)
    , m_Params(*pParams)
    , m_LocalPlane(PlaneToLocalSpace(pFluidObject, pParams->surfacePlane))
{
    m_pFluidObject->BecomeTrigger();
}

Box3DPhysicsFluidController::~Box3DPhysicsFluidController()
{
    if (m_pFluidObject)
        m_pFluidObject->RemoveTrigger();
}

void Box3DPhysicsFluidController::SetGameData(void* pGameData)
{
    m_Params.pGameData = pGameData;
}
void* Box3DPhysicsFluidController::GetGameData() const
{
    return m_Params.pGameData;
}
int Box3DPhysicsFluidController::GetContents() const
{
    return m_Params.contents;
}

void Box3DPhysicsFluidController::GetSurfacePlane(Vector* pNormal, float* pDist) const
{
    const cplane_t worldPlane = GetWorldSurfacePlane();
    if (pNormal)
        *pNormal = worldPlane.normal;
    if (pDist)
        *pDist = worldPlane.dist;
}

// Water density in kg/m^3 from surfaceprops, defaulting to 1000 if unset.
float Box3DPhysicsFluidController::GetDensity() const
{
    surfacedata_t* pSurface = m_pFluidObject
        ? Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData(m_pFluidObject->GetMaterialIndex())
        : nullptr;
    return (pSurface && pSurface->physics.density > 0.0f) ? pSurface->physics.density : 1000.0f;
}

void Box3DPhysicsFluidController::WakeAllSleepingObjects()
{
    for (int i = 0; i < m_ObjectsInFluid.Count(); i++)
        m_ObjectsInFluid[i]->Wake();
}

void Box3DPhysicsFluidController::DetachObject(Box3DPhysicsObject* pObject)
{
    if (pObject == m_pFluidObject)
        m_pFluidObject = nullptr;
    m_ObjectsInFluid.FindAndRemove(pObject);
}

cplane_t Box3DPhysicsFluidController::GetWorldSurfacePlane() const
{
    if (!m_pFluidObject)
        return cplane_t{};

    matrix3x4_t objectToWorld;
    m_pFluidObject->GetPositionMatrix(&objectToWorld);

    cplane_t worldPlane;
    MatrixTransformPlane(objectToWorld, m_LocalPlane, worldPlane);
    return worldPlane;
}

void Box3DPhysicsFluidController::OnPreSimulate(float flDeltaTime)
{
    if (!m_pFluidObject || flDeltaTime <= 0.0f)
        return;

    const b3WorldId worldId = m_pFluidObject->GetEnvironment()->GetWorldId();

    CUtlVector<Box3DPhysicsObject*> overlapping;
    FluidQuery query = { m_pFluidObject, &overlapping };
    b3World_OverlapAABB(
        worldId, b3Body_ComputeAABB(m_pFluidObject->GetBodyID()), b3DefaultQueryFilter(), FluidOverlapFcn, &query);

    // The bodies actually below the surface this step; diffed against m_ObjectsInFluid for touch events.
    CUtlVector<Box3DPhysicsObject*> nowInFluid;

    const cplane_t worldPlane = GetWorldSurfacePlane();
    const b3Vec3 n = SourceToBox::Unitless(worldPlane.normal);        // unit, points up out of the water
    const float flPlaneDist = SourceToBox::Distance(worldPlane.dist); // water is where dot(n,x) < planeDist
    const b3Vec3 gravity = b3World_GetGravity(worldId);
    const b3Vec3 current = SourceToBox::Distance(m_Params.currentVelocity);
    const float flWaterDensity = GetDensity();
    const float flLinearDrag = m_Params.damping;
    const float flAngularDrag = 0.1f;

    for (int i = 0; i < overlapping.Count(); i++)
    {
        Box3DPhysicsObject* pObject = overlapping[i];
        if (pObject->GetShadowController() || !(pObject->GetCallbackFlags() & CALLBACK_DO_FLUID_SIMULATION))
            continue;

        const b3BodyId body = pObject->GetBodyID();
        const float flInvMass = b3Body_GetInverseMass(body);
        b3AABB aabb;
        const float flTotalVolume = ComputeBodyBuoyancy(pObject, &aabb);
        if (flInvMass <= 0.0f || flTotalVolume <= 0.0f)
            continue;

        // Submerged fraction of the AABB along the surface normal.
        const b3Vec3 c = b3MulSV(0.5f, b3Add(aabb.lowerBound, aabb.upperBound));
        const b3Vec3 half = b3MulSV(0.5f, b3Sub(aabb.upperBound, aabb.lowerBound));
        const float flR = fabsf(n.x) * half.x + fabsf(n.y) * half.y + fabsf(n.z) * half.z;
        const float flCN = b3Dot(n, c);
        const float flLo = flCN - flR, flHi = flCN + flR;
        if (flHi <= flLo)
            continue;
        const float flTop = Min(flHi, flPlaneDist);
        const float flSubLen = clamp(flTop - flLo, 0.0f, flHi - flLo);
        if (flSubLen <= 0.0f)
            continue;

        nowInFluid.AddToTail(pObject);

        const float flFraction = flSubLen / (flHi - flLo);
        const float flSubmergedVolume = flTotalVolume * flFraction;

        // Centre of buoyancy: midpoint of the submerged span, relative to the centre of mass.
        const float flMidSub = 0.5f * (flLo + flTop);
        const b3Vec3 cob = b3Add(c, b3MulSV(flMidSub - flCN, n));
        const b3Vec3 relCob = b3Sub(cob, b3Body_GetWorldCenter(body));

        const float flFluidDensity = flWaterDensity * pObject->BuoyancyRatio();

        // Buoyancy = -fluidDensity * submergedVolume * gravity * dt.
        const b3Vec3 buoyImpulse = b3MulSV(-flFluidDensity * flSubmergedVolume * flDeltaTime, gravity);

        // Velocity of the centre of buoyancy relative to the fluid.
        const b3Vec3 linVel = b3Body_GetLinearVelocity(body);
        const b3Vec3 angVel = b3Body_GetAngularVelocity(body);
        const b3Vec3 relVel = b3Sub(current, b3Add(linVel, b3Cross(angVel, relCob)));

        // Quadratic drag; frontal area from the AABB.
        const b3Vec3 size = b3Sub(aabb.upperBound, aabb.lowerBound);
        const float flRelSq = b3Dot(relVel, relVel);
        b3Vec3 dragImpulse = { 0.0f, 0.0f, 0.0f };
        if (flRelSq > 1e-12f)
        {
            const float flArea = (fabsf(relVel.x) * size.y * size.z + fabsf(relVel.y) * size.z * size.x
                                  + fabsf(relVel.z) * size.x * size.y)
                / sqrtf(flRelSq);
            dragImpulse = b3MulSV(0.5f * flFluidDensity * flLinearDrag * flArea * flDeltaTime * sqrtf(flRelSq), relVel);

            // Clamp so drag can't reverse the velocity.
            const float flLinVelSq = b3Dot(linVel, linVel);
            const b3Vec3 dragDv = b3MulSV(flInvMass, dragImpulse);
            const float flDragDvSq = b3Dot(dragDv, dragDv);
            if (flDragDvSq > flLinVelSq && flDragDvSq > 0.0f)
                dragImpulse = b3MulSV(sqrtf(flLinVelSq / flDragDvSq), dragImpulse);
        }

        // wake=false: buoyancy cancels gravity at rest, so forcing wake would stop props ever sleeping.
        const b3Vec3 linImpulse = b3Add(buoyImpulse, dragImpulse);
        b3Body_ApplyLinearImpulseToCenter(body, linImpulse, false);
        b3Body_ApplyAngularImpulse(body, b3Cross(relCob, linImpulse), false);

        // Angular drag: -angularDrag * fraction * dt * avgWidth^2 * mass * angVel.
        const float flL = (size.x + size.y + size.z) / 3.0f;
        b3Body_ApplyAngularImpulse(
            body, b3MulSV(-flAngularDrag * flFraction * flDeltaTime * flL * flL / flInvMass, angVel), false);
    }

    // Fluid touch events: diff this step's submerged set against last step's.
    if (IPhysicsCollisionEvent* pEvent = m_pFluidObject->GetEnvironment()->GetCollisionEvent())
    {
        for (int i = 0; i < nowInFluid.Count(); i++)
            if (m_ObjectsInFluid.Find(nowInFluid[i]) == m_ObjectsInFluid.InvalidIndex())
                pEvent->FluidStartTouch(nowInFluid[i], this);
        for (int i = 0; i < m_ObjectsInFluid.Count(); i++)
            if (nowInFluid.Find(m_ObjectsInFluid[i]) == nowInFluid.InvalidIndex())
                pEvent->FluidEndTouch(m_ObjectsInFluid[i], this);
    }

    m_ObjectsInFluid.RemoveAll();
    m_ObjectsInFluid.AddVectorToTail(nowInFluid);
}
