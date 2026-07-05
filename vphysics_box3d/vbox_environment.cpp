//=================================================================================================
//
// Interface to a physics scene
//
//=================================================================================================

#include "vbox_environment.h"

#include "cbase.h"
#include "tier0/memdbgon.h"
#include "tier0/threadtools.h"
#include "tier1/convar.h"
#include "vbox_collide.h"
#include "vbox_constraints.h"
#include "vbox_controller_fluid.h"
#include "vbox_controller_motion.h"
#include "vbox_controller_player.h"
#include "vbox_controller_shadow.h"
#include "vbox_object.h"
#include "vbox_surfaceprops.h"

static ConVar vbox_substeps("vbox_substeps", "16", FCVAR_NONE, "Solver substeps per physics step.", true, 1.0f, true, 1024.0f);
static ConVar vbox_contact_hertz(
    "vbox_contact_hertz", "240", FCVAR_NONE, "Contact stiffness in Hz. Lower is softer/smushier.", true, 1.0f, true, 480.0f);
static ConVar vbox_contact_damping(
    "vbox_contact_damping", "10", FCVAR_NONE, "Contact damping ratio. Higher settles overlap with less bounce.", true, 0.0f,
    true, 100.0f);
static ConVar vbox_contact_speed(
    "vbox_contact_speed", "400", FCVAR_NONE, "Overlap push-out speed cap in in/s.", true, 0.0f, true, 1000.0f);

namespace
{
    // Minimum time between collision events for the same object pair (IVP's deltaCollisionTime gate).
    constexpr float kCollisionEventInterval = 0.2f;

    // Apply a Source surface's friction/bounce/density to a shape. Box3D combines both shapes on contact.
    void ApplyMaterialToShape(b3ShapeDef& shapeDef, int materialIndex)
    {
        surfacedata_t* pSurface = Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData(materialIndex);
        if (!pSurface)
            return;

        shapeDef.baseMaterial.friction = Max(pSurface->physics.friction, 0.0f);
        // Raw surfaceprop elasticity (can be far above 1, e.g. Metal_bouncy = 1000); the combined
        // product is clamped in Box3DRestitutionCombine, matching IVP's get_elasticity.
        shapeDef.baseMaterial.restitution = pSurface->physics.elasticity;
        if (pSurface->physics.density > 0.0f)
            shapeDef.density = pSurface->physics.density; // kg/m^3 in both, geometry is in metres
    }

    // IVP combines both surfaces' coefficients as a product; Box3D defaults to
    // max(restitution) and sqrt(friction), which makes props far too bouncy and slightly too grippy.
    float Box3DFrictionCombine(float a, uint64_t, float b, uint64_t)
    {
        return a * b;
    }
    float Box3DRestitutionCombine(float a, uint64_t, float b, uint64_t)
    {
        return clamp(a * b, 0.0f, 1.0f);
    }

    // These callbacks run on Box3D worker threads; the game's solver isn't thread-safe.
    CThreadFastMutex g_CollisionSolverMutex;

    // Ask the game's solver whether two shapes' objects may collide (collision groups, no-collide, debris).
    bool ShapesCollide(void* context, b3ShapeId shapeA, b3ShapeId shapeB)
    {
        if (!b3Shape_IsValid(shapeA) || !b3Shape_IsValid(shapeB))
            return true;

        Box3DPhysicsObject* pA = static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(b3Shape_GetBody(shapeA)));
        Box3DPhysicsObject* pB = static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(b3Shape_GetBody(shapeB)));
        if (!pA || !pB)
            return true;

        if (!pA->IsCollisionEnabled() || !pB->IsCollisionEnabled())
            return false;
        if ((pA->GetCallbackFlags() | pB->GetCallbackFlags()) & CALLBACK_MARKED_FOR_DELETE)
            return false;

        IPhysicsCollisionSolver* pSolver = static_cast<Box3DPhysicsEnvironment*>(context)->GetCollisionSolver();
        if (!pSolver)
            return true;

        AUTO_LOCK(g_CollisionSolverMutex);
        return pSolver->ShouldCollide(pA, pB, pA->GetGameData(), pB->GetGameData()) != 0;
    }

    // Broadphase filter (new pairs) and pre-solve (existing pairs, so a runtime no-collide takes effect).
    bool Box3DCustomFilter(b3ShapeId a, b3ShapeId b, void* ctx)
    {
        return ShapesCollide(ctx, a, b);
    }
    bool Box3DPreSolve(b3ShapeId a, b3ShapeId b, b3Pos, b3Vec3, void* ctx)
    {
        return ShapesCollide(ctx, a, b);
    }
} // namespace

Box3DPhysicsEnvironment::Box3DPhysicsEnvironment()
{
    m_PerformanceParams.Defaults();

    b3WorldDef def = b3DefaultWorldDef();
    // Only hit events for impacts >= 70 in/s (Source's collision-sound threshold).
    def.hitEventThreshold = SourceToBox::Distance(70.0f);
    // Cap at sv_maxvelocity so a solver blow-up can't fling an object to an invalid (deleted) coordinate.
    def.maximumLinearSpeed = SourceToBox::Distance(3500.0f);
    def.enableContinuous = true;
    // Penetration push-out cap: gentler than Box3D's 3 m/s default, but not so low ragdoll limbs wedge.
    def.contactSpeed = SourceToBox::Distance(100.0f);
    // workerCount > 1 with no task callbacks runs Box3D's built-in scheduler; physical cores only, HT hurts.
    def.workerCount = (uint32_t)clamp((int)GetCPUInformation()->m_nPhysicalProcessors, 1, B3_MAX_WORKERS);
    m_WorldId = b3CreateWorld(&def);

    // Match IVP's product combine rule for both coefficients (not Box3D's max/sqrt defaults).
    b3World_SetFrictionCallback(m_WorldId, Box3DFrictionCombine);
    b3World_SetRestitutionCallback(m_WorldId, Box3DRestitutionCombine);
    // Route the game's per-pair collision rules (ShouldCollide) into both the broadphase and the solver.
    b3World_SetCustomFilterCallback(m_WorldId, Box3DCustomFilter, this);
    b3World_SetPreSolveCallback(m_WorldId, Box3DPreSolve, this);
}

Box3DPhysicsEnvironment::~Box3DPhysicsEnvironment()
{
    CleanupDeleteList();
    b3DestroyWorld(m_WorldId);
}

void Box3DPhysicsEnvironment::SetDebugOverlay(CreateInterfaceFn debugOverlayFactory)
{
    Log_Stub(LOG_VBox3D);
}

IVPhysicsDebugOverlay* Box3DPhysicsEnvironment::GetDebugOverlay(void)
{
    Log_Stub(LOG_VBox3D);
    return nullptr;
}

void Box3DPhysicsEnvironment::SetGravity(const Vector& gravityVector)
{
    m_vecGravity = gravityVector;
    b3World_SetGravity(m_WorldId, SourceToBox::Distance(gravityVector));
}

void Box3DPhysicsEnvironment::GetGravity(Vector* pGravityVector) const
{
    *pGravityVector = m_vecGravity;
}

void Box3DPhysicsEnvironment::SetAirDensity(float density)
{
    m_flAirDensity = density;
}

float Box3DPhysicsEnvironment::GetAirDensity() const
{
    return m_flAirDensity;
}

IPhysicsObject* Box3DPhysicsEnvironment::CreateObject(
    const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles,
    objectparams_t* pParams, bool bStatic)
{
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = bStatic ? b3_staticBody : b3_dynamicBody;
    bodyDef.position = SourceToBox::Distance(position);
    bodyDef.rotation = SourceToBox::Angle(angles);
    bodyDef.isAwake = false;
    // 2x Box3D's default so piles settle and sleep instead of jittering awake.
    bodyDef.sleepThreshold = SourceToBox::Distance(4.0f);

    const b3BodyId bodyId = b3CreateBody(m_WorldId, &bodyDef);

    if (pCollisionModel)
    {
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.enableContactEvents = true;
        shapeDef.enableHitEvents = true;
        // Required for the ShouldCollide filter/pre-solve callbacks to fire.
        shapeDef.enableCustomFiltering = true;
        shapeDef.enablePreSolveEvents = true;
        ApplyMaterialToShape(shapeDef, materialIndex);
        for (int i = 0; i < pCollisionModel->m_Convexes.Count(); i++)
        {
            CPhysConvex* pConvex = pCollisionModel->m_Convexes[i];
            if (!pConvex->m_pHull)
                continue;
            // Dynamic bodies get the rest-margin-inflated hull (props rest proud); static geometry stays pristine.
            b3CreateHullShape(bodyId, &shapeDef, bStatic ? pConvex->m_pHull : pConvex->GetSimHull());
        }

        if (pCollisionModel->m_pMesh)
            b3CreateMeshShape(bodyId, &shapeDef, pCollisionModel->m_pMesh, b3Vec3{ 1.0f, 1.0f, 1.0f });
    }

    if (!bStatic && pParams && pParams->massCenterOverride && *pParams->massCenterOverride != vec3_origin)
    {
        b3MassData massData = b3Body_GetMassData(bodyId);
        massData.center = SourceToBox::Distance(*pParams->massCenterOverride);
        b3Body_SetMassData(bodyId, massData);
    }

    Box3DPhysicsObject* pObject = new Box3DPhysicsObject(bodyId, this, bStatic, materialIndex, pCollisionModel, pParams);
    m_Objects.AddToTail(pObject);
    return pObject;
}

IPhysicsObject* Box3DPhysicsEnvironment::CreatePolyObject(
    const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles,
    objectparams_t* pParams)
{
    return CreateObject(pCollisionModel, materialIndex, position, angles, pParams, false);
}

IPhysicsObject* Box3DPhysicsEnvironment::CreatePolyObjectStatic(
    const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles,
    objectparams_t* pParams)
{
    return CreateObject(pCollisionModel, materialIndex, position, angles, pParams, true);
}

IPhysicsObject* Box3DPhysicsEnvironment::CreateSphereObject(
    float radius, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams, bool isStatic)
{
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = isStatic ? b3_staticBody : b3_dynamicBody;
    bodyDef.position = SourceToBox::Distance(position);
    bodyDef.rotation = SourceToBox::Angle(angles);
    bodyDef.isAwake = false;
    bodyDef.sleepThreshold = SourceToBox::Distance(4.0f);

    const b3BodyId bodyId = b3CreateBody(m_WorldId, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.enableContactEvents = true;
    shapeDef.enableHitEvents = true;
    shapeDef.enableCustomFiltering = true;
    shapeDef.enablePreSolveEvents = true;
    ApplyMaterialToShape(shapeDef, materialIndex);
    b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, SourceToBox::Distance(radius) };
    b3CreateSphereShape(bodyId, &shapeDef, &sphere);

    Box3DPhysicsObject* pObject = new Box3DPhysicsObject(bodyId, this, isStatic, materialIndex, nullptr, pParams);
    pObject->SetSphereRadius(radius);
    m_Objects.AddToTail(pObject);
    return pObject;
}

void Box3DPhysicsEnvironment::DestroyObject(IPhysicsObject* pObject)
{
    if (!pObject)
        return;

    Box3DPhysicsObject* pBoxObject = static_cast<Box3DPhysicsObject*>(pObject);

    if (pBoxObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)
    {
        AssertMsg(false, "Object deleted twice.\n");
        return;
    }

    // Drop any controllers referencing this object while its body is still valid.
    pBoxObject->RemoveShadowController();
    for (int i = 0; i < m_MotionControllers.Count(); i++)
        m_MotionControllers[i]->DetachObject(pBoxObject);
    for (int i = 0; i < m_PlayerControllers.Count(); i++)
    {
        if (m_PlayerControllers[i]->GetControlledObject() == pBoxObject)
            m_PlayerControllers[i]->SetObject(nullptr);
        m_PlayerControllers[i]->ClearGround(pBoxObject);
    }
    for (int i = 0; i < m_FluidControllers.Count(); i++)
        m_FluidControllers[i]->DetachObject(pBoxObject);
    // Break constraints and springs on this object so their getters can't return a freed pointer.
    for (int i = 0; i < m_Constraints.Count(); i++)
        m_Constraints[i]->NotifyObjectDestroyed(pBoxObject);
    for (int i = 0; i < m_Springs.Count(); i++)
        m_Springs[i]->NotifyObjectDestroyed(pBoxObject);

    m_ActiveObjects.FindAndRemove(pBoxObject);

    pBoxObject->SetCallbackFlags(pBoxObject->GetCallbackFlags() | CALLBACK_MARKED_FOR_DELETE);

    // While the delete queue is on, keep the wrapper alive and in m_Objects so pending references stay
    // valid -- GMod validates queued damage-event inflictors by GetObjectList() membership.
    if (m_bInSimulation || m_bDeleteQueueEnabled)
    {
        m_DeadObjects.AddToTail(pBoxObject);
        return;
    }

    DeleteObject(pBoxObject);
}

void Box3DPhysicsEnvironment::DeleteObject(Box3DPhysicsObject* pObject)
{
    m_Objects.FindAndRemove(pObject);
    b3DestroyBody(pObject->GetBodyID());
    delete pObject;
}

IPhysicsFluidController* Box3DPhysicsEnvironment::CreateFluidController(IPhysicsObject* pFluidObject, fluidparams_t* pParams)
{
    Box3DPhysicsFluidController* pController = new Box3DPhysicsFluidController(
        static_cast<Box3DPhysicsObject*>(pFluidObject), pParams);
    m_FluidControllers.AddToTail(pController);
    return pController;
}

void Box3DPhysicsEnvironment::DestroyFluidController(IPhysicsFluidController* pController)
{
    Box3DPhysicsFluidController* pFluid = static_cast<Box3DPhysicsFluidController*>(pController);
    m_FluidControllers.FindAndRemove(pFluid);
    delete pFluid;
}

IPhysicsShadowController* Box3DPhysicsEnvironment::CreateShadowController(
    IPhysicsObject* pObject, bool allowTranslation, bool allowRotation)
{
    Box3DPhysicsShadowController* pController = new Box3DPhysicsShadowController(
        static_cast<Box3DPhysicsObject*>(pObject), allowTranslation, allowRotation);
    m_ShadowControllers.AddToTail(pController);
    return pController;
}

void Box3DPhysicsEnvironment::DestroyShadowController(IPhysicsShadowController* pController)
{
    Box3DPhysicsShadowController* pShadow = static_cast<Box3DPhysicsShadowController*>(pController);
    m_ShadowControllers.FindAndRemove(pShadow);
    delete pShadow;
}

IPhysicsPlayerController* Box3DPhysicsEnvironment::CreatePlayerController(IPhysicsObject* pObject)
{
    Box3DPhysicsPlayerController* pController = new Box3DPhysicsPlayerController(static_cast<Box3DPhysicsObject*>(pObject));
    m_PlayerControllers.AddToTail(pController);
    return pController;
}

void Box3DPhysicsEnvironment::DestroyPlayerController(IPhysicsPlayerController* pController)
{
    Box3DPhysicsPlayerController* pPlayer = static_cast<Box3DPhysicsPlayerController*>(pController);
    m_PlayerControllers.FindAndRemove(pPlayer);
    delete pPlayer;
}

IPhysicsMotionController* Box3DPhysicsEnvironment::CreateMotionController(IMotionEvent* pHandler)
{
    Box3DPhysicsMotionController* pController = new Box3DPhysicsMotionController(pHandler);
    m_MotionControllers.AddToTail(pController);
    return pController;
}

void Box3DPhysicsEnvironment::DestroyMotionController(IPhysicsMotionController* pController)
{
    Box3DPhysicsMotionController* pMotion = static_cast<Box3DPhysicsMotionController*>(pController);
    m_MotionControllers.FindAndRemove(pMotion);
    delete pMotion;
}

IPhysicsVehicleController* Box3DPhysicsEnvironment::CreateVehicleController(
    IPhysicsObject* pVehicleBodyObject, const vehicleparams_t& params, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace)
{
    Log_Stub(LOG_VBox3D);
    return nullptr;
}

void Box3DPhysicsEnvironment::DestroyVehicleController(IPhysicsVehicleController*)
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::SetCollisionSolver(IPhysicsCollisionSolver* pSolver)
{
    m_pCollisionSolver = pSolver;
}

void Box3DPhysicsEnvironment::Simulate(float deltaTime)
{
    if (deltaTime <= 0.0f)
        return;

    m_flLastStepTime = deltaTime;

    CleanupDeleteList();

    m_bInSimulation = true;

    // Drive game-controlled objects before stepping: pickup/physgun/doors (shadow), the player,
    // and the gravity-gun grab (motion) each nudge their objects toward the game's target.
    for (int i = 0; i < m_ShadowControllers.Count(); i++)
        m_ShadowControllers[i]->OnPreSimulate(deltaTime);
    for (int i = 0; i < m_PlayerControllers.Count(); i++)
        m_PlayerControllers[i]->OnPreSimulate(deltaTime);
    for (int i = 0; i < m_MotionControllers.Count(); i++)
        m_MotionControllers[i]->OnPreSimulate(deltaTime);
    for (int i = 0; i < m_FluidControllers.Count(); i++)
        m_FluidControllers[i]->OnPreSimulate(deltaTime);

    // The solver overwrites velocities; the pre-step values are what impact damage measures against.
    for (int i = 0; i < m_Objects.Count(); i++)
        m_Objects[i]->SnapshotPreStepVelocity();

    m_flSimulationClock += deltaTime;

    // IVP-style quadratic air drag (CDragController) on every awake body.
    for (int i = 0; i < m_Objects.Count(); i++)
    {
        Box3DPhysicsObject* pObject = m_Objects[i];
        if (pObject->IsDragEnabled() && !pObject->IsStatic() && !pObject->IsAsleep())
            pObject->ApplyAirDrag(m_flAirDensity, deltaTime);
    }

    b3World_SetContactTuning(
        m_WorldId, vbox_contact_hertz.GetFloat(), vbox_contact_damping.GetFloat(),
        SourceToBox::Distance(vbox_contact_speed.GetFloat()));
    b3World_Step(m_WorldId, deltaTime, vbox_substeps.GetInt());

    // Wake/sleep transitions -> ObjectWake/ObjectSleep (prop sleep networking and game logic).
    for (int i = 0; i < m_Objects.Count(); i++)
    {
        Box3DPhysicsObject* pObject = m_Objects[i];
        const bool bAwake = !pObject->IsAsleep();
        if (bAwake == pObject->WasAwakeLastStep())
            continue;

        pObject->SetAwakeLastStep(bAwake);
        if (m_pObjectEvent)
        {
            if (bAwake)
                m_pObjectEvent->ObjectWake(pObject);
            else
                m_pObjectEvent->ObjectSleep(pObject);
        }
    }

    // Collect the objects that moved so the game can read back their transforms.
    const b3BodyEvents events = b3World_GetBodyEvents(m_WorldId);
    m_ActiveObjects.RemoveAll();

    // IVP clamps angular speed to PI/2 rad/tick (apply_velocity_limit); Box3D has no cap, so explosion
    // gibs spin unbounded to an invalid transform and the engine deletes them. Clamp moved bodies, and
    // share the limit so the object read-back (GetVelocity) clamps to the same value.
    m_flMaxAngularVelocity = (3.14159265f * 0.5f) / deltaTime;
    for (int i = 0; i < events.moveCount; i++)
    {
        Box3DPhysicsObject* pObject = static_cast<Box3DPhysicsObject*>(events.moveEvents[i].userData);
        if (!pObject)
            continue;
        m_ActiveObjects.AddToTail(pObject);

        const b3BodyId body = pObject->GetBodyID();
        const b3Vec3 w = b3Body_GetAngularVelocity(body);
        const float flLen = sqrtf(b3Dot(w, w));
        if (flLen > m_flMaxAngularVelocity)
            b3Body_SetAngularVelocity(body, b3MulSV(m_flMaxAngularVelocity / flLen, w));
    }

    DrainContactEvents();

    m_bInSimulation = false;

    // The game's PostSimulationFrame processes deferred touch/damage and expects IsInSimulation() false.
    if (m_pCollisionEvent)
        m_pCollisionEvent->PostSimulationFrame();

    if (!m_bDeleteQueueEnabled)
        CleanupDeleteList();
}

namespace
{
    // A single contact's collision data, handed to the game during a Pre/PostCollision or touch callback.
    class Box3DCollisionData final : public IPhysicsCollisionData
    {
    public:
        Box3DCollisionData(const Vector& vecNormal, const Vector& vecPoint)
            : m_vecNormal(vecNormal)
            , m_vecPoint(vecPoint)
        {
        }
        void GetSurfaceNormal(Vector& out) override
        {
            out = m_vecNormal;
        }
        void GetContactPoint(Vector& out) override
        {
            out = m_vecPoint;
        }
        void GetContactSpeed(Vector& out) override
        {
            out = vec3_origin;
        }

    private:
        Vector m_vecNormal;
        Vector m_vecPoint;
    };

    Box3DPhysicsObject* ObjectFromShape(b3ShapeId shapeId)
    {
        if (!b3Shape_IsValid(shapeId))
            return nullptr;
        const b3BodyId bodyId = b3Shape_GetBody(shapeId);
        if (!b3Body_IsValid(bodyId))
            return nullptr;
        return static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(bodyId));
    }

    // Should a Pre/PostCollision (sound/damage) callback fire for this pair? Mirrors the game's rules.
    bool IsCollisionCallback(Box3DPhysicsObject* p1, Box3DPhysicsObject* p2)
    {
        bool bIsCollision = (p1->GetCallbackFlags() & p2->GetCallbackFlags()) & CALLBACK_GLOBAL_COLLISION;
        if (p1->IsStatic() && !(p2->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC))
            bIsCollision = false;
        if (p2->IsStatic() && !(p1->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC))
            bIsCollision = false;
        return bIsCollision;
    }

    // Should a StartTouch/EndTouch callback fire for this pair?
    bool ShouldTouchCallback(Box3DPhysicsObject* p1, Box3DPhysicsObject* p2)
    {
        const uint32 uFlags = (uint32)p1->GetCallbackFlags() | (uint32)p2->GetCallbackFlags();
        if (!(uFlags & CALLBACK_GLOBAL_TOUCH))
            return false;
        if (!(uFlags & CALLBACK_GLOBAL_TOUCH_STATIC) && (p1->IsStatic() || p2->IsStatic()))
            return false;
        return true;
    }
} // namespace

void Box3DPhysicsEnvironment::DrainContactEvents()
{
    if (!m_pCollisionEvent)
        return;

    const b3ContactEvents events = b3World_GetContactEvents(m_WorldId);

    // Begin-touch -> StartTouch
    for (int i = 0; i < events.beginCount; i++)
    {
        Box3DPhysicsObject* p1 = ObjectFromShape(events.beginEvents[i].shapeIdA);
        Box3DPhysicsObject* p2 = ObjectFromShape(events.beginEvents[i].shapeIdB);
        if (!p1 || !p2 || !ShouldTouchCallback(p1, p2))
            continue;

        Box3DCollisionData data(vec3_origin, vec3_origin);
        m_pCollisionEvent->StartTouch(p1, p2, &data);
    }

    // Hit events -> Pre/PostCollision (drives impact sounds and damage). Pre/Post must be a matched pair.
    for (int i = 0; i < events.hitCount; i++)
    {
        const b3ContactHitEvent& hit = events.hitEvents[i];
        Box3DPhysicsObject* p1 = ObjectFromShape(hit.shapeIdA);
        Box3DPhysicsObject* p2 = ObjectFromShape(hit.shapeIdB);
        if (!p1 || !p2)
            continue;

        // Shadow collisions fire when exactly one side is a shadow (player damage from props goes
        // through this); if both are shadow the game handles it in AI, if neither there's no callback.
        const bool bIsCollision = IsCollisionCallback(p1, p2);
        const bool bIsShadowCollision = ((p1->GetCallbackFlags() ^ p2->GetCallbackFlags()) & CALLBACK_SHADOW_COLLISION) != 0;
        if (!bIsCollision && !bIsShadowCollision)
            continue;

        // Per-pair rate limit (IVP's deltaCollisionTime). Box3D fires a hit event every tick of sustained
        // fast contact; cap the same pair to one event per interval and report the real elapsed time.
        const float flDelta1 = (p1->m_nLastCollisionPartnerId == p2->m_nUniqueId)
            ? m_flSimulationClock - p1->m_flLastCollisionTime
            : 1000.0f;
        const float flDelta2 = (p2->m_nLastCollisionPartnerId == p1->m_nUniqueId)
            ? m_flSimulationClock - p2->m_flLastCollisionTime
            : 1000.0f;
        const float flDeltaCollisionTime = Min(flDelta1, flDelta2);
        if (flDeltaCollisionTime < kCollisionEventInterval)
            continue;

        p1->m_flLastCollisionTime = m_flSimulationClock;
        p1->m_nLastCollisionPartnerId = p2->m_nUniqueId;
        p2->m_flLastCollisionTime = m_flSimulationClock;
        p2->m_nLastCollisionPartnerId = p1->m_nUniqueId;

        // Box3D's normal points from A to B; the game wants it pointing toward object[1], so negate.
        Box3DCollisionData data(-BoxToSource::Unitless(hit.normal), BoxToSource::Distance(hit.point));

        vcollisionevent_t event = {};
        event.pObjects[0] = p1;
        event.pObjects[1] = p2;
        event.surfaceProps[0] = p1->GetMaterialIndex();
        event.surfaceProps[1] = p2->GetMaterialIndex();
        event.isCollision = bIsCollision;
        event.isShadowCollision = bIsShadowCollision;
        event.deltaCollisionTime = flDeltaCollisionTime;
        event.collisionSpeed = BoxToSource::Distance(hit.approachSpeed);
        event.pInternalData = &data;

        // The game samples velocities in PreCollision and again in PostCollision; the delta drives
        // impact damage. Hand it the pre-step velocities during PreCollision so the delta is real.
        const Vector vecOld1 = p1->FakeVelocity(p1->GetPreStepVelocity());
        const Vector vecOld2 = p2->FakeVelocity(p2->GetPreStepVelocity());
        m_pCollisionEvent->PreCollision(&event);
        p1->RestoreVelocity(vecOld1);
        p2->RestoreVelocity(vecOld2);

        m_pCollisionEvent->PostCollision(&event);
    }

    // End-touch -> EndTouch. Shapes/bodies here may already be destroyed; ObjectFromShape guards that.
    for (int i = 0; i < events.endCount; i++)
    {
        Box3DPhysicsObject* p1 = ObjectFromShape(events.endEvents[i].shapeIdA);
        Box3DPhysicsObject* p2 = ObjectFromShape(events.endEvents[i].shapeIdB);
        if (!p1 || !p2 || !ShouldTouchCallback(p1, p2))
            continue;

        Box3DCollisionData data(vec3_origin, vec3_origin);
        m_pCollisionEvent->EndTouch(p1, p2, &data);
    }
}

bool Box3DPhysicsEnvironment::IsInSimulation() const
{
    return m_bInSimulation;
}

float Box3DPhysicsEnvironment::GetSimulationTimestep() const
{
    return m_flSimulationTimestep;
}

void Box3DPhysicsEnvironment::SetSimulationTimestep(float timestep)
{
    m_flSimulationTimestep = timestep;
}

float Box3DPhysicsEnvironment::GetSimulationTime() const
{
    return m_flSimulationClock;
}

void Box3DPhysicsEnvironment::ResetSimulationClock()
{
    m_flSimulationClock = 0.0f;
}

float Box3DPhysicsEnvironment::GetNextFrameTime() const
{
    Log_Stub(LOG_VBox3D);
    return 0.0f;
}

void Box3DPhysicsEnvironment::SetCollisionEventHandler(IPhysicsCollisionEvent* pCollisionEvents)
{
    m_pCollisionEvent = pCollisionEvents;
}

void Box3DPhysicsEnvironment::SetObjectEventHandler(IPhysicsObjectEvent* pObjectEvents)
{
    m_pObjectEvent = pObjectEvents;
}

void Box3DPhysicsEnvironment::SetConstraintEventHandler(IPhysicsConstraintEvent* pConstraintEvents)
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::SetQuickDelete(bool bQuick)
{
    Log_Stub(LOG_VBox3D);
}

int Box3DPhysicsEnvironment::GetActiveObjectCount() const
{
    return m_ActiveObjects.Count();
}

void Box3DPhysicsEnvironment::GetActiveObjects(IPhysicsObject** pOutputObjectList) const
{
    for (int i = 0; i < m_ActiveObjects.Count(); i++)
        pOutputObjectList[i] = m_ActiveObjects[i];
}

const IPhysicsObject** Box3DPhysicsEnvironment::GetObjectList(int* pOutputObjectCount) const
{
    if (pOutputObjectCount)
        *pOutputObjectCount = m_Objects.Count();
    return (const IPhysicsObject**)m_Objects.Base();
}

bool Box3DPhysicsEnvironment::TransferObject(IPhysicsObject* pObject, IPhysicsEnvironment* pDestinationEnvironment)
{
    Log_Stub(LOG_VBox3D);
    return false;
}

void Box3DPhysicsEnvironment::CleanupDeleteList()
{
    for (int i = 0; i < m_DeadObjects.Count(); i++)
        DeleteObject(m_DeadObjects[i]);
    m_DeadObjects.RemoveAll();

    // Free collides that were deferred while their dead objects were still queued.
    for (int i = 0; i < m_DeadObjectCollides.Count(); i++)
        Box3DPhysicsCollision::GetInstance().DestroyCollide(m_DeadObjectCollides[i]);
    m_DeadObjectCollides.RemoveAll();
}

void Box3DPhysicsEnvironment::EnableDeleteQueue(bool enable)
{
    m_bDeleteQueueEnabled = enable;
}

bool Box3DPhysicsEnvironment::Save(const physsaveparams_t& params)
{
    Log_Stub(LOG_VBox3D);
    return false;
}

void Box3DPhysicsEnvironment::PreRestore(const physprerestoreparams_t& params)
{
    Log_Stub(LOG_VBox3D);
}

bool Box3DPhysicsEnvironment::Restore(const physrestoreparams_t& params)
{
    Log_Stub(LOG_VBox3D);
    return false;
}

void Box3DPhysicsEnvironment::PostRestore()
{
    Log_Stub(LOG_VBox3D);
}

bool Box3DPhysicsEnvironment::IsCollisionModelUsed(CPhysCollide* pCollide) const
{
    Log_Stub(LOG_VBox3D);
    return false;
}

void Box3DPhysicsEnvironment::TraceRay(const Ray_t& ray, unsigned int fMask, IPhysicsTraceFilter* pTraceFilter, trace_t* pTrace)
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::SweepCollideable(
    const CPhysCollide* pCollide, const Vector& vecAbsStart, const Vector& vecAbsEnd, const QAngle& vecAngles,
    unsigned int fMask, IPhysicsTraceFilter* pTraceFilter, trace_t* pTrace)
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::GetPerformanceSettings(physics_performanceparams_t* pOutput) const
{
    if (pOutput)
        *pOutput = m_PerformanceParams;
}

void Box3DPhysicsEnvironment::SetPerformanceSettings(const physics_performanceparams_t* pSettings)
{
    if (pSettings)
        m_PerformanceParams = *pSettings;
}

void Box3DPhysicsEnvironment::ReadStats(physics_stats_t* pOutput)
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::ClearStats()
{
    Log_Stub(LOG_VBox3D);
}

unsigned int Box3DPhysicsEnvironment::GetObjectSerializeSize(IPhysicsObject* pObject) const
{
    Log_Stub(LOG_VBox3D);
    return 0;
}

void Box3DPhysicsEnvironment::SerializeObjectToBuffer(IPhysicsObject* pObject, unsigned char* pBuffer, unsigned int bufferSize)
{
    Log_Stub(LOG_VBox3D);
}

IPhysicsObject* Box3DPhysicsEnvironment::UnserializeObjectFromBuffer(
    void* pGameData, unsigned char* pBuffer, unsigned int bufferSize, bool enableCollisions)
{
    Log_Stub(LOG_VBox3D);
    return nullptr;
}

void Box3DPhysicsEnvironment::EnableConstraintNotify(bool bEnable)
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::DebugCheckContacts()
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::SetAlternateGravity(const Vector& gravityVector)
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::GetAlternateGravity(Vector* pGravityVector) const
{
    Log_Stub(LOG_VBox3D);
}

float Box3DPhysicsEnvironment::GetDeltaFrameTime(int maxTicks) const
{
    Log_Stub(LOG_VBox3D);
    return 0.0f;
}

void Box3DPhysicsEnvironment::ForceObjectsToSleep(IPhysicsObject** pList, int listCount)
{
    for (int i = 0; i < listCount; i++)
        if (pList[i])
            pList[i]->Sleep();
}

void Box3DPhysicsEnvironment::SetPredicted(bool bPredicted)
{
    Log_Stub(LOG_VBox3D);
}

bool Box3DPhysicsEnvironment::IsPredicted()
{
    Log_Stub(LOG_VBox3D);
    return false;
}

void Box3DPhysicsEnvironment::SetPredictionCommandNum(int iCommandNum)
{
    Log_Stub(LOG_VBox3D);
}

int Box3DPhysicsEnvironment::GetPredictionCommandNum()
{
    Log_Stub(LOG_VBox3D);
    return 0;
}

void Box3DPhysicsEnvironment::DoneReferencingPreviousCommands(int iCommandNum)
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::RestorePredictedSimulation()
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::DestroyCollideOnDeadObjectFlush(CPhysCollide* pCollide)
{
    // Defer the collide destroy if a queued-dead object still uses it; CleanupDeleteList frees it after.
    for (int i = 0; i < m_DeadObjects.Count(); i++)
    {
        if (m_DeadObjects[i]->GetCollide() == pCollide)
        {
            if (m_DeadObjectCollides.Find(pCollide) == m_DeadObjectCollides.InvalidIndex())
                m_DeadObjectCollides.AddToTail(pCollide);
            return;
        }
    }
    Box3DPhysicsCollision::GetInstance().DestroyCollide(pCollide);
}

#if defined(GAME_GMOD_64X)
void Box3DPhysicsEnvironment::PreSave(const physprerestoreparams_t& params)
{
    Log_Stub(LOG_VBox3D);
}

void Box3DPhysicsEnvironment::PostSave()
{
    Log_Stub(LOG_VBox3D);
}
#endif
