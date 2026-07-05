
#include "vbox_controller_motion.h"

#include "cbase.h"
#include "vbox_object.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------
// Motion controller: each step the game's IMotionEvent computes a velocity/force for every attached
// object and we apply it. This is the gravity gun's grab.
//-------------------------------------------------------------------------------------------------

Box3DPhysicsMotionController::Box3DPhysicsMotionController(IMotionEvent* pHandler)
    : m_pHandler(pHandler)
{
}

void Box3DPhysicsMotionController::SetEventHandler(IMotionEvent* pHandler)
{
    m_pHandler = pHandler;
}

void Box3DPhysicsMotionController::AttachObject(IPhysicsObject* pObject, bool checkIfAlreadyAttached)
{
    if (!pObject || pObject->IsStatic())
        return;

    Box3DPhysicsObject* pPhysicsObject = static_cast<Box3DPhysicsObject*>(pObject);
    if (checkIfAlreadyAttached && m_Objects.Find(pPhysicsObject) != m_Objects.InvalidIndex())
        return;

    m_Objects.AddToTail(pPhysicsObject);
}

void Box3DPhysicsMotionController::DetachObject(IPhysicsObject* pObject)
{
    m_Objects.FindAndRemove(static_cast<Box3DPhysicsObject*>(pObject));
}

int Box3DPhysicsMotionController::CountObjects()
{
    return m_Objects.Count();
}

void Box3DPhysicsMotionController::GetObjects(IPhysicsObject** pObjectList)
{
    for (int i = 0; i < m_Objects.Count(); i++)
        pObjectList[i] = m_Objects[i];
}

void Box3DPhysicsMotionController::ClearObjects()
{
    m_Objects.RemoveAll();
}

void Box3DPhysicsMotionController::WakeObjects()
{
    for (int i = 0; i < m_Objects.Count(); i++)
        m_Objects[i]->Wake();
}

void Box3DPhysicsMotionController::SetPriority(priority_t)
{
}

void Box3DPhysicsMotionController::OnPreSimulate(float flDeltaTime)
{
    if (!m_pHandler)
        return;

    for (int i = 0; i < m_Objects.Count(); i++)
    {
        Box3DPhysicsObject* pObject = m_Objects[i];
        if (!pObject->IsMoveable())
            continue;

        Vector vecLinear = vec3_origin;
        AngularImpulse angLocalAngular = vec3_origin;
        const IMotionEvent::simresult_e result = m_pHandler->Simulate(this, pObject, flDeltaTime, vecLinear, angLocalAngular);

        vecLinear *= flDeltaTime;
        angLocalAngular *= flDeltaTime;

        // The linear value is local or global depending on the result type; the angular value is
        // always in the object's local space. AddVelocity takes it as-is; ApplyTorqueCenter takes
        // a world-space torque impulse, so rotate it out for the force path.
        Vector vecWorldLinear = vecLinear;
        if (result == IMotionEvent::SIM_LOCAL_ACCELERATION || result == IMotionEvent::SIM_LOCAL_FORCE)
            pObject->LocalToWorldVector(&vecWorldLinear, vecLinear);

        Vector vecWorldAngular;
        pObject->LocalToWorldVector(&vecWorldAngular, angLocalAngular);

        switch (result)
        {
            case IMotionEvent::SIM_GLOBAL_ACCELERATION:
            case IMotionEvent::SIM_LOCAL_ACCELERATION:
                pObject->AddVelocity(&vecWorldLinear, &angLocalAngular);
                break;

            case IMotionEvent::SIM_GLOBAL_FORCE:
            case IMotionEvent::SIM_LOCAL_FORCE:
                pObject->ApplyForceCenter(vecWorldLinear);
                pObject->ApplyTorqueCenter(vecWorldAngular);
                break;

            case IMotionEvent::SIM_NOTHING:
            default:
                break;
        }
    }
}
