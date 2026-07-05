
#pragma once

#include "vbox_interface.h"

class Box3DPhysicsObject;

class Box3DPhysicsMotionController final : public IPhysicsMotionController
{
public:
    explicit Box3DPhysicsMotionController(IMotionEvent* pHandler);

    void SetEventHandler(IMotionEvent* pHandler) override;
    void AttachObject(IPhysicsObject* pObject, bool checkIfAlreadyAttached) override;
    void DetachObject(IPhysicsObject* pObject) override;
    int CountObjects() override;
    void GetObjects(IPhysicsObject** pObjectList) override;
    void ClearObjects() override;
    void WakeObjects() override;
    void SetPriority(priority_t priority) override;

    // Ticked by the environment before each simulation step.
    void OnPreSimulate(float flDeltaTime);

private:
    IMotionEvent* m_pHandler;
    CUtlVector<Box3DPhysicsObject*> m_Objects;
};
