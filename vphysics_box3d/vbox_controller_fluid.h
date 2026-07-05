
#pragma once

#include "vbox_interface.h"

class Box3DPhysicsObject;

// Water buoyancy, drag and current, applied per step to bodies overlapping the fluid volume.
class Box3DPhysicsFluidController final : public IPhysicsFluidController
{
public:
    Box3DPhysicsFluidController(Box3DPhysicsObject* pFluidObject, const fluidparams_t* pParams);
    ~Box3DPhysicsFluidController() override;

    void SetGameData(void* pGameData) override;
    void* GetGameData() const override;
    void GetSurfacePlane(Vector* pNormal, float* pDist) const override;
    float GetDensity() const override;
    void WakeAllSleepingObjects() override;
    int GetContents() const override;

    // Ticked by the environment before each simulation step.
    void OnPreSimulate(float flDeltaTime);
    // A tracked object was destroyed.
    void DetachObject(Box3DPhysicsObject* pObject);
    Box3DPhysicsObject* GetFluidObject() const
    {
        return m_pFluidObject;
    }

private:
    cplane_t GetWorldSurfacePlane() const;

    Box3DPhysicsObject* m_pFluidObject;
    fluidparams_t m_Params;
    cplane_t m_LocalPlane; // surface plane in fluid-object local space
    CUtlVector<Box3DPhysicsObject*> m_ObjectsInFluid;
};
