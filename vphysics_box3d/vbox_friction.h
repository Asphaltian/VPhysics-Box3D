#pragma once

#include "vphysics/friction.h"

class Box3DPhysicsObject;

// The game iterates `while ( snapshot->IsValid() )` without null-checking, so we return a valid
// empty snapshot rather than null.
class Box3DFrictionSnapshot final : public IPhysicsFrictionSnapshot
{
public:
    Box3DFrictionSnapshot(Box3DPhysicsObject* pSelf, float flStepTime);

    bool IsValid() override;
    IPhysicsObject* GetObject(int index) override;
    int GetMaterial(int index) override;
    void GetContactPoint(Vector& out) override;
    void GetSurfaceNormal(Vector& out) override;
    float GetNormalForce() override;
    float GetEnergyAbsorbed() override;
    void RecomputeFriction() override;
    void ClearFrictionForce() override;
    void MarkContactForDelete() override;
    void DeleteAllMarkedContacts(bool wakeObjects) override;
    void NextFrictionData() override;
    float GetFrictionCoefficient() override;

private:
    struct Entry_t
    {
        Box3DPhysicsObject* pSelf;
        Box3DPhysicsObject* pOther;
        Vector vNormal;
        Vector vPoint;
        float flNormalForce;
        float flEnergy;
    };
    CUtlVector<Entry_t> m_Entries;
    int m_nIndex = 0;
};
