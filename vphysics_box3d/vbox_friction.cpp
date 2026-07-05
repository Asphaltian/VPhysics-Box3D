#include "vbox_friction.h"

#include "cbase.h"
#include "vbox_object.h"
#include "vbox_surfaceprops.h"

#include "tier0/memdbgon.h"

Box3DFrictionSnapshot::Box3DFrictionSnapshot(Box3DPhysicsObject* pSelf, float flStepTime)
{
    const float flInvStep = flStepTime > 0.0f ? 1.0f / flStepTime : 0.0f;
    const b3BodyId body = pSelf->GetBodyID();

    const int nCapacity = b3Body_GetContactCapacity(body);
    if (nCapacity <= 0)
        return;

    CUtlVector<b3ContactData> contacts;
    contacts.SetCount(nCapacity);
    const int nCount = b3Body_GetContactData(body, contacts.Base(), nCapacity);

    for (int i = 0; i < nCount; i++)
    {
        const b3ContactData& contact = contacts[i];
        const b3BodyId bodyA = b3Shape_GetBody(contact.shapeIdA);
        Box3DPhysicsObject* pA = static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(bodyA));
        const bool bSelfIsA = (pA == pSelf);
        Box3DPhysicsObject* pOther = bSelfIsA
            ? static_cast<Box3DPhysicsObject*>(b3Body_GetUserData(b3Shape_GetBody(contact.shapeIdB)))
            : pA;
        if (!pOther)
            continue;

        const b3Pos comA = b3Body_GetWorldCenter(bodyA);

        for (int m = 0; m < contact.manifoldCount; m++)
        {
            const b3Manifold& manifold = contact.manifolds[m];

            Vector vNormal = BoxToSource::Unitless(manifold.normal);
            if (!bSelfIsA)
                vNormal = -vNormal;

            for (int p = 0; p < manifold.pointCount; p++)
            {
                const b3ManifoldPoint& point = manifold.points[p];
                if (point.totalNormalImpulse <= 0.0f)
                    continue;

                Entry_t entry;
                entry.pSelf = pSelf;
                entry.pOther = pOther;
                entry.vNormal = vNormal;
                entry.vPoint = BoxToSource::Distance(
                    b3Vec3{ comA.x + point.anchorA.x, comA.y + point.anchorA.y, comA.z + point.anchorA.z });
                entry.flNormalForce = BoxToSource::Distance(point.totalNormalImpulse * flInvStep);
                entry.flEnergy = BoxToSource::Distance(
                    BoxToSource::Distance(fabsf(point.totalNormalImpulse * point.normalVelocity)));
                m_Entries.AddToTail(entry);
            }
        }
    }
}

bool Box3DFrictionSnapshot::IsValid()
{
    return m_nIndex < m_Entries.Count();
}

IPhysicsObject* Box3DFrictionSnapshot::GetObject(int index)
{
    if (!IsValid())
        return nullptr;
    return index == 1 ? m_Entries[m_nIndex].pOther : m_Entries[m_nIndex].pSelf;
}

int Box3DFrictionSnapshot::GetMaterial(int index)
{
    IPhysicsObject* pObj = GetObject(index);
    return pObj ? static_cast<Box3DPhysicsObject*>(pObj)->GetMaterialIndex() : 0;
}

void Box3DFrictionSnapshot::GetContactPoint(Vector& out)
{
    out = IsValid() ? m_Entries[m_nIndex].vPoint : vec3_origin;
}

void Box3DFrictionSnapshot::GetSurfaceNormal(Vector& out)
{
    out = IsValid() ? m_Entries[m_nIndex].vNormal : vec3_origin;
}

float Box3DFrictionSnapshot::GetNormalForce()
{
    return IsValid() ? m_Entries[m_nIndex].flNormalForce : 0.0f;
}

float Box3DFrictionSnapshot::GetEnergyAbsorbed()
{
    return IsValid() ? m_Entries[m_nIndex].flEnergy : 0.0f;
}

void Box3DFrictionSnapshot::RecomputeFriction()
{
}

void Box3DFrictionSnapshot::ClearFrictionForce()
{
}

void Box3DFrictionSnapshot::MarkContactForDelete()
{
}

void Box3DFrictionSnapshot::DeleteAllMarkedContacts(bool)
{
}

void Box3DFrictionSnapshot::NextFrictionData()
{
    m_nIndex++;
}

float Box3DFrictionSnapshot::GetFrictionCoefficient()
{
    if (!IsValid())
        return 0.0f;
    const surfacedata_t* pSelfSurf = Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData(
        m_Entries[m_nIndex].pSelf->GetMaterialIndex());
    const surfacedata_t* pOtherSurf = Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceData(
        m_Entries[m_nIndex].pOther->GetMaterialIndex());
    const float flSelf = pSelfSurf ? pSelfSurf->physics.friction : 0.0f;
    const float flOther = pOtherSurf ? pOtherSurf->physics.friction : flSelf;
    return flSelf * flOther;
}
