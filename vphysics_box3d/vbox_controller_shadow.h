
#pragma once

#include "vbox_interface.h"

class Box3DPhysicsObject;

class Box3DPhysicsShadowController final : public IPhysicsShadowController
{
public:
    Box3DPhysicsShadowController(Box3DPhysicsObject* pObject, bool allowTranslation, bool allowRotation);
    ~Box3DPhysicsShadowController() override;

    void Update(const Vector& position, const QAngle& angles, float timeOffset) override;
    void MaxSpeed(float maxSpeed, float maxAngularSpeed) override;
    void StepUp(float height) override;
    void SetTeleportDistance(float teleportDistance) override;
    bool AllowsTranslation() override;
    bool AllowsRotation() override;
    void SetPhysicallyControlled(bool isPhysicallyControlled) override;
    bool IsPhysicallyControlled() override;
    void GetLastImpulse(Vector* pOut) override;
    void UseShadowMaterial(bool bUseShadowMaterial) override;
    void ObjectMaterialChanged(int materialIndex) override;
    float GetTargetPosition(Vector* pPositionOut, QAngle* pAnglesOut) override;
    float GetTeleportDistance() override;
    void GetMaxSpeed(float* pMaxSpeedOut, float* pMaxAngularSpeedOut) override;

    // Ticked by the environment before each simulation step.
    void OnPreSimulate(float flDeltaTime);
    Box3DPhysicsObject* GetObject() const
    {
        return m_pObject;
    }

private:
    Box3DPhysicsObject* m_pObject;

    Vector m_targetPosition = vec3_origin;
    QAngle m_targetAngles = vec3_angle;
    Vector m_lastImpulse = vec3_origin;
    float m_secondsToArrival = 0.0f;
    float m_maxSpeed = 0.0f;
    float m_maxDampSpeed = 0.0f;
    float m_maxAngular = 0.0f;
    float m_maxDampAngular = 0.0f;
    float m_teleportDistance = 0.0f;

    b3BodyType m_savedBodyType = b3_staticBody;
    int m_savedMaterialIndex = 0;
    unsigned short m_savedCallbackFlags = 0;
    bool m_allowTranslation = true;
    bool m_allowRotation = true;
    bool m_isPhysicallyControlled = false;
    bool m_enabled = false;
};
