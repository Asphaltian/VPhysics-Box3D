
#pragma once

#include "vbox_interface.h"

class Box3DPhysicsObject;

class Box3DPhysicsPlayerController final : public IPhysicsPlayerController
{
public:
    explicit Box3DPhysicsPlayerController(Box3DPhysicsObject* pObject);
    ~Box3DPhysicsPlayerController() override;

    void Update(
        const Vector& position, const Vector& velocity, float secondsToArrival, bool onground, IPhysicsObject* ground) override;
    void SetEventHandler(IPhysicsPlayerControllerEvent* handler) override;
    bool IsInContact() override;
    void MaxSpeed(const Vector& maxVelocity) override;
    void SetObject(IPhysicsObject* pObject) override;
    int GetShadowPosition(Vector* position, QAngle* angles) override;
    void StepUp(float height) override;
    void Jump() override;
    void GetShadowVelocity(Vector* velocity) override;
    IPhysicsObject* GetObject() override;
    void GetLastImpulse(Vector* pOut) override;
    void SetPushMassLimit(float maxPushMass) override;
    void SetPushSpeedLimit(float maxPushSpeed) override;
    float GetPushMassLimit() override;
    float GetPushSpeedLimit() override;
    bool WasFrozen() override;

    // Ticked by the environment before each simulation step.
    void OnPreSimulate(float flDeltaTime);
    void ClearGround(Box3DPhysicsObject* pObject);
    Box3DPhysicsObject* GetControlledObject() const
    {
        return m_pObject;
    }

private:
    void SetObjectInternal(Box3DPhysicsObject* pObject);
    void SetGround(Box3DPhysicsObject* pGround);
    int TryTeleportObject();

    Box3DPhysicsObject* m_pObject = nullptr;
    Box3DPhysicsObject* m_pGround = nullptr;
    IPhysicsPlayerControllerEvent* m_pHandler = nullptr;

    Vector m_targetPosition = vec3_origin;
    Vector m_groundPosition = vec3_origin;
    Vector m_maxSpeed = vec3_origin;
    Vector m_currentSpeed = vec3_origin;
    Vector m_lastImpulse = vec3_origin;
    float m_secondsToArrival = 0.0f;
    float m_maxDeltaPosition = 24.0f;
    float m_dampFactor = 1.0f;
    float m_pushableMassLimit = VPHYSICS_MAX_MASS;
    float m_pushableSpeedLimit = 1e4f;
    float m_flSavedAngularDamping = 0.0f;
    bool m_enable = false;
    bool m_updatedSinceLast = false;
};
