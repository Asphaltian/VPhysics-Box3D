//=================================================================================================
//
// The base physics DLL interface
//
//=================================================================================================

#pragma once

DECLARE_LOGGING_CHANNEL(LOG_VBox3D);

// Call this in stubbed functions to spew when they're hit
#if 1 // DEVELOPMENT_ONLY
#    define Log_Stub(Channel)
#else
#    define Log_Stub(Channel) Log_Warning(Channel, "Stub: %s\n", __FUNCTION__)
#endif

// So we can toggle assertions in this module at our discretion
#if DEVELOPMENT_ONLY
#    define VBoxAssert DevAssert
#    define VBoxAssertMsg DevAssertMsg
#else
#    define VBoxAssert Assert
#    define VBoxAssertMsg AssertMsg
#endif

//-------------------------------------------------------------------------------------------------

class Box3DPhysicsCollisionSet final : public IPhysicsCollisionSet
{
public:
    void EnableCollisions(int index0, int index1) override
    {
        m_Bits[index0] |= 1u << index1;
        m_Bits[index1] |= 1u << index0;
    }

    void DisableCollisions(int index0, int index1) override
    {
        m_Bits[index0] &= ~(1u << index1);
        m_Bits[index1] &= ~(1u << index0);
    }

    bool ShouldCollide(int index0, int index1) override
    {
        return !!(m_Bits[index0] & (1u << index1));
    }

private:
    std::array<uint32, 32> m_Bits = {};
};

//-------------------------------------------------------------------------------------------------

class Box3DPhysicsInterface final : public CTier1AppSystem<IPhysics>
{
private:
    using BaseClass = CTier1AppSystem<IPhysics>;

public:
    InitReturnVal_t Init() override;
    void Shutdown() override;
    void* QueryInterface(const char* pInterfaceName) override;

    IPhysicsEnvironment* CreateEnvironment() override;
    void DestroyEnvironment(IPhysicsEnvironment* pEnvironment) override;
    IPhysicsEnvironment* GetActiveEnvironmentByIndex(int index) override;

    IPhysicsObjectPairHash* CreateObjectPairHash() override;
    void DestroyObjectPairHash(IPhysicsObjectPairHash* pHash) override;

    IPhysicsCollisionSet* FindOrCreateCollisionSet(unsigned int id, int maxElementCount) override;
    IPhysicsCollisionSet* FindCollisionSet(unsigned int id) override;
    void DestroyAllCollisionSets() override;

#if GAME_GMOD
    bool IsValidPhysicsObject(IPhysicsObject* pObject) override
    {
        return pObject != nullptr;
    }
#endif

public:
    static Box3DPhysicsInterface& GetInstance()
    {
        return s_PhysicsInterface;
    }

private:
    std::unordered_map<unsigned int, Box3DPhysicsCollisionSet> m_CollisionSets;
    std::vector<IPhysicsEnvironment*> m_Environments;

    static Box3DPhysicsInterface s_PhysicsInterface;
};
