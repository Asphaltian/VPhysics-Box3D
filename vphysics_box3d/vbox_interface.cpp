//=================================================================================================
//
// The base physics DLL interface
//
//=================================================================================================

#include "vbox_interface.h"

#include "cbase.h"
#include "vbox_collide.h"
#include "vbox_environment.h"
#include "vbox_objectpairhash.h"
#include "vbox_surfaceprops.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

DEFINE_LOGGING_CHANNEL_NO_TAGS(LOG_VBox3D, "VBox3D", 0, LS_MESSAGE, Color(142, 205, 212, 255));

Box3DPhysicsInterface Box3DPhysicsInterface::s_PhysicsInterface;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(
    Box3DPhysicsInterface, IPhysics, VPHYSICS_INTERFACE_VERSION, Box3DPhysicsInterface::GetInstance());

//-------------------------------------------------------------------------------------------------

// Route all of Box3D's allocations through Valve's allocator, matching new/delete.
static void* Box3D_Allocate(int32 size, int32 alignment)
{
    return MemAlloc_AllocAligned(size, alignment);
}

static void Box3D_Free(void* block)
{
    MemAlloc_FreeAligned(block);
}

static int Box3D_OnAssert(const char* condition, const char* fileName, int lineNumber)
{
    Log_Warning(LOG_VBox3D, "Assert: %s (%s:%d)\n", condition, fileName, lineNumber);
    return 0;
}

static void Box3D_OnLog(const char* message)
{
    Log_Msg(LOG_VBox3D, "%s", message);
}

//-------------------------------------------------------------------------------------------------

InitReturnVal_t Box3DPhysicsInterface::Init()
{
    const InitReturnVal_t nRetVal = BaseClass::Init();
    if (nRetVal != INIT_OK)
        return nRetVal;

    MathLib_Init();

    b3SetAllocator(Box3D_Allocate, Box3D_Free);
    b3SetAssertFcn(Box3D_OnAssert);
    b3SetLogFcn(Box3D_OnLog);

    return INIT_OK;
}

void Box3DPhysicsInterface::Shutdown()
{
    BaseClass::Shutdown();
}

void* Box3DPhysicsInterface::QueryInterface(const char* pInterfaceName)
{
    CreateInterfaceFn factory = Sys_GetFactoryThis();
    return factory(pInterfaceName, NULL);
}

//-------------------------------------------------------------------------------------------------

IPhysicsEnvironment* Box3DPhysicsInterface::CreateEnvironment()
{
    IPhysicsEnvironment* pEnvironment = new Box3DPhysicsEnvironment();
    m_Environments.push_back(pEnvironment);
    return pEnvironment;
}

void Box3DPhysicsInterface::DestroyEnvironment(IPhysicsEnvironment* pEnvironment)
{
    Erase(m_Environments, pEnvironment);
    delete static_cast<Box3DPhysicsEnvironment*>(pEnvironment);
}

IPhysicsEnvironment* Box3DPhysicsInterface::GetActiveEnvironmentByIndex(int index)
{
    if (index < 0 || index >= (int)m_Environments.size())
        return nullptr;

    return m_Environments[index];
}

//-------------------------------------------------------------------------------------------------

IPhysicsObjectPairHash* Box3DPhysicsInterface::CreateObjectPairHash()
{
    return new Box3DPhysicsObjectPairHash;
}

void Box3DPhysicsInterface::DestroyObjectPairHash(IPhysicsObjectPairHash* pHash)
{
    delete static_cast<Box3DPhysicsObjectPairHash*>(pHash);
}

//-------------------------------------------------------------------------------------------------

IPhysicsCollisionSet* Box3DPhysicsInterface::FindOrCreateCollisionSet(unsigned int id, int maxElementCount)
{
    if (maxElementCount > 32)
        return nullptr;

    if (IPhysicsCollisionSet* pSet = FindCollisionSet(id))
        return pSet;

    auto result = m_CollisionSets.emplace(id, Box3DPhysicsCollisionSet{});
    return &result.first->second;
}

IPhysicsCollisionSet* Box3DPhysicsInterface::FindCollisionSet(unsigned int id)
{
    auto iter = m_CollisionSets.find(id);
    if (iter != m_CollisionSets.end())
        return &iter->second;

    return nullptr;
}

void Box3DPhysicsInterface::DestroyAllCollisionSets()
{
    m_CollisionSets.clear();
}
