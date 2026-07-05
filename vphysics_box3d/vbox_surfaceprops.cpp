
#include "vbox_surfaceprops.h"

#include "cbase.h"
#include "vbox_interface.h"
#include "vbox_keyvalues_schema.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

Box3DPhysicsSurfaceProps Box3DPhysicsSurfaceProps::s_PhysicsSurfaceProps;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(
    Box3DPhysicsSurfaceProps, IPhysicsSurfaceProps, VPHYSICS_SURFACEPROPS_INTERFACE_VERSION,
    Box3DPhysicsSurfaceProps::GetInstance());

//-------------------------------------------------------------------------------------------------

static const Box3DKVSchemaProp_t kSurfacePropDescs[] = {
    // Base Property
    { "Base", KVSCHEMA_DESC(Box3DSurfaceProp, data), FillBaseProp },

    // Physics Properties
    { "Friction", KVSCHEMA_DESC(Box3DSurfaceProp, data.physics.friction), FillFloatProp },
    { "Elasticity", KVSCHEMA_DESC(Box3DSurfaceProp, data.physics.elasticity), FillFloatProp },
    { "Density", KVSCHEMA_DESC(Box3DSurfaceProp, data.physics.density), FillFloatProp },
    { "Thickness", KVSCHEMA_DESC(Box3DSurfaceProp, data.physics.thickness), FillFloatProp },
    { "Dampening", KVSCHEMA_DESC(Box3DSurfaceProp, data.physics.dampening), FillFloatProp },

    // Audio Properties
    { "AudioReflectivity", KVSCHEMA_DESC(Box3DSurfaceProp, data.audio.reflectivity), FillFloatProp },
    { "AudioHardnessFactor", KVSCHEMA_DESC(Box3DSurfaceProp, data.audio.hardnessFactor), FillFloatProp },
    { "AudioRoughnessFactor", KVSCHEMA_DESC(Box3DSurfaceProp, data.audio.roughnessFactor), FillFloatProp },

    { "ScrapeRoughThreshold", KVSCHEMA_DESC(Box3DSurfaceProp, data.audio.roughThreshold), FillFloatProp },
    { "ImpactHardThreshold", KVSCHEMA_DESC(Box3DSurfaceProp, data.audio.hardThreshold), FillFloatProp },
    { "AudioHardMinVelocity", KVSCHEMA_DESC(Box3DSurfaceProp, data.audio.hardVelocityThreshold), FillFloatProp },

#ifdef GAME_CSGO_OR_NEWER
    { "HighPitchOcclusion", KVSCHEMA_DESC(Box3DSurfaceProp, data.audio.highPitchOcclusion), FillFloatProp },
    { "MidPitchOcclusion", KVSCHEMA_DESC(Box3DSurfaceProp, data.audio.midPitchOcclusion), FillFloatProp },
    { "LowPitchOcclusion", KVSCHEMA_DESC(Box3DSurfaceProp, data.audio.lowPitchOcclusion), FillFloatProp },
#endif

// Sound Properties
#ifdef GAME_ASW_OR_NEWER
    { "StepLeft", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.walkStepLeft), FillSoundProp },
    { "StepLeft", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.runStepLeft), FillSoundProp },
    { "StepRight", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.walkStepRight), FillSoundProp },
    { "StepRight", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.runStepRight), FillSoundProp },

    { "WalkLeft", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.walkStepLeft), FillSoundProp },
    { "WalkRight", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.walkStepRight), FillSoundProp },

    { "RunLeft", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.runStepLeft), FillSoundProp },
    { "RunRight", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.runStepRight), FillSoundProp },
#else
    { "StepLeft", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.stepleft), FillSoundProp },
    { "StepRight", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.stepright), FillSoundProp },
#endif

    { "ImpactSoft", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.impactSoft), FillSoundProp },
    { "ImpactHard", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.impactHard), FillSoundProp },

    { "ScrapeSmooth", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.scrapeSmooth), FillSoundProp },
    { "ScrapeRough", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.scrapeRough), FillSoundProp },

    { "BulletImpact", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.bulletImpact), FillSoundProp },
    { "Rolling", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.rolling), FillSoundProp },

    { "Break", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.breakSound), FillSoundProp },
    { "Strain", KVSCHEMA_DESC(Box3DSurfaceProp, data.sounds.strainSound), FillSoundProp },

    // Game Properties
    { "MaxSpeedFactor", KVSCHEMA_DESC(Box3DSurfaceProp, data.game.maxSpeedFactor), FillFloatProp },
    { "JumpFactor", KVSCHEMA_DESC(Box3DSurfaceProp, data.game.jumpFactor), FillFloatProp },
#ifdef GAME_CSGO_OR_NEWER
    { "PenetrationModifier", KVSCHEMA_DESC(Box3DSurfaceProp, data.game.penetrationModifier), FillFloatProp },
    { "DamageModifier", KVSCHEMA_DESC(Box3DSurfaceProp, data.game.damageModifier), FillFloatProp },
#endif
    { "GameMaterial", KVSCHEMA_DESC(Box3DSurfaceProp, data.game.material), FillGameMaterialProp },
    { "Climbable", KVSCHEMA_DESC(Box3DSurfaceProp, data.game.climbable), FillUnsignedCharProp },
#ifdef GAME_CSGO_OR_NEWER
    { "HideTargetID", KVSCHEMA_DESC(Box3DSurfaceProp, data.game.hidetargetid), FillBoolProp },
    { "DamageLossPercentPerPenetration", KVSCHEMA_DESC(Box3DSurfaceProp, data.game.damageLossPercentPerPenetration),
      FillFloatProp },
#endif
};

//-------------------------------------------------------------------------------------------------

Box3DPhysicsSurfaceProps::Box3DPhysicsSurfaceProps()
{
    Box3DSurfaceProp prop = {};
    prop.data.physics.friction = 0.8f;
    prop.data.physics.elasticity = 0.25f;
    prop.data.physics.density = 2000.0f;
    prop.data.physics.thickness = 0.0f;
    prop.data.physics.dampening = 0.0f;
    m_SurfaceProps["default"] = prop;

    // Game code uses 0 as invalid index, expects empty string
    m_SoundStrings.AddString("");
}

//-------------------------------------------------------------------------------------------------

int Box3DPhysicsSurfaceProps::ParseSurfaceData(const char* pFilename, const char* pTextfile)
{
    KeyValues::AutoDelete kv(SurfacePropsToKeyValues(pTextfile));

    for (KeyValues* pSurface = kv->GetFirstSubKey(); pSurface != nullptr; pSurface = pSurface->GetNextKey())
    {
        const char* pSurfaceName = pSurface->GetName();

        Box3DSurfaceProp values = {};
        // Update an existing material in place, else start from the base material.
        UtlSymId_t id = m_SurfaceProps.Find(pSurfaceName);
        if (id != m_SurfaceProps.InvalidIndex())
            values = m_SurfaceProps[id];
        else
            values = m_SurfaceProps[BaseMaterialIdx];

        ParseBox3DKVSchema(pSurface, kSurfacePropDescs, uint(std::size(kSurfacePropDescs)), &values);

        if (id == m_SurfaceProps.InvalidIndex())
            m_SurfaceProps[pSurfaceName] = values;
        else
            m_SurfaceProps[id] = values;
    }

    if (m_ShadowFallbackIdx < 0)
    {
        Box3DSurfaceProp prop = {};
        prop.data.physics = m_SurfaceProps[BaseMaterialIdx].data.physics;
        prop.data.physics.elasticity = 1e-3f;
        prop.data.physics.friction = 0.8f;
        m_SurfaceProps["$MATERIAL_INDEX_SHADOW"] = prop;
        m_ShadowFallbackIdx = int(m_SurfaceProps.Find("$MATERIAL_INDEX_SHADOW"));
    }

    return m_SurfaceProps.GetNumStrings();
}

int Box3DPhysicsSurfaceProps::SurfacePropCount(void) const
{
    return int(m_SurfaceProps.GetNumStrings());
}

//-------------------------------------------------------------------------------------------------

bool Box3DPhysicsSurfaceProps::IsReservedMaterialIndex(int nMaterialIndex) const
{
    return nMaterialIndex > 127;
}

int Box3DPhysicsSurfaceProps::GetReservedSurfaceIndex(const char* pSurfacePropName) const
{
    if (!Q_stricmp(pSurfacePropName, "$MATERIAL_INDEX_SHADOW"))
        return MATERIAL_INDEX_SHADOW;

    return -1;
}

int Box3DPhysicsSurfaceProps::GetReservedFallBack(int nMaterialIndex) const
{
    switch (nMaterialIndex)
    {
        case MATERIAL_INDEX_SHADOW:
            if (m_ShadowFallbackIdx >= 0)
                return m_ShadowFallbackIdx;
    }

    return 0;
}

UtlSymId_t Box3DPhysicsSurfaceProps::ResolveSurfaceIndex(int surfaceDataIndex) const
{
    if (IsReservedMaterialIndex(surfaceDataIndex))
        surfaceDataIndex = GetReservedFallBack(surfaceDataIndex);

    if (surfaceDataIndex < 0 || surfaceDataIndex >= int(m_SurfaceProps.GetNumStrings()))
        return BaseMaterialIdx;

    return UtlSymId_t(surfaceDataIndex);
}

int Box3DPhysicsSurfaceProps::GetSurfaceIndex(const char* pSurfacePropName) const
{
    if (pSurfacePropName[0] == '$')
    {
        const int nReserved = GetReservedSurfaceIndex(pSurfacePropName);
        if (nReserved >= 0)
            return nReserved;
    }

    UtlSymId_t nIndex = m_SurfaceProps.Find(pSurfacePropName);
    if (nIndex != m_SurfaceProps.InvalidIndex())
        return int(nIndex);

    return -1;
}

void Box3DPhysicsSurfaceProps::GetPhysicsProperties(
    int surfaceDataIndex, float* density, float* thickness, float* friction, float* elasticity) const
{
    const Box3DSurfaceProp& prop = m_SurfaceProps[ResolveSurfaceIndex(surfaceDataIndex)];
    if (density)
        *density = prop.data.physics.density;
    if (thickness)
        *thickness = prop.data.physics.thickness;
    if (friction)
        *friction = prop.data.physics.friction;
    if (elasticity)
        *elasticity = prop.data.physics.elasticity;
}

//-------------------------------------------------------------------------------------------------

surfacedata_t* Box3DPhysicsSurfaceProps::GetSurfaceData(int surfaceDataIndex)
{
    Box3DSurfaceProp& prop = m_SurfaceProps[ResolveSurfaceIndex(surfaceDataIndex)];
    return &prop.data;
}

const char* Box3DPhysicsSurfaceProps::GetString(unsigned short stringTableIndex) const
{
    return m_SoundStrings.String(stringTableIndex);
}

//-------------------------------------------------------------------------------------------------

const char* Box3DPhysicsSurfaceProps::GetPropName(int surfaceDataIndex) const
{
    if (IsReservedMaterialIndex(surfaceDataIndex))
        surfaceDataIndex = GetReservedFallBack(surfaceDataIndex);

    if (surfaceDataIndex < 0 || surfaceDataIndex >= int(m_SurfaceProps.GetNumStrings()))
        return nullptr;
    return m_SurfaceProps.String(surfaceDataIndex);
}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsSurfaceProps::SetWorldMaterialIndexTable(int* pMapArray, int mapSize)
{
    Log_Stub(LOG_VBox3D);
}

//-------------------------------------------------------------------------------------------------

void Box3DPhysicsSurfaceProps::GetPhysicsParameters(int surfaceDataIndex, surfacephysicsparams_t* pParamsOut) const
{
    if (!pParamsOut)
        return;

    const Box3DSurfaceProp& prop = m_SurfaceProps[ResolveSurfaceIndex(surfaceDataIndex)];
    *pParamsOut = prop.data.physics;
}

//-------------------------------------------------------------------------------------------------

ISaveRestoreOps* Box3DPhysicsSurfaceProps::GetMaterialIndexDataOps() const
{
    return &Box3DPhysicsMaterialIndexSaveOps::GetInstance();
}

//-------------------------------------------------------------------------------------------------

unsigned short Box3DPhysicsSurfaceProps::RegisterSound(const char* pName)
{
    return m_SoundStrings.AddString(pName);
}

//-------------------------------------------------------------------------------------------------

KeyValues* Box3DPhysicsSurfaceProps::SurfacePropsToKeyValues(const char* pszBuffer)
{
    return HeaderlessKVBufferToKeyValues(pszBuffer, "PhysProps");
}

//-------------------------------------------------------------------------------------------------

#if !defined(GAME_GMOD_64X)
void* Box3DPhysicsSurfaceProps::GetIVPMaterial(int nIndex)
{
    Log_Stub(LOG_VBox3D);
    return nullptr;
}

int Box3DPhysicsSurfaceProps::GetIVPMaterialIndex(const void* pMaterial) const
{
    Log_Stub(LOG_VBox3D);
    return (int)(uintp)(pMaterial);
}

void* Box3DPhysicsSurfaceProps::GetIVPManager(void)
{
    Log_Stub(LOG_VBox3D);
    return nullptr;
}

int Box3DPhysicsSurfaceProps::RemapIVPMaterialIndex(int nIndex) const
{
    Log_Stub(LOG_VBox3D);
    return nIndex;
}

const char* Box3DPhysicsSurfaceProps::GetReservedMaterialName(int nMaterialIndex) const
{
    switch (nMaterialIndex)
    {
        case MATERIAL_INDEX_SHADOW:
            return "$MATERIAL_INDEX_SHADOW";
    }

    return nullptr;
}
#endif

//-------------------------------------------------------------------------------------------------

Box3DPhysicsMaterialIndexSaveOps Box3DPhysicsMaterialIndexSaveOps::s_Instance;

void Box3DPhysicsMaterialIndexSaveOps::Save(const SaveRestoreFieldInfo_t& fieldInfo, ISave* pSave)
{
    int* pMaterialIdx = reinterpret_cast<int*>(fieldInfo.pField);

    const char* pMaterialName = Box3DPhysicsSurfaceProps::GetInstance().GetPropName(*pMaterialIdx);
    if (!pMaterialName)
        pMaterialName = Box3DPhysicsSurfaceProps::GetInstance().GetPropName(0);

    int nMaterialNameLength = V_strlen(pMaterialName) + 1;
    pSave->WriteInt(&nMaterialNameLength);
    pSave->WriteString(pMaterialName);
}

void Box3DPhysicsMaterialIndexSaveOps::Restore(const SaveRestoreFieldInfo_t& fieldInfo, IRestore* pRestore)
{
    int nMaterialNameLength = pRestore->ReadInt();
    char szMaterialName[2048];
    pRestore->ReadString(szMaterialName, sizeof(szMaterialName), nMaterialNameLength);

    int* pMaterialIdx = reinterpret_cast<int*>(fieldInfo.pField);
    *pMaterialIdx = Max(Box3DPhysicsSurfaceProps::GetInstance().GetSurfaceIndex(szMaterialName), 0);
}

bool Box3DPhysicsMaterialIndexSaveOps::IsEmpty(const SaveRestoreFieldInfo_t& fieldInfo)
{
    int* pMaterialIdx = reinterpret_cast<int*>(fieldInfo.pField);
    return !*pMaterialIdx;
}

void Box3DPhysicsMaterialIndexSaveOps::MakeEmpty(const SaveRestoreFieldInfo_t& fieldInfo)
{
    int* pMaterialIdx = reinterpret_cast<int*>(fieldInfo.pField);
    *pMaterialIdx = 0;
}
