
#pragma once

enum
{
    MATERIAL_INDEX_SHADOW = 0xF000,
};

struct Box3DSurfaceProp
{
    surfacedata_t data;
};

class Box3DPhysicsMaterialIndexSaveOps : public CDefSaveRestoreOps
{
public:
    void Save(const SaveRestoreFieldInfo_t& fieldInfo, ISave* pSave) override;
    void Restore(const SaveRestoreFieldInfo_t& fieldInfo, IRestore* pRestore) override;

    bool IsEmpty(const SaveRestoreFieldInfo_t& fieldInfo) override;
    void MakeEmpty(const SaveRestoreFieldInfo_t& fieldInfo) override;

    static Box3DPhysicsMaterialIndexSaveOps& GetInstance()
    {
        return s_Instance;
    }

private:
    static Box3DPhysicsMaterialIndexSaveOps s_Instance;
};

class Box3DPhysicsSurfaceProps final : public IPhysicsSurfaceProps
{
public:
    Box3DPhysicsSurfaceProps();

    int ParseSurfaceData(const char* pFilename, const char* pTextfile) override;
    int SurfacePropCount(void) const override;

    int GetSurfaceIndex(const char* pSurfacePropName) const override;
    void GetPhysicsProperties(
        int surfaceDataIndex, float* density, float* thickness, float* friction, float* elasticity) const override;

    surfacedata_t* GetSurfaceData(int surfaceDataIndex) override;
    const char* GetString(unsigned short stringTableIndex) const override;

    const char* GetPropName(int surfaceDataIndex) const override;

    void SetWorldMaterialIndexTable(int* pMapArray, int mapSize) override;

    void GetPhysicsParameters(int surfaceDataIndex, surfacephysicsparams_t* pParamsOut) const override;

    ISaveRestoreOps* GetMaterialIndexDataOps() const override_portal2;

    // GMod-specific internal gubbins that was exposed in the public interface.
#if !defined(GAME_GMOD_64X)
    void* GetIVPMaterial(int nIndex) override_gmod;
    int GetIVPMaterialIndex(const void* pMaterial) const override_gmod;
    void* GetIVPManager(void) override_gmod;
    int RemapIVPMaterialIndex(int nIndex) const override_gmod;
    const char* GetReservedMaterialName(int nMaterialIndex) const override_gmod;
#endif

public:
    static Box3DPhysicsSurfaceProps& GetInstance()
    {
        return s_PhysicsSurfaceProps;
    }

    unsigned short RegisterSound(const char* pName);

private:
    static Box3DPhysicsSurfaceProps s_PhysicsSurfaceProps;

    bool IsReservedMaterialIndex(int nMaterialIndex) const;
    int GetReservedSurfaceIndex(const char* pSurfacePropName) const;
    int GetReservedFallBack(int nMaterialIndex) const;
    UtlSymId_t ResolveSurfaceIndex(int surfaceDataIndex) const;

    CUtlStringMap<Box3DSurfaceProp> m_SurfaceProps;
    CUtlSymbolTable m_SoundStrings;
    int m_ShadowFallbackIdx = -1;

    static constexpr UtlSymId_t BaseMaterialIdx = UtlSymId_t(0);

    KeyValues* SurfacePropsToKeyValues(const char* pszBuffer);
};
