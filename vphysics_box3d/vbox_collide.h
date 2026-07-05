//=================================================================================================
//
// CPhysCollide and friends!
//
//=================================================================================================

#pragma once

//-------------------------------------------------------------------------------------------------
//
// Box3D uses opaque integer handles and separates cooked geometry (b3HullData/b3MeshData) from
// body-bound shapes, so unlike Volt we can't reinterpret-cast a shape pointer. CPhysConvex/
// CPhysCollide are real owned wrappers over cooked Box3D geometry; objects instantiate them
// per-body via b3CreateHullShape / b3CreateMeshShape.
//

// A single cooked convex piece.
class CPhysConvex
{
public:
    b3HullData* m_pHull = nullptr;    // owned, pristine -- used by traces and static bodies
    b3HullData* m_pSimHull = nullptr; // owned, inflated by the rest margin; lazily cooked
    unsigned int m_nGameData = 0;

    // Pristine hull grown by the mesh rest offset so props rest a hair proud of flat ground instead of
    // z-fighting it. Dynamic bodies only; traces keep m_pHull. Cooked on first use.
    const b3HullData* GetSimHull();
};

// A collision model: one or more convex pieces, or a single concave triangle mesh (world/displacement).
class CPhysCollide
{
public:
    CUtlVector<CPhysConvex*> m_Convexes; // owned convex pieces (empty if a mesh)
    b3MeshData* m_pMesh = nullptr;       // owned concave mesh, null if convex

    Vector m_vecMassCenter = vec3_origin;
    Vector m_vecOrthographicAreas = Vector(1.0f, 1.0f, 1.0f);
};

// Triangle soup accumulator used to build a concave mesh.
class CPhysPolysoup
{
public:
    CUtlVector<b3Vec3> m_Vertices;       // 3 verts per triangle, Box3D space
    CUtlVector<uint8> m_MaterialIndices; // 1 per triangle
};

//-------------------------------------------------------------------------------------------------

// Josh: Suprise! This is not an app system! Just an interface...
class Box3DPhysicsCollision final : public IPhysicsCollision
{
public:
    CPhysConvex* ConvexFromVerts(Vector** pVerts, int vertCount) override;
    CPhysConvex* ConvexFromPlanes(float* pPlanes, int planeCount, float mergeDistance) override;
    float ConvexVolume(CPhysConvex* pConvex) override;

    float ConvexSurfaceArea(CPhysConvex* pConvex) override;
    void SetConvexGameData(CPhysConvex* pConvex, unsigned int gameData) override;
    void ConvexFree(CPhysConvex* pConvex) override;
    CPhysConvex* BBoxToConvex(const Vector& mins, const Vector& maxs) override;
    CPhysConvex* ConvexFromConvexPolyhedron(const CPolyhedron& ConvexPolyhedron) override;
    void ConvexesFromConvexPolygon(
        const Vector& vPolyNormal, const Vector* pPoints, int iPointCount, CPhysConvex** pOutput) override;

    CPhysPolysoup* PolysoupCreate() override;
    void PolysoupDestroy(CPhysPolysoup* pSoup) override;
    void PolysoupAddTriangle(
        CPhysPolysoup* pSoup, const Vector& a, const Vector& b, const Vector& c, int materialIndex7bits) override;
    CPhysCollide* ConvertPolysoupToCollide(CPhysPolysoup* pSoup, bool useMOPP) override;

    CPhysCollide* ConvertConvexToCollide(CPhysConvex** pConvex, int convexCount) override;
    CPhysCollide* ConvertConvexToCollideParams(
        CPhysConvex** pConvex, int convexCount, const convertconvexparams_t& convertParams) override;
    void DestroyCollide(CPhysCollide* pCollide) override;

    int CollideSize(CPhysCollide* pCollide) override;
    int CollideWrite(char* pDest, CPhysCollide* pCollide, bool bSwap = false) override;
    CPhysCollide* UnserializeCollide(char* pBuffer, int size, int index) override;

    float CollideVolume(CPhysCollide* pCollide) override;
    float CollideSurfaceArea(CPhysCollide* pCollide) override;

    Vector CollideGetExtent(
        const CPhysCollide* pCollide, const Vector& collideOrigin, const QAngle& collideAngles,
        const Vector& direction) override;

    void CollideGetAABB(
        Vector* pMins, Vector* pMaxs, const CPhysCollide* pCollide, const Vector& collideOrigin,
        const QAngle& collideAngles) override;

    void CollideGetMassCenter(CPhysCollide* pCollide, Vector* pOutMassCenter) override;
    void CollideSetMassCenter(CPhysCollide* pCollide, const Vector& massCenter) override;
    Vector CollideGetOrthographicAreas(const CPhysCollide* pCollide) override;
    void CollideSetOrthographicAreas(CPhysCollide* pCollide, const Vector& areas) override;

    int CollideIndex(const CPhysCollide* pCollide) override;

    CPhysCollide* BBoxToCollide(const Vector& mins, const Vector& maxs) override;
    int GetConvexesUsedInCollideable(
        const CPhysCollide* pCollideable, CPhysConvex** pOutputArray, int iOutputArrayLimit) override;

    void TraceBox(
        const Vector& start, const Vector& end, const Vector& mins, const Vector& maxs, const CPhysCollide* pCollide,
        const Vector& collideOrigin, const QAngle& collideAngles, trace_t* ptr) override;
    void TraceBox(
        const Ray_t& ray, const CPhysCollide* pCollide, const Vector& collideOrigin, const QAngle& collideAngles,
        trace_t* ptr) override;
    void TraceBox(
        const Ray_t& ray, unsigned int contentsMask, IConvexInfo* pConvexInfo, const CPhysCollide* pCollide,
        const Vector& collideOrigin, const QAngle& collideAngles, trace_t* ptr) override;

    void TraceCollide(
        const Vector& start, const Vector& end, const CPhysCollide* pSweepCollide, const QAngle& sweepAngles,
        const CPhysCollide* pCollide, const Vector& collideOrigin, const QAngle& collideAngles, trace_t* ptr) override;

    bool IsBoxIntersectingCone(const Vector& boxAbsMins, const Vector& boxAbsMaxs, const truncatedcone_t& cone) override;

    void VCollideLoad(vcollide_t* pOutput, int solidCount, const char* pBuffer, int size, bool swap = false) override;
    void VCollideUnload(vcollide_t* pVCollide) override;

    IVPhysicsKeyParser* VPhysicsKeyParserCreate(const char* pKeyData) override;
    IVPhysicsKeyParser* VPhysicsKeyParserCreate(vcollide_t* pVCollide) override_asw;
    void VPhysicsKeyParserDestroy(IVPhysicsKeyParser* pParser) override;

    int CreateDebugMesh(CPhysCollide const* pCollisionModel, Vector** outVerts) override;
    void DestroyDebugMesh(int vertCount, Vector* outVerts) override;

    ICollisionQuery* CreateQueryModel(CPhysCollide* pCollide) override;
    void DestroyQueryModel(ICollisionQuery* pQuery) override;

    IPhysicsCollision* ThreadContextCreate() override;
    void ThreadContextDestroy(IPhysicsCollision* pThreadContex) override;

    CPhysCollide* CreateVirtualMesh(const virtualmeshparams_t& params) override;
    bool SupportsVirtualMesh() override;

    bool GetBBoxCacheSize(int* pCachedSize, int* pCachedCount) override;

    CPolyhedron* PolyhedronFromConvex(CPhysConvex* const pConvex, bool bUseTempPolyhedron) override;

    void OutputDebugInfo(const CPhysCollide* pCollide) override;
    unsigned int ReadStat(int statID) override;

    float CollideGetRadius(const CPhysCollide* pCollide) override_asw;

    void* VCollideAllocUserData(vcollide_t* pVCollide, size_t userDataSize) override_asw;
    void VCollideFreeUserData(vcollide_t* pVCollide) override_asw;
    void VCollideCheck(vcollide_t* pVCollide, const char* pName) override_asw;

    bool TraceBoxAA(const Ray_t& ray, const CPhysCollide* pCollide, trace_t* ptr) override_csgo;

    void DuplicateAndScale(vcollide_t* pOut, const vcollide_t* pIn, float flScale) override_csgo;

public:
    static Box3DPhysicsCollision& GetInstance()
    {
        return s_PhysicsCollision;
    }

private:
    static Box3DPhysicsCollision s_PhysicsCollision;
};
