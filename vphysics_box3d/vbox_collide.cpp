//=================================================================================================
//
// CPhysCollide cooking / collision queries.
//
//=================================================================================================

#include "vbox_collide.h"

#include "cbase.h"
#include "mathlib/polyhedron.h"
#include "vbox_parse.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

Box3DPhysicsCollision Box3DPhysicsCollision::s_PhysicsCollision;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(
    Box3DPhysicsCollision, IPhysicsCollision, VPHYSICS_COLLISION_INTERFACE_VERSION, Box3DPhysicsCollision::GetInstance());

//-------------------------------------------------------------------------------------------------
//
// IVP compact-ledge (.phy) ingestion. Binary format copied from Source's studiobyteswap; only the
// leaf-ledge -> convex step targets Box3D. IVP data is already in metres; the sole transform is the
// axis swap Vec3( x, z, -y ).
//

#ifndef MAKEID
#    define MAKEID(d, c, b, a) (((int)(a) << 24) | ((int)(b) << 16) | ((int)(c) << 8) | ((int)(d)))
#endif

// Box3D rejects a hull with >=255 half-edges (~6*verts-12), so a round prop's full-detail hull fails
// to cook. Cap vertices so b3CreateHull simplifies below the limit (44 verts -> 252 half-edges).
static constexpr int kMaxHullVertices = 44;

namespace ivp_compat
{
    struct collideheader_t
    {
        int vphysicsID;
        short version;
        short modelType;
    };

    struct compactsurfaceheader_t
    {
        int surfaceSize;
        Vector dragAxisAreas;
        int axisMapSize;
    };

    struct compactsurface_t
    {
        float mass_center[3];
        float rotation_inertia[3];
        float upper_limit_radius;

        unsigned int max_factor_surface_deviation : 8;
        int byte_size : 24;
        int offset_ledgetree_root;
        int dummy[3];
    };

    struct compactledge_t
    {
        int c_point_offset;

        union
        {
            int ledgetree_node_offset;
            int client_data;
        };

        struct
        {
            uint has_children_flag : 2;
            int is_compact_flag : 2;
            uint dummy : 4;
            uint size_div_16 : 24;
        };

        short n_triangles;
        short for_future_use;
    };

    struct compactedge_t
    {
        uint start_point_index : 16;
        int opposite_index : 15;
        uint is_virtual : 1;
    };

    struct compacttriangle_t
    {
        uint tri_index : 12;
        uint pierce_index : 12;
        uint material_index : 7;
        uint is_virtual : 1;
        compactedge_t c_three_edges[3];
    };

    struct compactledgenode_t
    {
        int offset_right_node;
        int offset_compact_ledge;
        float center[3];
        float radius;
        unsigned char box_sizes[3];
        unsigned char free_0;

        const compactledge_t* GetCompactLedge() const
        {
            return (compactledge_t*)((char*)this + this->offset_compact_ledge);
        }
        const compactledgenode_t* GetLeftChild() const
        {
            return this + 1;
        }
        const compactledgenode_t* GetRightChild() const
        {
            return (compactledgenode_t*)((char*)this + this->offset_right_node);
        }
        bool IsTerminal() const
        {
            return this->offset_right_node == 0;
        }
    };

    static constexpr int IVP_COMPACT_SURFACE_SUPER_LEGACY = 0;
    static constexpr int IVP_COMPACT_SURFACE_ID = MAKEID('I', 'V', 'P', 'S');
    static constexpr int IVP_COMPACT_SURFACE_ID_SWAPPED = MAKEID('S', 'P', 'V', 'I');
    static constexpr int IVP_COMPACT_MOPP_ID = MAKEID('M', 'O', 'P', 'P');
    static constexpr int VPHYSICS_COLLISION_ID = MAKEID('V', 'P', 'H', 'Y');
    static constexpr short VPHYSICS_COLLISION_VERSION = 0x0100;

    enum
    {
        COLLIDE_POLY = 0,
        COLLIDE_MOPP = 1,
        COLLIDE_BALL = 2,
        COLLIDE_VIRTUAL = 3
    };

    static CPhysConvex* IVPLedgeToConvex(const compactledge_t* pLedge)
    {
        if (!pLedge->n_triangles)
            return nullptr;

        const char* pVertices = reinterpret_cast<const char*>(pLedge) + pLedge->c_point_offset;
        const compacttriangle_t* pTriangles = reinterpret_cast<const compacttriangle_t*>(pLedge + 1);
        const int nVertCount = pLedge->n_triangles * 3;

        CUtlVector<b3Vec3> verts;
        verts.SetCount(nVertCount);

        for (int i = 0; i < pLedge->n_triangles; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                static constexpr size_t IVPAlignedVectorSize = 16;
                const int nIndex = pTriangles[i].c_three_edges[j].start_point_index;
                const float* pVertex = reinterpret_cast<const float*>(pVertices + (nIndex * IVPAlignedVectorSize));
                verts[(i * 3) + j] = b3Vec3{ pVertex[0], pVertex[2], -pVertex[1] };
            }
        }

        b3HullData* pHull = b3CreateHull(verts.Base(), nVertCount, kMaxHullVertices);
        if (!pHull)
            return nullptr;

        CPhysConvex* pConvex = new CPhysConvex;
        pConvex->m_pHull = pHull;
        pConvex->m_nGameData = pLedge->client_data;
        return pConvex;
    }

    static void GetAllIVPEdges(const compactledgenode_t* pNode, CUtlVector<const compactledge_t*>& vecOut)
    {
        if (!pNode)
            return;

        if (!pNode->IsTerminal())
        {
            GetAllIVPEdges(pNode->GetRightChild(), vecOut);
            GetAllIVPEdges(pNode->GetLeftChild(), vecOut);
        }
        else
        {
            vecOut.AddToTail(pNode->GetCompactLedge());
        }
    }

    static CPhysCollide* DeserializeIVP_Poly(const compactsurface_t* pSurface)
    {
        const compactledgenode_t* pFirstLedgeNode = reinterpret_cast<const compactledgenode_t*>(
            reinterpret_cast<const char*>(pSurface) + pSurface->offset_ledgetree_root);

        CUtlVector<const compactledge_t*> ledges;
        GetAllIVPEdges(pFirstLedgeNode, ledges);

        CPhysCollide* pCollide = new CPhysCollide;
        for (int i = 0; i < ledges.Count(); i++)
        {
            CPhysConvex* pConvex = IVPLedgeToConvex(ledges[i]);
            if (pConvex)
                pCollide->m_Convexes.AddToTail(pConvex);
        }

        // A solid with no usable convexes must come back NULL, not empty: the game treats a null world
        // solid as "physics DLL can't build this" and falls back to virtual terrain (displacement props).
        if (pCollide->m_Convexes.Count() == 0)
        {
            delete pCollide;
            return nullptr;
        }
        return pCollide;
    }

    static CPhysCollide* DeserializeIVP_Poly(const collideheader_t* pCollideHeader)
    {
        const compactsurfaceheader_t* pSurfaceHeader = reinterpret_cast<const compactsurfaceheader_t*>(pCollideHeader + 1);
        const compactsurface_t* pSurface = reinterpret_cast<const compactsurface_t*>(pSurfaceHeader + 1);
        return DeserializeIVP_Poly(pSurface);
    }
} // namespace ivp_compat

//-------------------------------------------------------------------------------------------------

static CPhysConvex* HullToConvex(b3HullData* pHull)
{
    if (!pHull)
        return nullptr;

    CPhysConvex* pConvex = new CPhysConvex;
    pConvex->m_pHull = pHull;
    return pConvex;
}

const b3HullData* CPhysConvex::GetSimHull()
{
    if (m_pSimHull)
        return m_pSimHull;
    if (!m_pHull)
        return nullptr;

    const int nFaceCount = m_pHull->faceCount;
    if (nFaceCount < 4)
        return m_pHull;

    const b3Plane* pPlanes = b3GetHullPlanes(m_pHull);

    // Source .phy hulls are authored pre-shrunk by IVP's collision tolerance (0.25in per face); box3d
    // rests contacts at the true surface, so re-inflate each face plane outward by that distance and
    // re-enumerate the vertices (triple-plane intersections kept only if inside every offset plane).
    // Offsetting the planes, not the vertices, keeps rotated/diagonal convexes from shearing.
    const float flInflate = SourceToBox::Distance(0.25f);
    const float flInsideEps = SourceToBox::Distance(0.01f);
    const float flDedupEps = SourceToBox::Distance(0.01f);

    CUtlVector<b3Vec3> verts;
    for (int i = 0; i < nFaceCount; i++)
    {
        const b3Vec3 ni = pPlanes[i].normal;
        for (int j = i + 1; j < nFaceCount; j++)
        {
            const b3Vec3 nj = pPlanes[j].normal;
            for (int k = j + 1; k < nFaceCount; k++)
            {
                const b3Vec3 nk = pPlanes[k].normal;

                const b3Vec3 cjk = b3Cross(nj, nk);
                const float det = b3Dot(ni, cjk);
                if (fabsf(det) < 1e-6f)
                    continue;

                const float di = pPlanes[i].offset + flInflate;
                const float dj = pPlanes[j].offset + flInflate;
                const float dk = pPlanes[k].offset + flInflate;

                const b3Vec3 cki = b3Cross(nk, ni);
                const b3Vec3 cij = b3Cross(ni, nj);
                const b3Vec3 x = b3MulSV(1.0f / det, b3Add(b3Add(b3MulSV(di, cjk), b3MulSV(dj, cki)), b3MulSV(dk, cij)));

                bool bInside = true;
                for (int m = 0; m < nFaceCount; m++)
                {
                    if (b3Dot(pPlanes[m].normal, x) - (pPlanes[m].offset + flInflate) > flInsideEps)
                    {
                        bInside = false;
                        break;
                    }
                }
                if (!bInside)
                    continue;

                bool bDup = false;
                for (int v = 0; v < verts.Count(); v++)
                {
                    if (b3Length(b3Sub(verts[v], x)) < flDedupEps)
                    {
                        bDup = true;
                        break;
                    }
                }
                if (!bDup)
                    verts.AddToTail(x);
            }
        }
    }

    if (verts.Count() < 4)
        return m_pHull;

    m_pSimHull = b3CreateHull(verts.Base(), verts.Count(), kMaxHullVertices);
    return m_pSimHull ? m_pSimHull : m_pHull;
}

CPhysConvex* Box3DPhysicsCollision::ConvexFromVerts(Vector** pVerts, int vertCount)
{
    CUtlVector<b3Vec3> points;
    points.SetCount(vertCount);
    for (int i = 0; i < vertCount; i++)
        points[i] = SourceToBox::Distance(*pVerts[i]);

    return HullToConvex(b3CreateHull(points.Base(), vertCount, kMaxHullVertices));
}

CPhysConvex* Box3DPhysicsCollision::ConvexFromPlanes(float* pPlanes, int planeCount, float mergeDistance)
{
    Log_Stub(LOG_VBox3D);
    return nullptr;
}

float Box3DPhysicsCollision::ConvexVolume(CPhysConvex* pConvex)
{
    if (!pConvex || !pConvex->m_pHull)
        return 0.0f;

    // Density of 1 makes the reported mass equal to the volume.
    const b3MassData massData = b3ComputeHullMass(pConvex->m_pHull, 1.0f);
    return BoxToSource::Volume(massData.mass);
}

float Box3DPhysicsCollision::ConvexSurfaceArea(CPhysConvex* pConvex)
{
    Log_Stub(LOG_VBox3D);
    return 0.0f;
}

void Box3DPhysicsCollision::SetConvexGameData(CPhysConvex* pConvex, unsigned int gameData)
{
    if (pConvex)
        pConvex->m_nGameData = gameData;
}

void Box3DPhysicsCollision::ConvexFree(CPhysConvex* pConvex)
{
    if (!pConvex)
        return;

    if (pConvex->m_pHull)
        b3DestroyHull(pConvex->m_pHull);
    if (pConvex->m_pSimHull)
        b3DestroyHull(pConvex->m_pSimHull);
    delete pConvex;
}

CPhysConvex* Box3DPhysicsCollision::BBoxToConvex(const Vector& mins, const Vector& maxs)
{
    b3Vec3 corners[8];
    for (int i = 0; i < 8; i++)
    {
        const Vector corner((i & 1) ? maxs.x : mins.x, (i & 2) ? maxs.y : mins.y, (i & 4) ? maxs.z : mins.z);
        corners[i] = SourceToBox::Distance(corner);
    }

    return HullToConvex(b3CreateHull(corners, 8, 8));
}

CPhysConvex* Box3DPhysicsCollision::ConvexFromConvexPolyhedron(const CPolyhedron& ConvexPolyhedron)
{
    CUtlVector<b3Vec3> points;
    points.SetCount(ConvexPolyhedron.iVertexCount);
    for (int i = 0; i < ConvexPolyhedron.iVertexCount; i++)
        points[i] = SourceToBox::Distance(ConvexPolyhedron.pVertices[i]);

    return HullToConvex(b3CreateHull(points.Base(), ConvexPolyhedron.iVertexCount, kMaxHullVertices));
}

void Box3DPhysicsCollision::ConvexesFromConvexPolygon(
    const Vector& vPolyNormal, const Vector* pPoints, int iPointCount, CPhysConvex** pOutput)
{
    Log_Stub(LOG_VBox3D);
}

//-------------------------------------------------------------------------------------------------

CPhysPolysoup* Box3DPhysicsCollision::PolysoupCreate()
{
    return new CPhysPolysoup;
}

void Box3DPhysicsCollision::PolysoupDestroy(CPhysPolysoup* pSoup)
{
    delete pSoup;
}

void Box3DPhysicsCollision::PolysoupAddTriangle(
    CPhysPolysoup* pSoup, const Vector& a, const Vector& b, const Vector& c, int materialIndex7bits)
{
    if (!pSoup)
        return;

    pSoup->m_Vertices.AddToTail(SourceToBox::Distance(a));
    pSoup->m_Vertices.AddToTail(SourceToBox::Distance(b));
    pSoup->m_Vertices.AddToTail(SourceToBox::Distance(c));
    pSoup->m_MaterialIndices.AddToTail((uint8)materialIndex7bits);
}

CPhysCollide* Box3DPhysicsCollision::ConvertPolysoupToCollide(CPhysPolysoup* pSoup, bool useMOPP)
{
    if (!pSoup || pSoup->m_Vertices.Count() < 3)
        return nullptr;

    const int triangleCount = pSoup->m_Vertices.Count() / 3;

    CUtlVector<int32> indices;
    indices.SetCount(pSoup->m_Vertices.Count());
    for (int i = 0; i < indices.Count(); i++)
        indices[i] = i;

    b3MeshDef def = {};
    def.vertices = pSoup->m_Vertices.Base();
    def.vertexCount = pSoup->m_Vertices.Count();
    def.indices = indices.Base();
    def.triangleCount = triangleCount;
    def.materialIndices = pSoup->m_MaterialIndices.Base();
    def.weldVertices = true;
    def.weldTolerance = SourceToBox::Distance(0.1f);
    def.identifyEdges = true; // adjacency info so props don't catch on internal triangle edges

    b3MeshData* pMesh = b3CreateMesh(&def, nullptr, 0);
    if (!pMesh)
        return nullptr;

    CPhysCollide* pCollide = new CPhysCollide;
    pCollide->m_pMesh = pMesh;
    return pCollide;
}

//-------------------------------------------------------------------------------------------------

CPhysCollide* Box3DPhysicsCollision::ConvertConvexToCollide(CPhysConvex** pConvex, int convexCount)
{
    return ConvertConvexToCollideParams(pConvex, convexCount, convertconvexparams_t{});
}

CPhysCollide* Box3DPhysicsCollision::ConvertConvexToCollideParams(
    CPhysConvex** pConvex, int convexCount, const convertconvexparams_t& convertParams)
{
    CPhysCollide* pCollide = new CPhysCollide;

    b3Vec3 weightedCenter = { 0.0f, 0.0f, 0.0f };
    float totalMass = 0.0f;

    for (int i = 0; i < convexCount; i++)
    {
        if (!pConvex[i])
            continue;

        pCollide->m_Convexes.AddToTail(pConvex[i]);

        if (pConvex[i]->m_pHull)
        {
            const b3MassData massData = b3ComputeHullMass(pConvex[i]->m_pHull, 1.0f);
            weightedCenter.x += massData.mass * massData.center.x;
            weightedCenter.y += massData.mass * massData.center.y;
            weightedCenter.z += massData.mass * massData.center.z;
            totalMass += massData.mass;
        }
    }

    if (totalMass > 0.0f)
    {
        const b3Vec3 center = { weightedCenter.x / totalMass, weightedCenter.y / totalMass, weightedCenter.z / totalMass };
        pCollide->m_vecMassCenter = BoxToSource::Distance(center);
    }

    return pCollide;
}

void Box3DPhysicsCollision::DestroyCollide(CPhysCollide* pCollide)
{
    if (!pCollide)
        return;

    for (int i = 0; i < pCollide->m_Convexes.Count(); i++)
    {
        if (pCollide->m_Convexes[i]->m_pHull)
            b3DestroyHull(pCollide->m_Convexes[i]->m_pHull);
        if (pCollide->m_Convexes[i]->m_pSimHull)
            b3DestroyHull(pCollide->m_Convexes[i]->m_pSimHull);
        delete pCollide->m_Convexes[i];
    }

    if (pCollide->m_pMesh)
        b3DestroyMesh(pCollide->m_pMesh);

    delete pCollide;
}

//-------------------------------------------------------------------------------------------------

int Box3DPhysicsCollision::CollideSize(CPhysCollide* pCollide)
{
    Log_Stub(LOG_VBox3D);
    return 0;
}

int Box3DPhysicsCollision::CollideWrite(char* pDest, CPhysCollide* pCollide, bool bSwap)
{
    Log_Stub(LOG_VBox3D);
    return 0;
}

CPhysCollide* Box3DPhysicsCollision::UnserializeCollide(char* pBuffer, int size, int index)
{
    Log_Stub(LOG_VBox3D);
    return nullptr;
}

float Box3DPhysicsCollision::CollideVolume(CPhysCollide* pCollide)
{
    if (!pCollide)
        return 0.0f;

    float volume = 0.0f;
    for (int i = 0; i < pCollide->m_Convexes.Count(); i++)
        volume += ConvexVolume(pCollide->m_Convexes[i]);

    return volume;
}

float Box3DPhysicsCollision::CollideSurfaceArea(CPhysCollide* pCollide)
{
    Log_Stub(LOG_VBox3D);
    return 0.0f;
}

// The support point: the collide's farthest vertex along the direction, in world space.
Vector Box3DPhysicsCollision::CollideGetExtent(
    const CPhysCollide* pCollide, const Vector& collideOrigin, const QAngle& collideAngles, const Vector& direction)
{
    if (!pCollide)
        return collideOrigin;

    const b3Transform xf = { SourceToBox::Distance(collideOrigin), SourceToBox::Angle(collideAngles) };
    const b3Vec3 localDir = b3InvRotateVector(xf.q, SourceToBox::Unitless(direction));

    float flBest = -FLT_MAX;
    b3Vec3 best = { 0.0f, 0.0f, 0.0f };
    bool bFound = false;

    for (int i = 0; i < pCollide->m_Convexes.Count(); i++)
    {
        const b3HullData* pHull = pCollide->m_Convexes[i]->m_pHull;
        if (!pHull)
            continue;

        const b3Vec3* pPoints = b3GetHullPoints(pHull);
        for (int p = 0; p < pHull->vertexCount; p++)
        {
            const float flDot = b3Dot(pPoints[p], localDir);
            if (flDot > flBest)
            {
                flBest = flDot;
                best = pPoints[p];
                bFound = true;
            }
        }
    }

    if (!bFound)
        return collideOrigin;

    return BoxToSource::Distance(b3TransformPoint(xf, best));
}

void Box3DPhysicsCollision::CollideGetAABB(
    Vector* pMins, Vector* pMaxs, const CPhysCollide* pCollide, const Vector& collideOrigin, const QAngle& collideAngles)
{
    if (!pCollide)
    {
        if (pMins)
            *pMins = collideOrigin;
        if (pMaxs)
            *pMaxs = collideOrigin;
        return;
    }

    b3Transform xf;
    xf.p = SourceToBox::Distance(collideOrigin);
    xf.q = SourceToBox::Angle(collideAngles);

    b3AABB bounds = {};
    bool bHasBounds = false;
    for (int i = 0; i < pCollide->m_Convexes.Count(); i++)
    {
        if (!pCollide->m_Convexes[i]->m_pHull)
            continue;

        const b3AABB hullBounds = b3ComputeHullAABB(pCollide->m_Convexes[i]->m_pHull, xf);
        bounds = bHasBounds ? b3AABB_Union(bounds, hullBounds) : hullBounds;
        bHasBounds = true;
    }

    if (pCollide->m_pMesh)
    {
        const b3AABB meshBounds = b3ComputeMeshAABB(pCollide->m_pMesh, xf, b3Vec3{ 1.0f, 1.0f, 1.0f });
        bounds = bHasBounds ? b3AABB_Union(bounds, meshBounds) : meshBounds;
        bHasBounds = true;
    }

    if (!bHasBounds)
    {
        if (pMins)
            *pMins = collideOrigin;
        if (pMaxs)
            *pMaxs = collideOrigin;
        return;
    }

    if (pMins)
        *pMins = BoxToSource::Distance(bounds.lowerBound);
    if (pMaxs)
        *pMaxs = BoxToSource::Distance(bounds.upperBound);
}

void Box3DPhysicsCollision::CollideGetMassCenter(CPhysCollide* pCollide, Vector* pOutMassCenter)
{
    if (pOutMassCenter)
        *pOutMassCenter = pCollide ? pCollide->m_vecMassCenter : vec3_origin;
}

void Box3DPhysicsCollision::CollideSetMassCenter(CPhysCollide* pCollide, const Vector& massCenter)
{
    if (pCollide)
        pCollide->m_vecMassCenter = massCenter;
}

Vector Box3DPhysicsCollision::CollideGetOrthographicAreas(const CPhysCollide* pCollide)
{
    return pCollide ? pCollide->m_vecOrthographicAreas : Vector(1.0f, 1.0f, 1.0f);
}

void Box3DPhysicsCollision::CollideSetOrthographicAreas(CPhysCollide* pCollide, const Vector& areas)
{
    if (pCollide)
        pCollide->m_vecOrthographicAreas = areas;
}

int Box3DPhysicsCollision::CollideIndex(const CPhysCollide* pCollide)
{
    Log_Stub(LOG_VBox3D);
    return 0;
}

CPhysCollide* Box3DPhysicsCollision::BBoxToCollide(const Vector& mins, const Vector& maxs)
{
    CPhysConvex* pConvex = BBoxToConvex(mins, maxs);
    if (!pConvex)
        return nullptr;

    return ConvertConvexToCollide(&pConvex, 1);
}

int Box3DPhysicsCollision::GetConvexesUsedInCollideable(
    const CPhysCollide* pCollideable, CPhysConvex** pOutputArray, int iOutputArrayLimit)
{
    if (!pCollideable)
        return 0;

    const int count = Min(pCollideable->m_Convexes.Count(), iOutputArrayLimit);
    for (int i = 0; i < count; i++)
        pOutputArray[i] = pCollideable->m_Convexes[i];

    return count;
}

//-------------------------------------------------------------------------------------------------

namespace
{
    // Hit back-off. Must clear Box3D's shape-cast contact zone (1.25 * B3_LINEAR_SLOP ~= 0.25"), or
    // every subsequent trace starts at fraction 0 with no normal.
    constexpr float kTraceDistEpsilon = 0.15f;

    // Back the hit fraction off along the normal by DIST_EPSILON, matching IVP/Source trace behaviour.
    float CalculateSourceFraction(const Vector& vecDelta, float flFraction, const Vector& vecNormal)
    {
        const float flLength = vecDelta.Length();
        if (flLength == 0.0f)
            return 0.0f;

        const Vector vecDir = vecDelta / flLength;
        float flHitLength = flLength * flFraction;

        const float flDot = DotProduct(vecDir, vecNormal);
        if (flDot < 0.0f)
            flHitLength += kTraceDistEpsilon / flDot;

        return Max(flHitLength, 0.0f) / flLength;
    }

    // Trace a ray (point) or swept box against a collide's convex hulls, in the collide's local frame.
    // Polysoup meshes are not traced.
    void TraceBoxVsCollide(
        const Ray_t& ray, const CPhysCollide* pCollide, const Vector& collideOrigin, const QAngle& collideAngles,
        trace_t* pTrace)
    {
        if (!pTrace)
            return;

        ClearTrace(pTrace);

        // m_Start is the swept box's centre; m_Start + m_StartOffset is the entity origin, which is
        // only used for the reported positions.
        const Vector vecCenter = ray.m_Start;
        const Vector vecStart = ray.m_Start + ray.m_StartOffset;
        pTrace->startpos = vecStart;
        pTrace->endpos = vecStart + ray.m_Delta;

        if (!pCollide || (pCollide->m_Convexes.Count() == 0 && !pCollide->m_pMesh))
            return;

        // The collide's world transform, and the ray taken into the collide's local space.
        const b3Transform xf = { SourceToBox::Distance(collideOrigin), SourceToBox::Angle(collideAngles) };
        const b3Vec3 localOrigin = b3InvTransformPoint(xf, SourceToBox::Distance(vecCenter));
        const b3Vec3 localTranslation = b3InvRotateVector(xf.q, SourceToBox::Distance(ray.m_Delta));

        const bool bIsPoint = ray.m_Extents.LengthSqr() < 1e-6f;

        // Point trace: a simple ray cast, solid when it starts inside.
        if (bIsPoint)
        {
            b3CastOutput best = {};
            best.fraction = 1.0f;
            bool bHit = false;

            for (int i = 0; i < pCollide->m_Convexes.Count(); i++)
            {
                const b3HullData* pHull = pCollide->m_Convexes[i]->m_pHull;
                if (!pHull)
                    continue;

                const b3RayCastInput in = { localOrigin, localTranslation, 1.0f };
                const b3CastOutput out = b3RayCastHull(pHull, &in);
                if (out.hit && (!bHit || out.fraction < best.fraction))
                {
                    best = out;
                    bHit = true;
                }
            }

            if (pCollide->m_pMesh)
            {
                const b3Mesh mesh = { pCollide->m_pMesh, b3Vec3{ 1.0f, 1.0f, 1.0f } };
                const b3RayCastInput in = { localOrigin, localTranslation, 1.0f };
                const b3CastOutput out = b3RayCastMesh(&mesh, &in);
                if (out.hit && (!bHit || out.fraction < best.fraction))
                {
                    best = out;
                    bHit = true;
                }
            }

            if (!bHit)
                return;

            Vector vecNormal = BoxToSource::Unitless(b3RotateVector(xf.q, best.normal));
            if (vecNormal.LengthSqr() < 1e-6f)
                vecNormal = ray.m_Delta.LengthSqr() > 1e-6f ? -ray.m_Delta : Vector(0.0f, 0.0f, 1.0f);
            VectorNormalize(vecNormal);

            pTrace->fraction = best.fraction;
            pTrace->endpos = vecStart + ray.m_Delta * best.fraction;
            pTrace->plane.normal = vecNormal;
            pTrace->plane.dist = DotProduct(pTrace->endpos, vecNormal);
            pTrace->contents = CONTENTS_SOLID;
            pTrace->allsolid = best.fraction == 0.0f;
            pTrace->startsolid = best.fraction == 0.0f;
            return;
        }

        // Swept box: the box corners taken into the collide's local space, and the box as a hull for
        // the penetration case below.
        b3Vec3 boxPoints[8];
        int k = 0;
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2)
                for (int sz = -1; sz <= 1; sz += 2)
                {
                    const Vector vecCorner = vecCenter
                        + Vector(sx * ray.m_Extents.x, sy * ray.m_Extents.y, sz * ray.m_Extents.z);
                    boxPoints[k++] = b3InvTransformPoint(xf, SourceToBox::Distance(vecCorner));
                }

        // On an initial overlap the shape cast returns fraction 0 with no normal; recover one via SAT
        // and, like IVP, ignore penetrations the sweep is moving out of.
        const b3Transform boxWorldXf = { SourceToBox::Distance(vecCenter), SourceToBox::Angle(vec3_angle) };
        const b3Transform boxToHull = b3InvMulTransforms(xf, boxWorldXf);
        const b3BoxHull boxHull = b3MakeBoxHull(
            SourceToBox::Distance(ray.m_Extents.x), SourceToBox::Distance(ray.m_Extents.y),
            SourceToBox::Distance(ray.m_Extents.z));

        // Unswept box (stuck checks): solid only when actually intersecting. Derived from m_Delta --
        // GMod x64's engine Ray_t lacks m_pWorldAxisTransform, so m_IsSwept/m_IsRay read garbage.
        const bool bIsSwept = ray.m_Delta.LengthSqr() != 0.0f;
        if (!bIsSwept)
        {
            for (int i = 0; i < pCollide->m_Convexes.Count(); i++)
            {
                const b3HullData* pHull = pCollide->m_Convexes[i]->m_pHull;
                if (!pHull)
                    continue;

                b3LocalManifoldPoint points[8];
                b3LocalManifold manifold = {};
                manifold.points = points;
                b3SATCache cache = {};
                b3CollideHulls(&manifold, ARRAYSIZE(points), pHull, &boxHull.base, boxToHull, &cache);

                float flSeparation = FLT_MAX;
                for (int p = 0; p < manifold.pointCount; p++)
                    flSeparation = Min(flSeparation, manifold.points[p].separation);

                if (manifold.pointCount > 0 && flSeparation < -SourceToBox::Distance(0.02f))
                {
                    // b3CollideHulls' normal points from the hull into the box: out of the surface.
                    Vector vecNormal = BoxToSource::Unitless(b3RotateVector(xf.q, manifold.normal));
                    VectorNormalize(vecNormal);

                    pTrace->fraction = 0.0f;
                    pTrace->endpos = vecStart;
                    pTrace->plane.normal = vecNormal;
                    pTrace->plane.dist = DotProduct(pTrace->endpos, vecNormal);
                    pTrace->contents = CONTENTS_SOLID;
                    pTrace->allsolid = true;
                    pTrace->startsolid = true;
                    return;
                }
            }

            // Mesh overlap has no push-out normal; report solid, up normal.
            if (pCollide->m_pMesh)
            {
                const b3Mesh mesh = { pCollide->m_pMesh, b3Vec3{ 1.0f, 1.0f, 1.0f } };
                const b3ShapeProxy proxy = { boxPoints, 8, 0.0f };
                if (b3OverlapMesh(&mesh, b3Transform_identity, &proxy))
                {
                    pTrace->fraction = 0.0f;
                    pTrace->endpos = vecStart;
                    pTrace->plane.normal = Vector(0.0f, 0.0f, 1.0f);
                    pTrace->plane.dist = DotProduct(pTrace->endpos, pTrace->plane.normal);
                    pTrace->contents = CONTENTS_SOLID;
                    pTrace->allsolid = true;
                    pTrace->startsolid = true;
                    return;
                }
            }
            return;
        }

        // Solid only when genuinely penetrating; a flush contact must block-and-slide, not freeze.
        const float flDeepPenetration = SourceToBox::Distance(0.5f);

        bool bHit = false, bStartSolid = false, bEndSolid = false;
        float flBestFraction = 1.0f;
        Vector vecBestNormal = vec3_origin;

        for (int i = 0; i < pCollide->m_Convexes.Count(); i++)
        {
            const b3HullData* pHull = pCollide->m_Convexes[i]->m_pHull;
            if (!pHull)
                continue;

            b3ShapeCastInput in = {};
            in.proxy.points = boxPoints;
            in.proxy.count = 8;
            in.proxy.radius = 0.0f;
            in.translation = localTranslation;
            in.maxFraction = 1.0f;
            in.canEncroach = false;
            const b3CastOutput out = b3ShapeCastHull(pHull, &in);

            if (out.hit && out.fraction > 0.0f && b3LengthSquared(out.normal) > 1e-8f)
            {
                // Ordinary swept hit; b3ShapeCastHull's normal already points out of the hull.
                Vector vecNormal = BoxToSource::Unitless(b3RotateVector(xf.q, out.normal));
                VectorNormalize(vecNormal);
                if (DotProduct(ray.m_Delta, vecNormal) < 0.0f && (!bHit || out.fraction < flBestFraction))
                {
                    flBestFraction = out.fraction;
                    vecBestNormal = vecNormal;
                    bHit = true;
                }
            }
            else if (out.hit)
            {
                b3LocalManifoldPoint points[8];
                b3LocalManifold manifold = {};
                manifold.points = points;
                b3SATCache cache = {};
                b3CollideHulls(&manifold, ARRAYSIZE(points), pHull, &boxHull.base, boxToHull, &cache);
                if (manifold.pointCount <= 0)
                    continue;

                float flSeparation = 0.0f;
                for (int p = 0; p < manifold.pointCount; p++)
                    flSeparation = Min(flSeparation, manifold.points[p].separation);
                const bool bDeep = flSeparation < -flDeepPenetration;
                if (bDeep)
                    bStartSolid = true;

                // Normal points out of the surface. A contact-zone hit only blocks a move advancing into
                // the face; near-parallel slides must pass, else clip-and-retry wedges the move to zero.
                Vector vecNormal = BoxToSource::Unitless(b3RotateVector(xf.q, manifold.normal));
                VectorNormalize(vecNormal);
                if (DotProduct(ray.m_Delta, vecNormal) < -0.01f && (!bHit || flBestFraction > 0.0f))
                {
                    flBestFraction = 0.0f;
                    vecBestNormal = vecNormal;
                    bHit = true;
                    if (bDeep)
                        bEndSolid = true;
                }
            }
        }

        // Swept box vs concave mesh; initial overlap is a miss (unswept path handles stuck boxes).
        if (pCollide->m_pMesh)
        {
            const b3Mesh mesh = { pCollide->m_pMesh, b3Vec3{ 1.0f, 1.0f, 1.0f } };
            b3ShapeCastInput in = {};
            in.proxy.points = boxPoints;
            in.proxy.count = 8;
            in.proxy.radius = 0.0f;
            in.translation = localTranslation;
            in.maxFraction = 1.0f;
            in.canEncroach = false;
            const b3CastOutput out = b3ShapeCastMesh(&mesh, &in);
            if (out.hit && out.fraction > 0.0f && b3LengthSquared(out.normal) > 1e-8f)
            {
                Vector vecNormal = BoxToSource::Unitless(b3RotateVector(xf.q, out.normal));
                VectorNormalize(vecNormal);
                if (DotProduct(ray.m_Delta, vecNormal) < 0.0f && (!bHit || out.fraction < flBestFraction))
                {
                    flBestFraction = out.fraction;
                    vecBestNormal = vecNormal;
                    bHit = true;
                }
            }
        }

        if (!bHit)
        {
            pTrace->fraction = 1.0f;
            pTrace->endpos = vecStart + ray.m_Delta;
            return;
        }

        pTrace->plane.normal = vecBestNormal;
        pTrace->fraction = CalculateSourceFraction(ray.m_Delta, flBestFraction, vecBestNormal);
        pTrace->endpos = vecStart + ray.m_Delta * pTrace->fraction;
        pTrace->plane.dist = DotProduct(pTrace->endpos, vecBestNormal);
        pTrace->contents = CONTENTS_SOLID;
        pTrace->allsolid = bStartSolid && bEndSolid;
        pTrace->startsolid = bStartSolid;
    }
} // namespace

void Box3DPhysicsCollision::TraceBox(
    const Vector& start, const Vector& end, const Vector& mins, const Vector& maxs, const CPhysCollide* pCollide,
    const Vector& collideOrigin, const QAngle& collideAngles, trace_t* ptr)
{
    Ray_t ray;
    ray.Init(start, end, mins, maxs);
    TraceBoxVsCollide(ray, pCollide, collideOrigin, collideAngles, ptr);
}

void Box3DPhysicsCollision::TraceBox(
    const Ray_t& ray, const CPhysCollide* pCollide, const Vector& collideOrigin, const QAngle& collideAngles, trace_t* ptr)
{
    TraceBoxVsCollide(ray, pCollide, collideOrigin, collideAngles, ptr);
}

void Box3DPhysicsCollision::TraceBox(
    const Ray_t& ray, unsigned int contentsMask, IConvexInfo* pConvexInfo, const CPhysCollide* pCollide,
    const Vector& collideOrigin, const QAngle& collideAngles, trace_t* ptr)
{
    TraceBoxVsCollide(ray, pCollide, collideOrigin, collideAngles, ptr);
}

// Overlap test between two collides; the swept case is unsupported, same as Volt (nothing uses it).
void Box3DPhysicsCollision::TraceCollide(
    const Vector& start, const Vector& end, const CPhysCollide* pSweepCollide, const QAngle& sweepAngles,
    const CPhysCollide* pCollide, const Vector& collideOrigin, const QAngle& collideAngles, trace_t* ptr)
{
    if (!ptr)
        return;

    ClearTrace(ptr);
    ptr->startpos = start;
    ptr->endpos = start;

    if (!pSweepCollide || !pCollide)
        return;

    if (start != end)
    {
        Log_Stub(LOG_VBox3D);
        return;
    }

    const b3Transform xfSweep = { SourceToBox::Distance(start), SourceToBox::Angle(sweepAngles) };
    const b3Transform xfHit = { SourceToBox::Distance(collideOrigin), SourceToBox::Angle(collideAngles) };
    const b3Transform sweepToHit = b3InvMulTransforms(xfHit, xfSweep);

    for (int i = 0; i < pCollide->m_Convexes.Count(); i++)
    {
        const b3HullData* pHull = pCollide->m_Convexes[i]->m_pHull;
        if (!pHull)
            continue;

        for (int j = 0; j < pSweepCollide->m_Convexes.Count(); j++)
        {
            const b3HullData* pSweepHull = pSweepCollide->m_Convexes[j]->m_pHull;
            if (!pSweepHull)
                continue;

            b3DistanceInput input = {};
            input.proxyA = { b3GetHullPoints(pHull), pHull->vertexCount, 0.0f };
            input.proxyB = { b3GetHullPoints(pSweepHull), pSweepHull->vertexCount, 0.0f };
            input.transform = sweepToHit;
            input.useRadii = true;

            b3SimplexCache cache = {};
            const b3DistanceOutput out = b3ShapeDistance(&input, &cache, nullptr, 0);
            if (out.distance < B3_OVERLAP_SLOP)
            {
                ptr->fraction = 0.0f;
                ptr->contents = CONTENTS_SOLID;
                ptr->allsolid = true;
                ptr->startsolid = true;
                return;
            }
        }
    }
}

bool Box3DPhysicsCollision::IsBoxIntersectingCone(
    const Vector& boxAbsMins, const Vector& boxAbsMaxs, const truncatedcone_t& cone)
{
    Log_Stub(LOG_VBox3D);
    return false;
}

void Box3DPhysicsCollision::VCollideLoad(vcollide_t* pOutput, int solidCount, const char* pBuffer, int size, bool swap)
{
    if (swap)
        return;

    pOutput->solidCount = solidCount;
    pOutput->solids = new CPhysCollide*[solidCount];

    const char* pCursor = pBuffer;
    for (int i = 0; i < solidCount; i++)
    {
        pOutput->solids[i] = nullptr;

        const int solidSize = *reinterpret_cast<const int*>(pCursor);
        pCursor += sizeof(int);

        const ivp_compat::collideheader_t* pCollideHeader = reinterpret_cast<const ivp_compat::collideheader_t*>(pCursor);

        if (pCollideHeader->vphysicsID == ivp_compat::VPHYSICS_COLLISION_ID)
        {
            if (pCollideHeader->version != ivp_compat::VPHYSICS_COLLISION_VERSION)
                Log_Warning(LOG_VBox3D, "Solid with unknown version: 0x%x, may crash!\n", pCollideHeader->version);

            if (pCollideHeader->modelType == ivp_compat::COLLIDE_POLY)
                pOutput->solids[i] = ivp_compat::DeserializeIVP_Poly(pCollideHeader);
            else
                Log_Warning(
                    LOG_VBox3D, "Unsupported solid type 0x%x on solid %d. Skipping...\n", (int)pCollideHeader->modelType, i);
        }
        else
        {
            // Legacy .phy: just a dumped compact surface.
            const ivp_compat::compactsurface_t* pCompactSurface = reinterpret_cast<const ivp_compat::compactsurface_t*>(
                pCursor);
            const int legacyModelType = pCompactSurface->dummy[2];
            if (legacyModelType == ivp_compat::IVP_COMPACT_SURFACE_SUPER_LEGACY
                || legacyModelType == ivp_compat::IVP_COMPACT_SURFACE_ID
                || legacyModelType == ivp_compat::IVP_COMPACT_SURFACE_ID_SWAPPED)
                pOutput->solids[i] = ivp_compat::DeserializeIVP_Poly(pCompactSurface);
            else
                Log_Warning(LOG_VBox3D, "Unsupported legacy solid type 0x%x on solid %d. Skipping...\n", legacyModelType, i);
        }

        pCursor += solidSize;
    }

    // The rest of the buffer is the KeyValues text.
    const int keyValuesSize = size - (int)(uintp(pCursor) - uintp(pBuffer));
    pOutput->pKeyValues = new char[keyValuesSize + 1];
    V_memcpy(pOutput->pKeyValues, pCursor, keyValuesSize);
    pOutput->pKeyValues[keyValuesSize] = '\0';
    pOutput->descSize = keyValuesSize;
    pOutput->isPacked = false;
#ifdef GAME_ASW_OR_NEWER
    pOutput->pUserData = nullptr;
#endif
}

void Box3DPhysicsCollision::VCollideUnload(vcollide_t* pVCollide)
{
    for (int i = 0; i < pVCollide->solidCount; i++)
        DestroyCollide(pVCollide->solids[i]);

    delete[] pVCollide->solids;
    delete[] pVCollide->pKeyValues;
    V_memset(pVCollide, 0, sizeof(*pVCollide));
}

IVPhysicsKeyParser* Box3DPhysicsCollision::VPhysicsKeyParserCreate(const char* pKeyData)
{
    return CreateVPhysicsKeyParser(pKeyData, false);
}

IVPhysicsKeyParser* Box3DPhysicsCollision::VPhysicsKeyParserCreate(vcollide_t* pVCollide)
{
    // GMod x64's engine calls this overload (not the const char* one) to parse a model's physics
    // keyvalues. Returning nullptr faults the engine when it iterates the parser.
    return CreateVPhysicsKeyParser(pVCollide->pKeyValues ? pVCollide->pKeyValues : "", pVCollide->isPacked);
}

void Box3DPhysicsCollision::VPhysicsKeyParserDestroy(IVPhysicsKeyParser* pParser)
{
    delete pParser;
}

int Box3DPhysicsCollision::CreateDebugMesh(CPhysCollide const* pCollisionModel, Vector** outVerts)
{
    Log_Stub(LOG_VBox3D);
    return 0;
}

void Box3DPhysicsCollision::DestroyDebugMesh(int vertCount, Vector* outVerts)
{
    Log_Stub(LOG_VBox3D);
}

// Empty collision query. The engine virtual-calls the returned object, so it must be non-null.
namespace
{
    class Box3DDummyCollisionQuery final : public ICollisionQuery
    {
    public:
        int ConvexCount() override
        {
            return 0;
        }
        int TriangleCount(int) override
        {
            return 0;
        }
        unsigned int GetGameData(int) override
        {
            return 0;
        }
        void GetTriangleVerts(int, int, Vector* verts) override
        {
            if (verts)
                verts[0] = verts[1] = verts[2] = vec3_origin;
        }
        void SetTriangleVerts(int, int, const Vector*) override
        {
        }
        int GetTriangleMaterialIndex(int, int) override
        {
            return 0;
        }
        void SetTriangleMaterialIndex(int, int, int) override
        {
        }
    };
} // namespace

ICollisionQuery* Box3DPhysicsCollision::CreateQueryModel(CPhysCollide* pCollide)
{
    return new Box3DDummyCollisionQuery;
}

void Box3DPhysicsCollision::DestroyQueryModel(ICollisionQuery* pQuery)
{
    delete static_cast<Box3DDummyCollisionQuery*>(pQuery);
}

IPhysicsCollision* Box3DPhysicsCollision::ThreadContextCreate()
{
    return this;
}

void Box3DPhysicsCollision::ThreadContextDestroy(IPhysicsCollision* pThreadContex)
{
}

// Displacement terrain: the game hands us its triangles through the event handler.
CPhysCollide* Box3DPhysicsCollision::CreateVirtualMesh(const virtualmeshparams_t& params)
{
    if (!params.pMeshEventHandler)
        return nullptr;

    virtualmeshlist_t list;
    params.pMeshEventHandler->GetVirtualMesh(params.userData, &list);
    if (list.triangleCount <= 0 || list.vertexCount <= 0)
        return nullptr;

    CUtlVector<b3Vec3> verts;
    verts.SetCount(list.vertexCount);
    for (int i = 0; i < list.vertexCount; i++)
        verts[i] = SourceToBox::Distance(list.pVerts[i]);

    // Reverse each triangle's winding: Box3D meshes are one-sided and Source displacement triangles
    // wind the opposite way, so without this props fall straight through the terrain.
    CUtlVector<int32> indices;
    indices.SetCount(list.triangleCount * 3);
    for (int i = 0; i < list.triangleCount; i++)
    {
        indices[i * 3 + 0] = list.indices[i * 3 + 0];
        indices[i * 3 + 1] = list.indices[i * 3 + 2];
        indices[i * 3 + 2] = list.indices[i * 3 + 1];
    }

    b3MeshDef def = {};
    def.vertices = verts.Base();
    def.vertexCount = list.vertexCount;
    def.indices = indices.Base();
    def.triangleCount = list.triangleCount;
    def.useMedianSplit = true; // displacement patches are grid-structured
    def.identifyEdges = true;  // adjacency info so props don't catch on internal triangle edges

    b3MeshData* pMesh = b3CreateMesh(&def, nullptr, 0);
    if (!pMesh)
        return nullptr;

    CPhysCollide* pCollide = new CPhysCollide;
    pCollide->m_pMesh = pMesh;
    return pCollide;
}

bool Box3DPhysicsCollision::SupportsVirtualMesh()
{
    return true;
}

bool Box3DPhysicsCollision::GetBBoxCacheSize(int* pCachedSize, int* pCachedCount)
{
    if (pCachedSize)
        *pCachedSize = 0;
    if (pCachedCount)
        *pCachedCount = 0;
    return false;
}

CPolyhedron* Box3DPhysicsCollision::PolyhedronFromConvex(CPhysConvex* const pConvex, bool bUseTempPolyhedron)
{
    Log_Stub(LOG_VBox3D);
    return nullptr;
}

void Box3DPhysicsCollision::OutputDebugInfo(const CPhysCollide* pCollide)
{
    Log_Stub(LOG_VBox3D);
}

unsigned int Box3DPhysicsCollision::ReadStat(int statID)
{
    return 0;
}

float Box3DPhysicsCollision::CollideGetRadius(const CPhysCollide* pCollide)
{
    if (!pCollide)
        return 0.0f;
    // Inscribed-sphere radius, approximated as half the smallest local AABB extent.
    Vector mins, maxs;
    CollideGetAABB(&mins, &maxs, pCollide, vec3_origin, vec3_angle);
    const Vector ext = (maxs - mins) * 0.5f;
    return Min(Min(ext.x, ext.y), ext.z);
}

void* Box3DPhysicsCollision::VCollideAllocUserData(vcollide_t* pVCollide, size_t userDataSize)
{
#ifdef GAME_ASW_OR_NEWER
    VCollideFreeUserData(pVCollide);
    if (userDataSize)
        pVCollide->pUserData = malloc(userDataSize);
    return pVCollide->pUserData;
#else
    return nullptr;
#endif
}

void Box3DPhysicsCollision::VCollideFreeUserData(vcollide_t* pVCollide)
{
#ifdef GAME_ASW_OR_NEWER
    if (pVCollide->pUserData)
    {
        free(pVCollide->pUserData);
        pVCollide->pUserData = nullptr;
    }
#endif
}

void Box3DPhysicsCollision::VCollideCheck(vcollide_t* pVCollide, const char* pName)
{
    Log_Stub(LOG_VBox3D);
}

bool Box3DPhysicsCollision::TraceBoxAA(const Ray_t& ray, const CPhysCollide* pCollide, trace_t* ptr)
{
    TraceBox(ray, pCollide, vec3_origin, vec3_angle, ptr);
    return true;
}

void Box3DPhysicsCollision::DuplicateAndScale(vcollide_t* pOut, const vcollide_t* pIn, float flScale)
{
    Log_Stub(LOG_VBox3D);
}
