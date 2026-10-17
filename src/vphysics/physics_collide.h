#pragma once

#include "mathlib/vector.h"
#include "tier0/valve_minmax_off.h"

#include "compact_builder/compact_surface.h"

#include "box3d/collision.h"

#include <atomic>
#include <memory>
#include <span>
#include <vector>

inline constexpr int COLLIDE_HEADER_SIZE = 28;

enum
{
	MODEL_TYPE_COMPACT_SURFACE = 0,
	MODEL_TYPE_NULL = 1,
};

struct physcollideheader_t
{
	int vphysicsID;
	short version;
	short modelType;
};

struct compactsurfaceheader_t
{
	int surfaceSize;
	float dragAxisAreas[3];
	int axisMapSize;
};

static_assert( sizeof( physcollideheader_t ) + sizeof( compactsurfaceheader_t ) == COLLIDE_HEADER_SIZE );

inline constexpr size_t MAX_COLLIDE_MAP_LEDGES = 32;
inline constexpr int MAX_COLLIDE_MAP_TRIANGLES = 512;
inline constexpr int MAX_COLLIDE_MAP_SPAN = 128;
inline constexpr int MAX_COLLIDE_MAP_RUNS = 14;

inline constexpr uint8_t COLLIDE_MAP_OCTANTS = 1;
inline constexpr uint8_t COLLIDE_MAP_CONTIGUOUS = 2;
inline constexpr uint8_t COLLIDE_MAP_RUNS = 4;

struct CollideMapEntry_t
{
	uint16_t m_nCount;
	uint8_t m_nFlags;
	uint8_t m_nRunCount;
	uint16_t m_nStart;
	uint8_t m_Runs[MAX_COLLIDE_MAP_RUNS];
	uint16_t m_OctantEdges[8];
	uint16_t m_nPointBlockOffset;
};

class CPhysConvex
{
public:
	struct Triangle_t
	{
		unsigned short m_nVertices[3];
		unsigned char m_nMaterialIndex;
	};

	CPhysConvex( std::vector<b3Vec3> points, std::vector<Triangle_t> triangles );
	~CPhysConvex();

	CPhysConvex( const CPhysConvex & ) = delete;
	CPhysConvex &operator=( const CPhysConvex & ) = delete;

	const b3HullData *GetHull() const
	{
		if ( m_nHullState.load( std::memory_order_acquire ) != HULL_BUILT )
			BuildHull();
		return m_pHull;
	}

	bool IsFlat() const
	{
		if ( m_nHullState.load( std::memory_order_acquire ) == HULL_UNCLASSIFIED )
			ClassifyHull();
		return m_bFlat;
	}

	const std::vector<unsigned char> &GetLedge() const { return m_Ledge; }

	void SetLedge( std::vector<unsigned char> ledge ) { m_Ledge = std::move( ledge ); }

	const std::vector<b3Vec3> &GetPoints() const { return m_Points; }

	const std::vector<Triangle_t> &GetTriangles() const { return m_Triangles; }

	std::vector<Triangle_t> &GetTriangles() { return m_Triangles; }

	unsigned int GetGameData() const { return m_nGameData; }

	void SetGameData( unsigned int nGameData );

	float GetVolume() const;
	float GetSurfaceArea() const;
	int FindTriangleMaterial( b3Vec3 localNormal, b3Vec3 localPoint ) const;
	int GetUniformMaterial() const;

private:
	enum : uint8_t
	{
		HULL_UNCLASSIFIED,
		HULL_FLAT,
		HULL_BUILT,
	};

	b3HullData *CreateHull( b3HullData *pBase ) const;
	b3HullData *CreateTriangleOffsetHull() const;
	void ClassifyHull() const;
	void BuildHull() const;

	std::vector<b3Vec3> m_Points;
	std::vector<Triangle_t> m_Triangles;
	mutable std::vector<b3Vec3> m_TriangleNormals;
	std::vector<unsigned char> m_Ledge;
	mutable b3HullData *m_pHull = nullptr;
	mutable std::atomic<uint8_t> m_nHullState = HULL_UNCLASSIFIED;
	mutable bool m_bFlat = false;
	unsigned int m_nGameData = 0;
};

struct CollideMesh_t
{
	struct Source_t
	{
		int m_nConvex;
		int m_nTriangle;
	};

	CollideMesh_t() = default;
	~CollideMesh_t();

	CollideMesh_t( const CollideMesh_t & ) = delete;
	CollideMesh_t &operator=( const CollideMesh_t & ) = delete;

	b3MeshData *m_pData = nullptr;
	std::vector<Source_t> m_Sources;
	int m_nMaterial = 0;
};

class CPhysCollideCompactSurface;

class CPhysCollide
{
public:
	virtual ~CPhysCollide() = default;

	const std::vector<std::unique_ptr<CollideMesh_t>> &GetStaticMeshes() const
	{
		if ( !m_bStaticMeshesBuilt.load( std::memory_order_acquire ) )
			BuildStaticMeshes();
		return m_StaticMeshes;
	}

	virtual const CPhysCollideCompactSurface *GetCompactSurface() const { return nullptr; }

	virtual const std::vector<std::unique_ptr<CPhysConvex>> &GetConvexes() const = 0;

	virtual int GetSerializationSize() const = 0;
	virtual void Serialize( char *pDest, bool bSwap ) const = 0;

	virtual int GetIndex() const = 0;

	virtual Vector GetMassCenter() const = 0;
	virtual void SetMassCenter( const Vector &massCenter ) = 0;

	virtual Vector GetOrthographicAreas() const = 0;
	virtual void SetOrthographicAreas( const Vector &areas ) = 0;

	virtual void SetTriangleMaterial( int nConvex, int nTriangle, int nMaterial ) = 0;

	virtual void OutputDebugInfo() const = 0;

	virtual float GetVolume() const;
	virtual float GetSurfaceArea() const;

	virtual void GetAABB( Vector *pMins, Vector *pMaxs, const b3Transform &transform ) const;
	virtual Vector GetExtent( const b3Transform &transform, const Vector &direction ) const;

private:
	void BuildStaticMeshes() const;

	mutable std::vector<std::unique_ptr<CollideMesh_t>> m_StaticMeshes;
	mutable std::atomic<bool> m_bStaticMeshesBuilt = false;
};

class CPhysCollideCompactSurface final : public CPhysCollide
{
public:
	struct LedgeTreeNode_t
	{
		b3Vec3 m_Center;
		float m_flRadius;
		int m_nRightChild;
		int m_nConvex;
	};

	struct LedgeSphere_t
	{
		b3Vec3 m_Center;
		float m_flRadius;
	};

	struct Ledge_t
	{
		size_t m_nOffset;
		LedgeSphere_t m_Sphere;
	};

	struct LedgeTree_t
	{
		std::vector<LedgeTreeNode_t> m_Nodes;
		std::vector<Ledge_t> m_Ledges;
		std::vector<CollideMapEntry_t> m_CollideMap;
		int m_nPointBlockCount = 0;
		bool m_bHasOuterHull = false;

		void Clear()
		{
			m_Nodes.clear();
			m_Ledges.clear();
			m_CollideMap.clear();
			m_nPointBlockCount = 0;
			m_bHasOuterHull = false;
		}
	};

	static CPhysCollideCompactSurface *Create( const unsigned char *pSurface, size_t nSize );

	static void operator delete( void *pMemory ) { ::operator delete( pMemory ); }

	CPhysCollideCompactSurface( const CPhysCollideCompactSurface & ) = delete;
	CPhysCollideCompactSurface &operator=( const CPhysCollideCompactSurface & ) = delete;

	const CPhysCollideCompactSurface *GetCompactSurface() const override { return this; }

	const std::vector<std::unique_ptr<CPhysConvex>> &GetConvexes() const override;

	const CollideMapEntry_t *GetCollideMapEntry( int nLedge ) const { return m_pCollideMap ? &m_pCollideMap[nLedge] : nullptr; }

	int GetPointBlockCount() const { return m_nPointBlockCount; }

	const LedgeSphere_t &GetLedgeSphere( int nLedge ) const { return m_Ledges[nLedge].m_Sphere; }

	int GetSerializationSize() const override;
	void Serialize( char *pDest, bool bSwap ) const override;

	int GetIndex() const override;
	void SetIndex( int nIndex );

	Vector GetMassCenter() const override;
	void SetMassCenter( const Vector &massCenter ) override;

	Vector GetOrthographicAreas() const override { return m_OrthographicAreas; }

	void SetOrthographicAreas( const Vector &areas ) override { m_OrthographicAreas = areas; }

	void SetTriangleMaterial( int nConvex, int nTriangle, int nMaterial ) override;

	void OutputDebugInfo() const override;

	float GetVolume() const override;
	float GetSurfaceArea() const override;

	void GetAABB( Vector *pMins, Vector *pMaxs, const b3Transform &transform ) const override;
	Vector GetExtent( const b3Transform &transform, const Vector &direction ) const override;

	std::span<const LedgeTreeNode_t> GetLedgeTree() const { return m_Nodes; }

	template<typename Function>
	void VisitLeafLedges( Function &&function, int nNode = 0 ) const
	{
		while ( m_Nodes[nNode].m_nRightChild >= 0 )
		{
			VisitLeafLedges( function, nNode + 1 );
			nNode = m_Nodes[nNode].m_nRightChild;
		}
		function( m_Nodes[nNode].m_nConvex );
	}

	int GetLedgeCount() const { return static_cast<int>( m_Ledges.size() ); }

	const unsigned char *GetLedge( int nLedge ) const { return m_pSurface + m_Ledges[nLedge].m_nOffset; }

	int GetSurfaceSize() const;

	b3Vec3 GetIVPMassCenter() const;
	Vector GetRotationInertia() const;

	float GetUpperLimitRadius() const;

	float GetBoundingRadius() const;

	static bool ReadLedgeTree( const unsigned char *pSurface, size_t nSize, LedgeTree_t &tree );

private:
	CPhysCollideCompactSurface() = default;

	void BuildConvexes() const;

	unsigned char *m_pSurface = nullptr;
	std::span<const LedgeTreeNode_t> m_Nodes;
	std::span<const Ledge_t> m_Ledges;
	const CollideMapEntry_t *m_pCollideMap = nullptr;
	int m_nPointBlockCount = 0;
	bool m_bHasOuterHull = false;
	mutable std::vector<std::unique_ptr<CPhysConvex>> m_Convexes;
	mutable std::atomic<bool> m_bConvexesBuilt = false;
	Vector m_OrthographicAreas = Vector( 1.0f, 1.0f, 1.0f );
};

std::unique_ptr<CPhysConvex> ReadLedgeConvex( const unsigned char *pLedge );
CPhysConvex *CreateConvexFromLedge( std::vector<unsigned char> ledge );
CPhysCollideCompactSurface *CreateCollideFromCompactSurface( const std::vector<unsigned char> &surface );
CPhysCollideCompactSurface *CreateCollideFromCompactSurface( const void *pSurface, size_t nSize, bool bSwap );
