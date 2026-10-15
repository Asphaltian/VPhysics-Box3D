#pragma once

#include "mathlib/vector.h"
#include "tier0/valve_minmax_off.h"

#include "box3d/collision.h"

#include <memory>
#include <vector>

class CPhysConvex
{
public:
	struct Triangle_t
	{
		unsigned short m_nVertices[3];
		unsigned char m_nMaterialIndex;
	};

	static b3HullData *CreateHull( const b3Vec3 *pPoints, int nPointCount );

	CPhysConvex( std::vector<Vector> vertices, std::vector<Triangle_t> triangles );
	explicit CPhysConvex( b3HullData *pHull );
	~CPhysConvex();

	CPhysConvex( const CPhysConvex & ) = delete;
	CPhysConvex &operator=( const CPhysConvex & ) = delete;

	const b3HullData *GetHull() const { return m_pHull; }
	const std::vector<Vector> &GetVertices() const { return m_Vertices; }
	const std::vector<Triangle_t> &GetTriangles() const { return m_Triangles; }
	std::vector<Triangle_t> &GetTriangles() { return m_Triangles; }

	unsigned int GetGameData() const { return m_nGameData; }
	void SetGameData( unsigned int nGameData ) { m_nGameData = nGameData; }

	float GetVolume() const;
	float GetSurfaceArea() const;

private:
	std::vector<Vector> m_Vertices;
	std::vector<Triangle_t> m_Triangles;
	b3HullData *m_pHull = nullptr;
	unsigned int m_nGameData = 0;
};

class CPhysCollide
{
public:
	explicit CPhysCollide( std::vector<std::unique_ptr<CPhysConvex>> convexes );
	~CPhysCollide();

	CPhysCollide( const CPhysCollide & ) = delete;
	CPhysCollide &operator=( const CPhysCollide & ) = delete;

	const std::vector<std::unique_ptr<CPhysConvex>> &GetConvexes() const { return m_Convexes; }
	const b3CompoundData *GetCompound() const { return m_pCompound; }

	const Vector &GetMassCenter() const { return m_MassCenter; }
	void SetMassCenter( const Vector &massCenter ) { m_MassCenter = massCenter; }

	const Vector &GetRotationInertia() const { return m_RotationInertia; }
	void SetRotationInertia( const Vector &rotationInertia ) { m_RotationInertia = rotationInertia; }

	const Vector &GetOrthographicAreas() const { return m_OrthographicAreas; }
	void SetOrthographicAreas( const Vector &areas ) { m_OrthographicAreas = areas; }

	int GetIndex() const { return m_nIndex; }
	void SetIndex( int nIndex ) { m_nIndex = nIndex; }

	bool HasOuterHull() const { return m_bHasOuterHull; }
	void SetHasOuterHull( bool bHasOuterHull ) { m_bHasOuterHull = bHasOuterHull; }

	float GetVolume() const;
	float GetSurfaceArea() const;

	void GetAABB( Vector *pMins, Vector *pMaxs, const b3Transform &transform ) const;
	Vector GetExtent( const b3Transform &transform, const Vector &direction ) const;

private:
	std::vector<std::unique_ptr<CPhysConvex>> m_Convexes;
	b3CompoundData *m_pCompound = nullptr;
	Vector m_MassCenter = Vector( 0.0f, 0.0f, 0.0f );
	Vector m_RotationInertia = Vector( 0.0f, 0.0f, 0.0f );
	Vector m_OrthographicAreas = Vector( 1.0f, 1.0f, 1.0f );
	int m_nIndex = 0;
	bool m_bHasOuterHull = false;
};
