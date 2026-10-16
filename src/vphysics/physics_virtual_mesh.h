#pragma once

#include "physics_collide.h"

#include "vphysics/virtualmesh.h"

#include <atomic>

class CPhysCollideVirtualMesh final : public CPhysCollide
{
public:
	explicit CPhysCollideVirtualMesh( const virtualmeshparams_t &params );

	const std::vector<std::unique_ptr<CPhysConvex>> &GetConvexes() const override;

	int GetSerializationSize() const override { return static_cast<int>( m_PackedHull.size() ); }

	void Serialize( char *pDest, bool bSwap ) const override;

	int GetIndex() const override { return 0; }

	Vector GetMassCenter() const override;

	void SetMassCenter( const Vector & ) override {}

	Vector GetOrthographicAreas() const override { return Vector( 1.0f, 1.0f, 1.0f ); }

	void SetOrthographicAreas( const Vector & ) override {}

	void SetTriangleMaterial( int nConvex, int nTriangle, int nMaterial ) override;

	void OutputDebugInfo() const override;

private:
	IVirtualMeshEvent *m_pHandler;
	void *m_pUserData;
	std::vector<unsigned char> m_PackedHull;
	mutable std::vector<std::unique_ptr<CPhysConvex>> m_Convexes;
	mutable std::atomic<bool> m_bConvexesBuilt = false;
};
