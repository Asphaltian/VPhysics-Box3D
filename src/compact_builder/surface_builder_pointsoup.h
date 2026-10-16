#pragma once

#include "precision.h"

#include <vector>

std::vector<unsigned char> BuildPointSoupLedge( const std::vector<RealVec3_t> &points );

class CPlaneSet
{
public:
	void AddPlane( RealVec3_t normal, real_t flDistance );
	std::vector<RealVec3_t> ComputeVertices( real_t flMergeDistance ) const;

private:
	struct Plane_t
	{
		RealVec3_t m_Normal;
		real_t m_flDistance;
	};

	std::vector<Plane_t> m_Planes;
};
