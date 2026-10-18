#pragma once

#include "math_util.h"

#include <vector>

std::vector<unsigned char> BuildPointSoupLedge( const std::vector<DoubleVec3_t> &points );

class CPlaneSet
{
public:
	void AddPlane( DoubleVec3_t normal, double flDistance );
	std::vector<DoubleVec3_t> ComputeVertices( double flMergeDistance ) const;

private:
	struct Plane_t
	{
		DoubleVec3_t m_Normal;
		double m_flDistance;
	};

	std::vector<Plane_t> m_Planes;
};
