#pragma once

#include <vector>

struct CompactSurfaceParams_t
{
	const std::vector<unsigned char> *m_pForcedOuterHull = nullptr;
	bool m_bBuildOuterHull = false;
};

std::vector<unsigned char> BuildCompactSurface( const std::vector<const std::vector<unsigned char> *> &ledges, const CompactSurfaceParams_t &params );
