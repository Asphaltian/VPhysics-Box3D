#pragma once

#include "precision.h"

#include <array>
#include <vector>

struct TemplateLine_t
{
	int m_nStart;
	int m_nEnd;
};

class CTemplatePolygon
{
public:
	explicit CTemplatePolygon( const std::vector<RealVec3_t> &points );

	bool AddSurface( RealVec3_t normal, const std::vector<TemplateLine_t> &lines );

	const std::vector<std::array<int, 3>> &GetTriangles() const { return m_Triangles; }

private:
	struct SurfacePoint_t
	{
		real_t x;
		real_t y;
	};

	static int GetProjectionAxis( RealVec3_t normal );
	static bool IsProjectionFlipped( RealVec3_t normal, int nAxis );

	bool TriangulateSurface( std::vector<TemplateLine_t> edges, std::vector<std::array<int, 3>> &triangles ) const;
	real_t GetKey( int nPoint, int a, int b ) const;
	real_t Left( const TemplateLine_t &edge, int nPoint ) const;
	real_t GetLineDistance( const TemplateLine_t &edge, int nPoint ) const;
	bool IsInsideSegment( const TemplateLine_t &edge, int nPoint ) const;
	bool Overlaps( const TemplateLine_t &f, const TemplateLine_t &g ) const;
	bool Intersects( const TemplateLine_t &f, const TemplateLine_t &g ) const;
	bool IsEar( const std::vector<TemplateLine_t> &edges, const TemplateLine_t &front, int c ) const;

	const std::vector<RealVec3_t> &m_Points;
	std::vector<SurfacePoint_t> m_SurfacePoints;
	std::vector<std::array<int, 3>> m_SurfaceTriangles;
	std::vector<std::array<int, 3>> m_Triangles;
};
