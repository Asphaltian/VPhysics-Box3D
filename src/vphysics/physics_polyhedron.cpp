#include "physics_polyhedron.h"

#include "compact_builder/compact_surface.h"
#include "compact_builder/template_polygon.h"
#include "conversion.h"
#include "physics_collide.h"

#include "mathlib/polyhedron.h"
#include "tier0/valve_minmax_off.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <new>
#include <vector>

#include <xmmintrin.h>

static constexpr float NORMAL_LENGTH_EPSILON = 1e-10f;

void CPolyhedron_AllocByNew::Release()
{
	this->~CPolyhedron_AllocByNew();
	::operator delete( this );
}

static void AssignPolyhedronStorage( CPolyhedron &polyhedron, unsigned char *pStorage, unsigned short iVertices, unsigned short iLines, unsigned short iIndices, unsigned short iPolygons )
{
	polyhedron.iVertexCount = iVertices;
	polyhedron.iLineCount = iLines;
	polyhedron.iIndexCount = iIndices;
	polyhedron.iPolygonCount = iPolygons;
	polyhedron.pVertices = reinterpret_cast<Vector *>( pStorage );
	polyhedron.pLines = reinterpret_cast<Polyhedron_IndexedLine_t *>( polyhedron.pVertices + iVertices );
	polyhedron.pIndices = reinterpret_cast<Polyhedron_IndexedLineReference_t *>( polyhedron.pLines + iLines );
	polyhedron.pPolygons = reinterpret_cast<Polyhedron_IndexedPolygon_t *>( polyhedron.pIndices + iIndices );
}

static size_t GetPolyhedronStorageSize( unsigned short iVertices, unsigned short iLines, unsigned short iIndices, unsigned short iPolygons )
{
	return iVertices * sizeof( Vector ) + iLines * sizeof( Polyhedron_IndexedLine_t ) + iIndices * sizeof( Polyhedron_IndexedLineReference_t ) + iPolygons * sizeof( Polyhedron_IndexedPolygon_t );
}

CPolyhedron_AllocByNew *CPolyhedron_AllocByNew::Allocate( unsigned short iVertices, unsigned short iLines, unsigned short iIndices, unsigned short iPolygons )
{
	void *pMemory = ::operator new( sizeof( CPolyhedron_AllocByNew ) + GetPolyhedronStorageSize( iVertices, iLines, iIndices, iPolygons ) );
	CPolyhedron_AllocByNew *pPolyhedron = new ( pMemory ) CPolyhedron_AllocByNew;
	AssignPolyhedronStorage( *pPolyhedron, reinterpret_cast<unsigned char *>( pPolyhedron + 1 ), iVertices, iLines, iIndices, iPolygons );
	return pPolyhedron;
}

class CPolyhedron_TempMemory final : public CPolyhedron
{
public:
	void Release() override {}

	std::vector<unsigned char> m_Storage;
};

static CPolyhedron_TempMemory s_TempPolyhedron;

CPolyhedron *GetTempPolyhedron( unsigned short iVertices, unsigned short iLines, unsigned short iIndices, unsigned short iPolygons )
{
	s_TempPolyhedron.m_Storage.resize( GetPolyhedronStorageSize( iVertices, iLines, iIndices, iPolygons ) );
	AssignPolyhedronStorage( s_TempPolyhedron, s_TempPolyhedron.m_Storage.data(), iVertices, iLines, iIndices, iPolygons );
	return &s_TempPolyhedron;
}

CPhysConvex *CreateTriangleConvex( const Vector &a, const Vector &b, const Vector &c )
{
	std::vector<unsigned char> ledge = WriteTriangleLedge( ToDouble( SourceToIVP( a ) ), ToDouble( SourceToIVP( b ) ), ToDouble( SourceToIVP( c ) ) );
	return ledge.empty() ? nullptr : CreateConvexFromLedge( std::move( ledge ) );
}

CPhysConvex *CreateConvexFromPolyhedron( const CPolyhedron &polyhedron )
{
	std::vector<DoubleVec3_t> templatePoints( polyhedron.iVertexCount );
	for ( int i = 0; i < polyhedron.iVertexCount; ++i )
		templatePoints[i] = ToDouble( SourceToIVP( polyhedron.pVertices[i] ) );

	CTemplatePolygon templatePolygon( templatePoints );
	std::vector<TemplateLine_t> lines;

	for ( int nPolygon = 0; nPolygon < polyhedron.iPolygonCount; ++nPolygon )
	{
		const Polyhedron_IndexedPolygon_t &polygon = polyhedron.pPolygons[nPolygon];
		const DoubleVec3_t normal = ToDouble( SourceToIVP( polygon.polyNormal ) );

		lines.clear();
		for ( int i = 0; i < polygon.iIndexCount; ++i )
		{
			const Polyhedron_IndexedLineReference_t &reference = polyhedron.pIndices[polygon.iFirstIndex + i];
			const Polyhedron_IndexedLine_t &line = polyhedron.pLines[reference.iLineIndex];
			const int nReverse = reference.iEndPointIndex ? 1 : 0;
			lines.push_back( { line.iPointIndices[nReverse], line.iPointIndices[1 - nReverse] } );
		}

		if ( !templatePolygon.AddSurface( normal, lines ) )
			return nullptr;
	}

	if ( templatePolygon.GetTriangles().empty() )
		return nullptr;

	return CreateConvexFromLedge( WriteCompactLedge( templatePoints, templatePolygon.GetTriangles() ) );
}

CPolyhedron *CreatePolyhedronFromConvex( const CPhysConvex &convex, bool bUseTempPolyhedron )
{
	const std::vector<b3Vec3> &points = convex.GetPoints();
	const std::vector<CPhysConvex::Triangle_t> &triangles = convex.GetTriangles();

	std::vector<int> remap( points.size(), -1 );
	for ( const CPhysConvex::Triangle_t &triangle : triangles )
	{
		for ( unsigned short nVertex : triangle.m_nVertices )
			remap[nVertex] = 0;
	}

	int nVertexCount = 0;
	for ( int &nIndex : remap )
	{
		if ( nIndex == 0 )
			nIndex = nVertexCount++;
	}

	const int nTriangleCount = static_cast<int>( triangles.size() );
	const int nLineCount = ( 3 * nTriangleCount ) / 2;
	const int nIndexCount = 2 * nLineCount;

	std::vector<unsigned char> marked( static_cast<size_t>( nVertexCount ) * nVertexCount, 0 );
	for ( const CPhysConvex::Triangle_t &triangle : triangles )
	{
		for ( int i = 0; i < 3; ++i )
		{
			const int nFirst = remap[triangle.m_nVertices[i]];
			const int nSecond = remap[triangle.m_nVertices[( i + 1 ) % 3]];
			marked[static_cast<size_t>( std::min( nFirst, nSecond ) ) * nVertexCount + std::max( nFirst, nSecond )] = 1;
		}
	}

	const auto nVertices = static_cast<unsigned short>( nVertexCount );
	const auto nLines = static_cast<unsigned short>( nLineCount );
	const auto nIndices = static_cast<unsigned short>( nIndexCount );
	const auto nPolygons = static_cast<unsigned short>( nTriangleCount );
	CPolyhedron *pPolyhedron = bUseTempPolyhedron ? GetTempPolyhedron( nVertices, nLines, nIndices, nPolygons ) : CPolyhedron_AllocByNew::Allocate( nVertices, nLines, nIndices, nPolygons );

	for ( size_t i = 0; i < points.size(); ++i )
	{
		if ( remap[i] >= 0 )
			pPolyhedron->pVertices[remap[i]] = ToSource( points[i] );
	}

	int nLine = 0;
	for ( int nRow = 0; nRow + 1 < nVertexCount && nLine < nLineCount; ++nRow )
	{
		for ( int nColumn = nRow + 1; nColumn < nVertexCount && nLine < nLineCount; ++nColumn )
		{
			if ( marked[static_cast<size_t>( nRow ) * nVertexCount + nColumn] )
				pPolyhedron->pLines[nLine++] = { { static_cast<unsigned short>( nRow ), static_cast<unsigned short>( nColumn ) } };
		}
	}

	std::vector<int> lineStarts( nVertexCount, nLineCount );
	if ( nVertexCount > 0 )
		lineStarts[0] = 0;

	int nSearch = 0;
	for ( int nVertex = 1; nVertex < nVertexCount; ++nVertex )
	{
		for ( int i = nSearch; i < nLineCount; ++i )
		{
			if ( pPolyhedron->pLines[i].iPointIndices[0] == nVertex )
			{
				lineStarts[nVertex] = i;
				nSearch = i + 1;
				break;
			}
		}
	}

	for ( int nTriangle = 0; nTriangle < nTriangleCount; ++nTriangle )
	{
		const CPhysConvex::Triangle_t &triangle = triangles[nTriangle];
		const int indices[3] = { remap[triangle.m_nVertices[0]], remap[triangle.m_nVertices[1]], remap[triangle.m_nVertices[2]] };

		const Vector a = pPolyhedron->pVertices[indices[0]];
		const Vector b = pPolyhedron->pVertices[indices[1]];
		const Vector c = pPolyhedron->pVertices[indices[2]];
		const Vector normal = ( c - a ).Cross( b - a );
		const float flLengthSquared = ( normal.x * normal.x + normal.y * normal.y ) + ( normal.z * normal.z + NORMAL_LENGTH_EPSILON );
		float flInverseLength = _mm_cvtss_f32( _mm_rsqrt_ss( _mm_set_ss( flLengthSquared ) ) );
		flInverseLength = flInverseLength * ( ( 3.0f - ( flInverseLength * flInverseLength ) * flLengthSquared ) * 0.5f );

		Polyhedron_IndexedPolygon_t &polygon = pPolyhedron->pPolygons[nTriangle];
		polygon.iFirstIndex = static_cast<unsigned short>( 3 * nTriangle );
		polygon.iIndexCount = 3;
		polygon.polyNormal = normal * flInverseLength;

		for ( int i = 0; i < 3; ++i )
		{
			const int nIndex = 3 * nTriangle + i;
			if ( nIndex >= nIndexCount )
				break;

			const int nFirst = indices[i];
			const int nSecond = indices[( i + 1 ) % 3];
			const int nLow = std::min( nFirst, nSecond );
			const int nHigh = std::max( nFirst, nSecond );

			int nLineIndex = nLineCount;
			for ( int nCandidate = lineStarts[nLow]; nCandidate < nLineCount; ++nCandidate )
			{
				const Polyhedron_IndexedLine_t &line = pPolyhedron->pLines[nCandidate];
				if ( line.iPointIndices[0] == nLow && line.iPointIndices[1] == nHigh )
				{
					nLineIndex = nCandidate;
					break;
				}
			}

			Polyhedron_IndexedLineReference_t &reference = pPolyhedron->pIndices[nIndex];
			reference.iLineIndex = static_cast<unsigned short>( nLineIndex );
			reference.iEndPointIndex = nFirst <= nSecond ? 1 : 0;
		}
	}

	return pPolyhedron;
}
