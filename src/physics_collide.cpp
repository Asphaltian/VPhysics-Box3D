#include "physics_collide.h"

#include "conversion.h"

#include "box3d/constants.h"

#include <algorithm>
#include <cmath>

static b3HullData *CreateHullWithVertexLimit( const b3Vec3 *pPoints, int nPointCount )
{
	int nMaxVertexCount = std::min( nPointCount, B3_MAX_HULL_VERTICES );
	b3HullData *pHull = b3CreateHull( pPoints, nPointCount, nMaxVertexCount );

	while ( !pHull && nMaxVertexCount > 4 )
	{
		nMaxVertexCount = std::max( nMaxVertexCount * 3 / 4, 4 );
		pHull = b3CreateHull( pPoints, nPointCount, nMaxVertexCount );
	}

	return pHull;
}

static b3Vec3 ComputePlanarNormal( const b3Vec3 *pPoints, int nPointCount )
{
	if ( nPointCount < 3 )
		return b3Vec3_zero;

	const b3Vec3 origin = pPoints[0];

	int nFarthest = 0;
	float flFarthestDistance = 0.0f;
	for ( int i = 1; i < nPointCount; ++i )
	{
		const float flDistance = b3LengthSquared( b3Sub( pPoints[i], origin ) );
		if ( flDistance > flFarthestDistance )
		{
			flFarthestDistance = flDistance;
			nFarthest = i;
		}
	}

	const b3Vec3 axis = b3Sub( pPoints[nFarthest], origin );

	b3Vec3 normal = b3Vec3_zero;
	float flLargestArea = 0.0f;
	for ( int i = 1; i < nPointCount; ++i )
	{
		const b3Vec3 cross = b3Cross( axis, b3Sub( pPoints[i], origin ) );
		const float flArea = b3LengthSquared( cross );
		if ( flArea > flLargestArea )
		{
			flLargestArea = flArea;
			normal = cross;
		}
	}

	return flLargestArea > 0.0f ? b3Normalize( normal ) : b3Vec3_zero;
}

b3HullData *CPhysConvex::CreateHull( const b3Vec3 *pPoints, int nPointCount )
{
	if ( b3HullData *pHull = CreateHullWithVertexLimit( pPoints, nPointCount ) )
		return pHull;

	const b3Vec3 normal = ComputePlanarNormal( pPoints, nPointCount );
	if ( b3LengthSquared( normal ) == 0.0f )
		return nullptr;

	const b3Vec3 offset = b3MulSV( B3_LINEAR_SLOP, normal );

	std::vector<b3Vec3> slab;
	slab.reserve( nPointCount * 2 );
	for ( int i = 0; i < nPointCount; ++i )
	{
		slab.push_back( b3Add( pPoints[i], offset ) );
		slab.push_back( b3Sub( pPoints[i], offset ) );
	}

	return CreateHullWithVertexLimit( slab.data(), static_cast<int>( slab.size() ) );
}

static b3HullData *CreateHullFromVertices( const std::vector<Vector> &vertices )
{
	std::vector<b3Vec3> points;
	points.reserve( vertices.size() );
	for ( const Vector &vertex : vertices )
		points.push_back( ToBox3D( vertex ) );

	return CPhysConvex::CreateHull( points.data(), static_cast<int>( points.size() ) );
}

CPhysConvex::CPhysConvex( std::vector<Vector> vertices, std::vector<Triangle_t> triangles )
	: m_Vertices( std::move( vertices ) )
	, m_Triangles( std::move( triangles ) )
	, m_pHull( CreateHullFromVertices( m_Vertices ) )
{
}

CPhysConvex::CPhysConvex( b3HullData *pHull )
	: m_pHull( pHull )
{
	const b3Vec3 *pPoints = b3GetHullPoints( pHull );
	const b3HullHalfEdge *pEdges = b3GetHullEdges( pHull );
	const b3HullFace *pFaces = b3GetHullFaces( pHull );

	m_Vertices.reserve( pHull->vertexCount );
	for ( int i = 0; i < pHull->vertexCount; ++i )
		m_Vertices.push_back( ToSource( pPoints[i] ) );

	for ( int i = 0; i < pHull->faceCount; ++i )
	{
		const b3HullHalfEdge &first = pEdges[pFaces[i].edge];
		const b3HullHalfEdge *pEdge = &pEdges[first.next];
		while ( pEdges[pEdge->next].origin != first.origin )
		{
			const b3HullHalfEdge &next = pEdges[pEdge->next];
			m_Triangles.push_back( { { first.origin, pEdge->origin, next.origin }, 0 } );
			pEdge = &next;
		}
	}
}

CPhysConvex::~CPhysConvex()
{
	if ( m_pHull )
		b3DestroyHull( m_pHull );
}

float CPhysConvex::GetVolume() const
{
	if ( m_Triangles.empty() )
		return 0.0f;

	const Vector &origin = m_Vertices[m_Triangles.front().m_nVertices[0]];

	float flVolume = 0.0f;
	for ( const Triangle_t &triangle : m_Triangles )
	{
		const Vector a = m_Vertices[triangle.m_nVertices[0]] - origin;
		const Vector b = m_Vertices[triangle.m_nVertices[1]] - origin;
		const Vector c = m_Vertices[triangle.m_nVertices[2]] - origin;
		flVolume += std::fabs( a.Dot( b.Cross( c ) ) ) * ( 1.0f / 6.0f );
	}
	return flVolume;
}

float CPhysConvex::GetSurfaceArea() const
{
	float flArea = 0.0f;
	for ( const Triangle_t &triangle : m_Triangles )
	{
		const Vector &a = m_Vertices[triangle.m_nVertices[0]];
		const Vector ab = m_Vertices[triangle.m_nVertices[1]] - a;
		const Vector ac = m_Vertices[triangle.m_nVertices[2]] - a;
		flArea += ab.Cross( ac ).Length() * 0.5f;
	}
	return flArea;
}

static b3CompoundData *CreateCompound( const std::vector<std::unique_ptr<CPhysConvex>> &convexes )
{
	std::vector<b3CompoundHullDef> hulls;
	hulls.reserve( convexes.size() );

	for ( const std::unique_ptr<CPhysConvex> &pConvex : convexes )
	{
		if ( !pConvex->GetHull() )
			continue;

		b3CompoundHullDef def = {};
		def.hull = pConvex->GetHull();
		def.transform = b3Transform_identity;
		def.material = b3DefaultSurfaceMaterial();
		hulls.push_back( def );
	}

	if ( hulls.empty() )
		return nullptr;

	b3CompoundDef def = {};
	def.hulls = hulls.data();
	def.hullCount = static_cast<int>( hulls.size() );
	return b3CreateCompound( &def );
}

CPhysCollide::CPhysCollide( std::vector<std::unique_ptr<CPhysConvex>> convexes )
	: m_Convexes( std::move( convexes ) )
	, m_pCompound( CreateCompound( m_Convexes ) )
{
}

CPhysCollide::~CPhysCollide()
{
	if ( m_pCompound )
		b3DestroyCompound( m_pCompound );
}

float CPhysCollide::GetVolume() const
{
	float flVolume = 0.0f;
	for ( const std::unique_ptr<CPhysConvex> &pConvex : m_Convexes )
		flVolume += pConvex->GetVolume();
	return flVolume;
}

float CPhysCollide::GetSurfaceArea() const
{
	float flArea = 0.0f;
	for ( const std::unique_ptr<CPhysConvex> &pConvex : m_Convexes )
		flArea += pConvex->GetSurfaceArea();
	return flArea;
}

void CPhysCollide::GetAABB( Vector *pMins, Vector *pMaxs, const b3Transform &transform ) const
{
	b3Vec3 mins = { FLT_MAX, FLT_MAX, FLT_MAX };
	b3Vec3 maxs = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

	for ( const std::unique_ptr<CPhysConvex> &pConvex : m_Convexes )
	{
		for ( const Vector &vertex : pConvex->GetVertices() )
		{
			const b3Vec3 point = b3TransformPoint( transform, ToBox3D( vertex ) );
			mins = b3Min( mins, point );
			maxs = b3Max( maxs, point );
		}
	}

	*pMins = ToSource( mins );
	*pMaxs = ToSource( maxs );
}

Vector CPhysCollide::GetExtent( const b3Transform &transform, const Vector &direction ) const
{
	Vector extent = Vector( 0.0f, 0.0f, 0.0f );
	float flBestDistance = -1e6f;

	for ( const std::unique_ptr<CPhysConvex> &pConvex : m_Convexes )
	{
		for ( const Vector &vertex : pConvex->GetVertices() )
		{
			const Vector point = ToSource( b3TransformPoint( transform, ToBox3D( vertex ) ) );
			const float flDistance = point.Dot( direction );
			if ( flDistance > flBestDistance )
			{
				flBestDistance = flDistance;
				extent = point;
			}
		}
	}

	return extent;
}
