//=================================================================================================
//
// CPhysCollide cooking / collision queries.
//
//=================================================================================================

#include "cbase.h"

#include "vbox_collide.h"
#include "vbox_parse.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

Box3DPhysicsCollision Box3DPhysicsCollision::s_PhysicsCollision;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( Box3DPhysicsCollision, IPhysicsCollision, VPHYSICS_COLLISION_INTERFACE_VERSION, Box3DPhysicsCollision::GetInstance() );

//-------------------------------------------------------------------------------------------------
//
// IVP / Havok compact-ledge (.phy) ingestion. The binary format below is copied from Source's
// studiobyteswap; only the leaf-ledge -> convex step targets Box3D. IVP data is already in metres,
// so the only transform is the axis swap Vec3( x, z, -y ) (the one true axis remap in Vox3D).
//

#ifndef MAKEID
#define MAKEID( d, c, b, a )	( ( (int)(a) << 24 ) | ( (int)(b) << 16 ) | ( (int)(c) << 8 ) | ( (int)(d) ) )
#endif

namespace ivp_compat
{
	struct collideheader_t
	{
		int		vphysicsID;
		short	version;
		short	modelType;
	};

	struct compactsurfaceheader_t
	{
		int		surfaceSize;
		Vector	dragAxisAreas;
		int		axisMapSize;
	};

	struct compactsurface_t
	{
		float	mass_center[3];
		float	rotation_inertia[3];
		float	upper_limit_radius;

		unsigned int	max_factor_surface_deviation	: 8;
		int				byte_size						: 24;
		int				offset_ledgetree_root;
		int				dummy[3];
	};

	struct compactledge_t
	{
		int		c_point_offset;

		union
		{
			int	ledgetree_node_offset;
			int	client_data;
		};

		struct
		{
			uint	has_children_flag	: 2;
			int		is_compact_flag		: 2;
			uint	dummy				: 4;
			uint	size_div_16			: 24;
		};

		short	n_triangles;
		short	for_future_use;
	};

	struct compactedge_t
	{
		uint	start_point_index	: 16;
		int		opposite_index		: 15;
		uint	is_virtual			: 1;
	};

	struct compacttriangle_t
	{
		uint	tri_index		: 12;
		uint	pierce_index	: 12;
		uint	material_index	: 7;
		uint	is_virtual		: 1;
		compactedge_t c_three_edges[3];
	};

	struct compactledgenode_t
	{
		int		offset_right_node;
		int		offset_compact_ledge;
		float	center[3];
		float	radius;
		unsigned char box_sizes[3];
		unsigned char free_0;

		const compactledge_t *GetCompactLedge() const	{ return ( compactledge_t * )( ( char * )this + this->offset_compact_ledge ); }
		const compactledgenode_t *GetLeftChild() const	{ return this + 1; }
		const compactledgenode_t *GetRightChild() const	{ return ( compactledgenode_t * )( ( char * )this + this->offset_right_node ); }
		bool IsTerminal() const							{ return this->offset_right_node == 0; }
	};

	static constexpr int	IVP_COMPACT_SURFACE_SUPER_LEGACY	= 0;
	static constexpr int	IVP_COMPACT_SURFACE_ID				= MAKEID( 'I', 'V', 'P', 'S' );
	static constexpr int	IVP_COMPACT_SURFACE_ID_SWAPPED		= MAKEID( 'S', 'P', 'V', 'I' );
	static constexpr int	IVP_COMPACT_MOPP_ID					= MAKEID( 'M', 'O', 'P', 'P' );
	static constexpr int	VPHYSICS_COLLISION_ID				= MAKEID( 'V', 'P', 'H', 'Y' );
	static constexpr short	VPHYSICS_COLLISION_VERSION			= 0x0100;

	enum { COLLIDE_POLY = 0, COLLIDE_MOPP = 1, COLLIDE_BALL = 2, COLLIDE_VIRTUAL = 3 };

	static CPhysConvex *IVPLedgeToConvex( const compactledge_t *pLedge )
	{
		if ( !pLedge->n_triangles )
			return nullptr;

		const char *pVertices = reinterpret_cast< const char * >( pLedge ) + pLedge->c_point_offset;
		const compacttriangle_t *pTriangles = reinterpret_cast< const compacttriangle_t * >( pLedge + 1 );
		const int nVertCount = pLedge->n_triangles * 3;

		CUtlVector< b3Vec3 > verts;
		verts.SetCount( nVertCount );

		for ( int i = 0; i < pLedge->n_triangles; i++ )
		{
			for ( int j = 0; j < 3; j++ )
			{
				static constexpr size_t IVPAlignedVectorSize = 16;
				const int nIndex = pTriangles[ i ].c_three_edges[ j ].start_point_index;
				const float *pVertex = reinterpret_cast< const float * >( pVertices + ( nIndex * IVPAlignedVectorSize ) );
				verts[ ( i * 3 ) + j ] = b3Vec3{ pVertex[ 0 ], pVertex[ 2 ], -pVertex[ 1 ] };
			}
		}

		b3HullData *pHull = b3CreateHull( verts.Base(), nVertCount, nVertCount );
		if ( !pHull )
			return nullptr;

		CPhysConvex *pConvex = new CPhysConvex;
		pConvex->m_pHull = pHull;
		pConvex->m_nGameData = pLedge->client_data;
		return pConvex;
	}

	static void GetAllIVPEdges( const compactledgenode_t *pNode, CUtlVector< const compactledge_t * > &vecOut )
	{
		if ( !pNode )
			return;

		if ( !pNode->IsTerminal() )
		{
			GetAllIVPEdges( pNode->GetRightChild(), vecOut );
			GetAllIVPEdges( pNode->GetLeftChild(), vecOut );
		}
		else
		{
			vecOut.AddToTail( pNode->GetCompactLedge() );
		}
	}

	static CPhysCollide *DeserializeIVP_Poly( const compactsurface_t *pSurface )
	{
		const compactledgenode_t *pFirstLedgeNode = reinterpret_cast< const compactledgenode_t * >(
			reinterpret_cast< const char * >( pSurface ) + pSurface->offset_ledgetree_root );

		CUtlVector< const compactledge_t * > ledges;
		GetAllIVPEdges( pFirstLedgeNode, ledges );

		CPhysCollide *pCollide = new CPhysCollide;
		for ( int i = 0; i < ledges.Count(); i++ )
		{
			CPhysConvex *pConvex = IVPLedgeToConvex( ledges[ i ] );
			if ( pConvex )
				pCollide->m_Convexes.AddToTail( pConvex );
		}
		return pCollide;
	}

	static CPhysCollide *DeserializeIVP_Poly( const collideheader_t *pCollideHeader )
	{
		const compactsurfaceheader_t *pSurfaceHeader = reinterpret_cast< const compactsurfaceheader_t * >( pCollideHeader + 1 );
		const compactsurface_t *pSurface = reinterpret_cast< const compactsurface_t * >( pSurfaceHeader + 1 );
		return DeserializeIVP_Poly( pSurface );
	}
}

//-------------------------------------------------------------------------------------------------

static CPhysConvex *HullToConvex( b3HullData *pHull )
{
	if ( !pHull )
		return nullptr;

	CPhysConvex *pConvex = new CPhysConvex;
	pConvex->m_pHull = pHull;
	return pConvex;
}

CPhysConvex *Box3DPhysicsCollision::ConvexFromVerts( Vector **pVerts, int vertCount )
{
	CUtlVector< b3Vec3 > points;
	points.SetCount( vertCount );
	for ( int i = 0; i < vertCount; i++ )
		points[ i ] = SourceToBox::Distance( *pVerts[ i ] );

	return HullToConvex( b3CreateHull( points.Base(), vertCount, vertCount ) );
}

CPhysConvex *Box3DPhysicsCollision::ConvexFromPlanes( float *pPlanes, int planeCount, float mergeDistance )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

float Box3DPhysicsCollision::ConvexVolume( CPhysConvex *pConvex )
{
	if ( !pConvex || !pConvex->m_pHull )
		return 0.0f;

	// Density of 1 makes the reported mass equal to the volume.
	const b3MassData massData = b3ComputeHullMass( pConvex->m_pHull, 1.0f );
	return BoxToSource::Volume( massData.mass );
}

float Box3DPhysicsCollision::ConvexSurfaceArea( CPhysConvex *pConvex )
{
	Log_Stub( LOG_VBox3D );
	return 0.0f;
}

void Box3DPhysicsCollision::SetConvexGameData( CPhysConvex *pConvex, unsigned int gameData )
{
	if ( pConvex )
		pConvex->m_nGameData = gameData;
}

void Box3DPhysicsCollision::ConvexFree( CPhysConvex *pConvex )
{
	if ( !pConvex )
		return;

	if ( pConvex->m_pHull )
		b3DestroyHull( pConvex->m_pHull );
	delete pConvex;
}

CPhysConvex *Box3DPhysicsCollision::BBoxToConvex( const Vector &mins, const Vector &maxs )
{
	b3Vec3 corners[ 8 ];
	for ( int i = 0; i < 8; i++ )
	{
		const Vector corner(
			( i & 1 ) ? maxs.x : mins.x,
			( i & 2 ) ? maxs.y : mins.y,
			( i & 4 ) ? maxs.z : mins.z );
		corners[ i ] = SourceToBox::Distance( corner );
	}

	return HullToConvex( b3CreateHull( corners, 8, 8 ) );
}

CPhysConvex *Box3DPhysicsCollision::ConvexFromConvexPolyhedron( const CPolyhedron &ConvexPolyhedron )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

void Box3DPhysicsCollision::ConvexesFromConvexPolygon( const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput )
{
	Log_Stub( LOG_VBox3D );
}

//-------------------------------------------------------------------------------------------------

CPhysPolysoup *Box3DPhysicsCollision::PolysoupCreate()
{
	return new CPhysPolysoup;
}

void Box3DPhysicsCollision::PolysoupDestroy( CPhysPolysoup *pSoup )
{
	delete pSoup;
}

void Box3DPhysicsCollision::PolysoupAddTriangle( CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits )
{
	if ( !pSoup )
		return;

	pSoup->m_Vertices.AddToTail( SourceToBox::Distance( a ) );
	pSoup->m_Vertices.AddToTail( SourceToBox::Distance( b ) );
	pSoup->m_Vertices.AddToTail( SourceToBox::Distance( c ) );
	pSoup->m_MaterialIndices.AddToTail( (uint8)materialIndex7bits );
}

CPhysCollide *Box3DPhysicsCollision::ConvertPolysoupToCollide( CPhysPolysoup *pSoup, bool useMOPP )
{
	if ( !pSoup || pSoup->m_Vertices.Count() < 3 )
		return nullptr;

	const int triangleCount = pSoup->m_Vertices.Count() / 3;

	CUtlVector< int32 > indices;
	indices.SetCount( pSoup->m_Vertices.Count() );
	for ( int i = 0; i < indices.Count(); i++ )
		indices[ i ] = i;

	b3MeshDef def = {};
	def.vertices = pSoup->m_Vertices.Base();
	def.vertexCount = pSoup->m_Vertices.Count();
	def.indices = indices.Base();
	def.triangleCount = triangleCount;
	def.materialIndices = pSoup->m_MaterialIndices.Base();
	def.weldVertices = true;
	def.weldTolerance = SourceToBox::Distance( 0.1f );

	b3MeshData *pMesh = b3CreateMesh( &def, nullptr, 0 );
	if ( !pMesh )
		return nullptr;

	CPhysCollide *pCollide = new CPhysCollide;
	pCollide->m_pMesh = pMesh;
	return pCollide;
}

//-------------------------------------------------------------------------------------------------

CPhysCollide *Box3DPhysicsCollision::ConvertConvexToCollide( CPhysConvex **pConvex, int convexCount )
{
	return ConvertConvexToCollideParams( pConvex, convexCount, convertconvexparams_t{} );
}

CPhysCollide *Box3DPhysicsCollision::ConvertConvexToCollideParams( CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams )
{
	CPhysCollide *pCollide = new CPhysCollide;

	b3Vec3 weightedCenter = { 0.0f, 0.0f, 0.0f };
	float totalMass = 0.0f;

	for ( int i = 0; i < convexCount; i++ )
	{
		if ( !pConvex[ i ] )
			continue;

		pCollide->m_Convexes.AddToTail( pConvex[ i ] );

		if ( pConvex[ i ]->m_pHull )
		{
			const b3MassData massData = b3ComputeHullMass( pConvex[ i ]->m_pHull, 1.0f );
			weightedCenter.x += massData.mass * massData.center.x;
			weightedCenter.y += massData.mass * massData.center.y;
			weightedCenter.z += massData.mass * massData.center.z;
			totalMass += massData.mass;
		}
	}

	if ( totalMass > 0.0f )
	{
		const b3Vec3 center = { weightedCenter.x / totalMass, weightedCenter.y / totalMass, weightedCenter.z / totalMass };
		pCollide->m_vecMassCenter = BoxToSource::Distance( center );
	}

	return pCollide;
}

void Box3DPhysicsCollision::DestroyCollide( CPhysCollide *pCollide )
{
	if ( !pCollide )
		return;

	for ( int i = 0; i < pCollide->m_Convexes.Count(); i++ )
	{
		if ( pCollide->m_Convexes[ i ]->m_pHull )
			b3DestroyHull( pCollide->m_Convexes[ i ]->m_pHull );
		delete pCollide->m_Convexes[ i ];
	}

	if ( pCollide->m_pMesh )
		b3DestroyMesh( pCollide->m_pMesh );

	delete pCollide;
}

//-------------------------------------------------------------------------------------------------

int Box3DPhysicsCollision::CollideSize( CPhysCollide *pCollide )
{
	Log_Stub( LOG_VBox3D );
	return 0;
}

int Box3DPhysicsCollision::CollideWrite( char *pDest, CPhysCollide *pCollide, bool bSwap )
{
	Log_Stub( LOG_VBox3D );
	return 0;
}

CPhysCollide *Box3DPhysicsCollision::UnserializeCollide( char *pBuffer, int size, int index )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

float Box3DPhysicsCollision::CollideVolume( CPhysCollide *pCollide )
{
	if ( !pCollide )
		return 0.0f;

	float volume = 0.0f;
	for ( int i = 0; i < pCollide->m_Convexes.Count(); i++ )
		volume += ConvexVolume( pCollide->m_Convexes[ i ] );

	return volume;
}

float Box3DPhysicsCollision::CollideSurfaceArea( CPhysCollide *pCollide )
{
	Log_Stub( LOG_VBox3D );
	return 0.0f;
}

Vector Box3DPhysicsCollision::CollideGetExtent( const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction )
{
	Log_Stub( LOG_VBox3D );
	return collideOrigin;
}

void Box3DPhysicsCollision::CollideGetAABB( Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles )
{
	if ( !pCollide )
	{
		if ( pMins ) *pMins = collideOrigin;
		if ( pMaxs ) *pMaxs = collideOrigin;
		return;
	}

	b3Transform xf;
	xf.p = SourceToBox::Distance( collideOrigin );
	xf.q = SourceToBox::Angle( collideAngles );

	b3AABB bounds = {};
	bool bHasBounds = false;
	for ( int i = 0; i < pCollide->m_Convexes.Count(); i++ )
	{
		if ( !pCollide->m_Convexes[ i ]->m_pHull )
			continue;

		const b3AABB hullBounds = b3ComputeHullAABB( pCollide->m_Convexes[ i ]->m_pHull, xf );
		bounds = bHasBounds ? b3AABB_Union( bounds, hullBounds ) : hullBounds;
		bHasBounds = true;
	}

	if ( pCollide->m_pMesh )
	{
		const b3AABB meshBounds = b3ComputeMeshAABB( pCollide->m_pMesh, xf, b3Vec3{ 1.0f, 1.0f, 1.0f } );
		bounds = bHasBounds ? b3AABB_Union( bounds, meshBounds ) : meshBounds;
		bHasBounds = true;
	}

	if ( !bHasBounds )
	{
		if ( pMins ) *pMins = collideOrigin;
		if ( pMaxs ) *pMaxs = collideOrigin;
		return;
	}

	if ( pMins ) *pMins = BoxToSource::Distance( bounds.lowerBound );
	if ( pMaxs ) *pMaxs = BoxToSource::Distance( bounds.upperBound );
}

void Box3DPhysicsCollision::CollideGetMassCenter( CPhysCollide *pCollide, Vector *pOutMassCenter )
{
	if ( pOutMassCenter )
		*pOutMassCenter = pCollide ? pCollide->m_vecMassCenter : vec3_origin;
}

void Box3DPhysicsCollision::CollideSetMassCenter( CPhysCollide *pCollide, const Vector &massCenter )
{
	if ( pCollide )
		pCollide->m_vecMassCenter = massCenter;
}

Vector Box3DPhysicsCollision::CollideGetOrthographicAreas( const CPhysCollide *pCollide )
{
	return pCollide ? pCollide->m_vecOrthographicAreas : Vector( 1.0f, 1.0f, 1.0f );
}

void Box3DPhysicsCollision::CollideSetOrthographicAreas( CPhysCollide *pCollide, const Vector &areas )
{
	if ( pCollide )
		pCollide->m_vecOrthographicAreas = areas;
}

int Box3DPhysicsCollision::CollideIndex( const CPhysCollide *pCollide )
{
	Log_Stub( LOG_VBox3D );
	return 0;
}

CPhysCollide *Box3DPhysicsCollision::BBoxToCollide( const Vector &mins, const Vector &maxs )
{
	CPhysConvex *pConvex = BBoxToConvex( mins, maxs );
	if ( !pConvex )
		return nullptr;

	return ConvertConvexToCollide( &pConvex, 1 );
}

int Box3DPhysicsCollision::GetConvexesUsedInCollideable( const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit )
{
	if ( !pCollideable )
		return 0;

	const int count = Min( pCollideable->m_Convexes.Count(), iOutputArrayLimit );
	for ( int i = 0; i < count; i++ )
		pOutputArray[ i ] = pCollideable->m_Convexes[ i ];

	return count;
}

//-------------------------------------------------------------------------------------------------

namespace
{
	// Trace a ray (point) or swept box against a collide's convex hulls, in the collide's local frame.
	// Polysoup meshes are not traced.
	void TraceBoxVsCollide( const Ray_t &ray, const CPhysCollide *pCollide,
		const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
	{
		if ( !pTrace )
			return;

		ClearTrace( pTrace );

		const Vector vecStart = ray.m_Start + ray.m_StartOffset;
		pTrace->startpos = vecStart;
		pTrace->endpos = vecStart + ray.m_Delta;

		if ( !pCollide || pCollide->m_Convexes.Count() == 0 )
			return;

		// The collide's world transform, and the ray taken into the collide's local space.
		const b3Transform xf = { SourceToBox::Distance( collideOrigin ), SourceToBox::Angle( collideAngles ) };
		const b3Vec3 localOrigin = b3InvTransformPoint( xf, SourceToBox::Distance( vecStart ) );
		const b3Vec3 localTranslation = b3InvRotateVector( xf.q, SourceToBox::Distance( ray.m_Delta ) );

		const bool bIsPoint = ray.m_Extents.LengthSqr() < 1e-6f;

		// For a swept box, build its 8 corners at the ray origin (local space) as the cast proxy.
		const b3Vec3 he = SourceToBox::Distance( ray.m_Extents );
		b3Vec3 boxPoints[ 8 ];
		if ( !bIsPoint )
		{
			int k = 0;
			for ( int sx = -1; sx <= 1; sx += 2 )
				for ( int sy = -1; sy <= 1; sy += 2 )
					for ( int sz = -1; sz <= 1; sz += 2 )
						boxPoints[ k++ ] = b3Vec3{ localOrigin.x + sx * he.x, localOrigin.y + sy * he.y, localOrigin.z + sz * he.z };
		}

		b3CastOutput best = {};
		best.fraction = 1.0f;
		bool bHit = false;

		for ( int i = 0; i < pCollide->m_Convexes.Count(); i++ )
		{
			const b3HullData *pHull = pCollide->m_Convexes[ i ]->m_pHull;
			if ( !pHull )
				continue;

			b3CastOutput out;
			if ( bIsPoint )
			{
				const b3RayCastInput in = { localOrigin, localTranslation, 1.0f };
				out = b3RayCastHull( pHull, &in );
			}
			else
			{
				b3ShapeCastInput in = {};
				in.proxy.points = boxPoints;
				in.proxy.count = 8;
				in.proxy.radius = 0.0f;
				in.translation = localTranslation;
				in.maxFraction = 1.0f;
				in.canEncroach = false;
				out = b3ShapeCastHull( pHull, &in );
			}

			if ( out.hit && ( !bHit || out.fraction < best.fraction ) )
			{
				best = out;
				bHit = true;
			}
		}

		if ( !bHit )
			return;

		const Vector vecNormal = BoxToSource::Unitless( b3RotateVector( xf.q, best.normal ) );

		pTrace->fraction = best.fraction;
		pTrace->endpos = vecStart + ray.m_Delta * best.fraction;
		pTrace->plane.normal = vecNormal;
		pTrace->plane.dist = DotProduct( pTrace->endpos, vecNormal );
		pTrace->contents = CONTENTS_SOLID;
		pTrace->allsolid = best.fraction == 0.0f;
		pTrace->startsolid = best.fraction == 0.0f;
	}
}

void Box3DPhysicsCollision::TraceBox( const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	Ray_t ray;
	ray.Init( start, end, mins, maxs );
	TraceBoxVsCollide( ray, pCollide, collideOrigin, collideAngles, ptr );
}

void Box3DPhysicsCollision::TraceBox( const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	TraceBoxVsCollide( ray, pCollide, collideOrigin, collideAngles, ptr );
}

void Box3DPhysicsCollision::TraceBox( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	TraceBoxVsCollide( ray, pCollide, collideOrigin, collideAngles, ptr );
}

void Box3DPhysicsCollision::TraceCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	Log_Stub( LOG_VBox3D );
	if ( ptr )
		ClearTrace( ptr );
}

bool Box3DPhysicsCollision::IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone )
{
	Log_Stub( LOG_VBox3D );
	return false;
}

void Box3DPhysicsCollision::VCollideLoad( vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap )
{
	if ( swap )
		return;

	pOutput->solidCount = solidCount;
	pOutput->solids = new CPhysCollide *[ solidCount ];

	const char *pCursor = pBuffer;
	for ( int i = 0; i < solidCount; i++ )
	{
		pOutput->solids[ i ] = nullptr;

		const int solidSize = *reinterpret_cast< const int * >( pCursor );
		pCursor += sizeof( int );

		const ivp_compat::collideheader_t *pCollideHeader = reinterpret_cast< const ivp_compat::collideheader_t * >( pCursor );

		if ( pCollideHeader->vphysicsID == ivp_compat::VPHYSICS_COLLISION_ID )
		{
			if ( pCollideHeader->version != ivp_compat::VPHYSICS_COLLISION_VERSION )
				Log_Warning( LOG_VBox3D, "Solid with unknown version: 0x%x, may crash!\n", pCollideHeader->version );

			if ( pCollideHeader->modelType == ivp_compat::COLLIDE_POLY )
				pOutput->solids[ i ] = ivp_compat::DeserializeIVP_Poly( pCollideHeader );
			else
				Log_Warning( LOG_VBox3D, "Unsupported solid type 0x%x on solid %d. Skipping...\n", (int)pCollideHeader->modelType, i );
		}
		else
		{
			// Legacy .phy: just a dumped compact surface.
			const ivp_compat::compactsurface_t *pCompactSurface = reinterpret_cast< const ivp_compat::compactsurface_t * >( pCursor );
			const int legacyModelType = pCompactSurface->dummy[ 2 ];
			if ( legacyModelType == ivp_compat::IVP_COMPACT_SURFACE_SUPER_LEGACY ||
				 legacyModelType == ivp_compat::IVP_COMPACT_SURFACE_ID ||
				 legacyModelType == ivp_compat::IVP_COMPACT_SURFACE_ID_SWAPPED )
				pOutput->solids[ i ] = ivp_compat::DeserializeIVP_Poly( pCompactSurface );
			else
				Log_Warning( LOG_VBox3D, "Unsupported legacy solid type 0x%x on solid %d. Skipping...\n", legacyModelType, i );
		}

		pCursor += solidSize;
	}

	// The rest of the buffer is the KeyValues text.
	const int keyValuesSize = size - (int)( uintp( pCursor ) - uintp( pBuffer ) );
	pOutput->pKeyValues = new char[ keyValuesSize + 1 ];
	V_memcpy( pOutput->pKeyValues, pCursor, keyValuesSize );
	pOutput->pKeyValues[ keyValuesSize ] = '\0';
	pOutput->descSize = keyValuesSize;
	pOutput->isPacked = false;
#ifdef GAME_ASW_OR_NEWER
	pOutput->pUserData = nullptr;
#endif
}

void Box3DPhysicsCollision::VCollideUnload( vcollide_t *pVCollide )
{
	for ( int i = 0; i < pVCollide->solidCount; i++ )
		DestroyCollide( pVCollide->solids[ i ] );

	delete[] pVCollide->solids;
	delete[] pVCollide->pKeyValues;
	V_memset( pVCollide, 0, sizeof( *pVCollide ) );
}

IVPhysicsKeyParser *Box3DPhysicsCollision::VPhysicsKeyParserCreate( const char *pKeyData )
{
	return CreateVPhysicsKeyParser( pKeyData, false );
}

IVPhysicsKeyParser *Box3DPhysicsCollision::VPhysicsKeyParserCreate( vcollide_t *pVCollide )
{
	// GMod x64's engine calls this overload (not the const char* one) to parse a model's physics
	// keyvalues. Returning nullptr faults the engine when it iterates the parser.
	return CreateVPhysicsKeyParser( pVCollide->pKeyValues ? pVCollide->pKeyValues : "", pVCollide->isPacked );
}

void Box3DPhysicsCollision::VPhysicsKeyParserDestroy( IVPhysicsKeyParser *pParser )
{
	delete pParser;
}

int Box3DPhysicsCollision::CreateDebugMesh( CPhysCollide const *pCollisionModel, Vector **outVerts )
{
	Log_Stub( LOG_VBox3D );
	return 0;
}

void Box3DPhysicsCollision::DestroyDebugMesh( int vertCount, Vector *outVerts )
{
	Log_Stub( LOG_VBox3D );
}

// Empty collision query. The engine virtual-calls the returned object, so it must be non-null.
namespace
{
	class Box3DDummyCollisionQuery final : public ICollisionQuery
	{
	public:
		int ConvexCount() override { return 0; }
		int TriangleCount( int ) override { return 0; }
		unsigned int GetGameData( int ) override { return 0; }
		void GetTriangleVerts( int, int, Vector *verts ) override
		{
			if ( verts ) verts[ 0 ] = verts[ 1 ] = verts[ 2 ] = vec3_origin;
		}
		void SetTriangleVerts( int, int, const Vector * ) override {}
		int GetTriangleMaterialIndex( int, int ) override { return 0; }
		void SetTriangleMaterialIndex( int, int, int ) override {}
	};
}

ICollisionQuery *Box3DPhysicsCollision::CreateQueryModel( CPhysCollide *pCollide )
{
	return new Box3DDummyCollisionQuery;
}

void Box3DPhysicsCollision::DestroyQueryModel( ICollisionQuery *pQuery )
{
	delete static_cast< Box3DDummyCollisionQuery * >( pQuery );
}

IPhysicsCollision *Box3DPhysicsCollision::ThreadContextCreate()
{
	return this;
}

void Box3DPhysicsCollision::ThreadContextDestroy( IPhysicsCollision *pThreadContex )
{
}

CPhysCollide *Box3DPhysicsCollision::CreateVirtualMesh( const virtualmeshparams_t &params )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

bool Box3DPhysicsCollision::SupportsVirtualMesh()
{
	return false;
}

bool Box3DPhysicsCollision::GetBBoxCacheSize( int *pCachedSize, int *pCachedCount )
{
	if ( pCachedSize ) *pCachedSize = 0;
	if ( pCachedCount ) *pCachedCount = 0;
	return false;
}

CPolyhedron *Box3DPhysicsCollision::PolyhedronFromConvex( CPhysConvex * const pConvex, bool bUseTempPolyhedron )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

void Box3DPhysicsCollision::OutputDebugInfo( const CPhysCollide *pCollide )
{
	Log_Stub( LOG_VBox3D );
}

unsigned int Box3DPhysicsCollision::ReadStat( int statID )
{
	return 0;
}

float Box3DPhysicsCollision::CollideGetRadius( const CPhysCollide *pCollide )
{
	Log_Stub( LOG_VBox3D );
	return 0.0f;
}

void *Box3DPhysicsCollision::VCollideAllocUserData( vcollide_t *pVCollide, size_t userDataSize )
{
	Log_Stub( LOG_VBox3D );
	return nullptr;
}

void Box3DPhysicsCollision::VCollideFreeUserData( vcollide_t *pVCollide )
{
	Log_Stub( LOG_VBox3D );
}

void Box3DPhysicsCollision::VCollideCheck( vcollide_t *pVCollide, const char *pName )
{
	Log_Stub( LOG_VBox3D );
}

bool Box3DPhysicsCollision::TraceBoxAA( const Ray_t &ray, const CPhysCollide *pCollide, trace_t *ptr )
{
	Log_Stub( LOG_VBox3D );
	if ( ptr )
		ClearTrace( ptr );
	return false;
}

void Box3DPhysicsCollision::DuplicateAndScale( vcollide_t *pOut, const vcollide_t *pIn, float flScale )
{
	Log_Stub( LOG_VBox3D );
}
