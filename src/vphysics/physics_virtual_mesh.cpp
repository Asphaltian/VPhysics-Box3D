#include "physics_virtual_mesh.h"

#include "compact_builder/compact_surface.h"
#include "compact_builder/surface_builder_ledge_soup.h"
#include "conversion.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>

static constexpr int MAX_PACKED_HULL_INDICES = 0x200;
static constexpr int MAX_PACKED_HULL_VERTEX_SPAN = 0x100;
static constexpr uint32_t VIRTUAL_FLAG = 0x80000000u;

static std::vector<unsigned char> BuildHullSurface( const virtualmeshlist_t &list, int nFirstIndex, int nIndexCount )
{
	std::vector<std::vector<unsigned char>> ledges;
	for ( int nTriangle = nFirstIndex / 3; nTriangle < ( nFirstIndex + nIndexCount ) / 3; ++nTriangle )
	{
		const unsigned short *pIndices = &list.indices[nTriangle * 3];
		std::vector<unsigned char> ledge = WriteTriangleLedge( ToDouble( SourceToIVP( list.pVerts[pIndices[0]] ) ), ToDouble( SourceToIVP( list.pVerts[pIndices[1]] ) ), ToDouble( SourceToIVP( list.pVerts[pIndices[2]] ) ) );
		if ( !ledge.empty() )
			ledges.push_back( std::move( ledge ) );
	}

	if ( ledges.empty() )
		return {};

	std::vector<const std::vector<unsigned char> *> ledgePointers;
	for ( const std::vector<unsigned char> &ledge : ledges )
		ledgePointers.push_back( &ledge );

	CompactSurfaceParams_t params;
	params.m_bBuildOuterHull = true;
	return BuildCompactSurface( ledgePointers, params );
}

static const unsigned char *GetRootLedge( const std::vector<unsigned char> &surface )
{
	if ( surface.empty() )
		return nullptr;

	const int32_t nRoot = Load<int32_t>( surface.data() + SURFACE_LEDGETREE_ROOT_OFFSET );
	const int32_t nLedgeOffset = Load<int32_t>( surface.data() + nRoot + 4 );
	return nLedgeOffset ? surface.data() + nRoot + nLedgeOffset : nullptr;
}

class CHullVertexMap
{
public:
	CHullVertexMap( const unsigned char *pLedge, const virtualmeshlist_t &list )
	: m_nMinimum( list.vertexCount )
	{
		for ( int nTriangle = 0; nTriangle < GetLedgeTriangleCount( pLedge ); ++nTriangle )
		{
			const unsigned char *pTriangle = GetLedgeTriangle( pLedge, nTriangle );
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
			{
				const uint32_t nPoint = GetEdgeStartPoint( pTriangle, nEdge );
				if ( m_Vertices.contains( nPoint ) )
					continue;

				const int nVertex = FindNearestVertex( IVPToSource( LoadPoint( GetLedgePoint( pLedge, nPoint ) ) ), list );
				m_Vertices[nPoint] = nVertex;
				m_nMinimum = std::min( m_nMinimum, nVertex );
				m_nMaximum = std::max( m_nMaximum, nVertex );
			}
		}
	}

	int GetVertex( uint32_t nPoint ) const { return m_Vertices.at( nPoint ); }

	int GetMinimum() const { return m_nMinimum; }

	int GetMaximum() const { return m_nMaximum; }

private:
	static int FindNearestVertex( b3Vec3 point, const virtualmeshlist_t &list )
	{
		int nBest = 0;
		float flBest = 1e24f;
		for ( int i = 0; i < list.vertexCount; ++i )
		{
			const float flDistance = b3Length( b3Sub( point, ToBox3D( list.pVerts[i] ) ) );
			if ( flDistance < flBest )
			{
				flBest = flDistance;
				nBest = i;
			}
		}
		return nBest;
	}

	std::map<uint32_t, int> m_Vertices;
	int m_nMinimum;
	int m_nMaximum = 0;
};

static bool IsPackable( const unsigned char *pLedge, const virtualmeshlist_t &list )
{
	if ( GetLedgeTriangleCount( pLedge ) * 3 > MAX_PACKED_HULL_INDICES )
		return false;

	const CHullVertexMap vertices( pLedge, list );
	return vertices.GetMaximum() - vertices.GetMinimum() < MAX_PACKED_HULL_VERTEX_SPAN;
}

struct PackedHullLedge_t
{
	unsigned char m_Header[5];
	std::vector<unsigned char> m_Payload;
};

static PackedHullLedge_t PackHullLedge( const unsigned char *pLedge, const virtualmeshlist_t &list )
{
	const int nTriangleCount = GetLedgeTriangleCount( pLedge );
	const CHullVertexMap vertices( pLedge, list );
	const int nBase = vertices.GetMinimum();

	auto isVirtual = []( uint32_t nWord ) { return ( nWord & VIRTUAL_FLAG ) != 0; };
	auto getWord = [&]( int nTriangle, int nWord ) { return Load<uint32_t>( GetLedgeTriangle( pLedge, nTriangle ) + nWord * 4 ); };

	std::vector<int> order;
	for ( int nTriangle = 0; nTriangle < nTriangleCount; ++nTriangle )
	{
		if ( isVirtual( getWord( nTriangle, 0 ) ) )
			order.push_back( nTriangle );
	}
	const int nVirtualTriangles = static_cast<int>( order.size() );
	for ( int nTriangle = 0; nTriangle < nTriangleCount; ++nTriangle )
	{
		if ( !isVirtual( getWord( nTriangle, 0 ) ) )
			order.push_back( nTriangle );
	}

	std::vector<int> newIndices( nTriangleCount );
	for ( int i = 0; i < nTriangleCount; ++i )
		newIndices[order[i]] = i;

	const int nSlotCount = 4 * nTriangleCount + 4;
	std::vector<int> slotIds( nSlotCount, -1 );
	std::vector<int> owners;

	auto getOppositeSlot = [&]( int nTriangle, int nEdge ) {
		const uint32_t nOpposite = ( getWord( nTriangle, nEdge + 1 ) >> 16 ) & 0x7FFF;
		const int nOffset = nOpposite & 0x4000 ? static_cast<int>( nOpposite ) - 0x8000 : static_cast<int>( nOpposite );
		const int nSlot = 4 * nTriangle + 1 + nEdge + nOffset;
		return nSlot >= 0 && nSlot < nSlotCount ? nSlot : -1;
	};

	auto assign = [&]( bool bVirtualPass ) {
		for ( int nTriangle : order )
		{
			for ( int nEdge = 0; nEdge < 3; ++nEdge )
			{
				const int nSlot = 4 * nTriangle + 1 + nEdge;
				const int nOpposite = getOppositeSlot( nTriangle, nEdge );
				const bool bOppositeFree = nOpposite < 0 || slotIds[nOpposite] < 0;
				if ( isVirtual( getWord( nTriangle, nEdge + 1 ) ) == bVirtualPass && bOppositeFree )
				{
					slotIds[nSlot] = static_cast<int>( owners.size() );
					owners.push_back( nSlot );
				}

				if ( !bVirtualPass && slotIds[nSlot] < 0 && nOpposite >= 0 )
					slotIds[nSlot] = slotIds[nOpposite];
			}
		}
	};

	assign( true );
	const int nVirtualEdges = static_cast<int>( owners.size() );
	assign( false );

	const auto nTriangleByte = static_cast<unsigned char>( nTriangleCount );
	const int nEdgeCount = ( 3 * nTriangleByte ) / 2;

	PackedHullLedge_t packed;
	packed.m_Header[0] = nTriangleByte;
	packed.m_Header[1] = static_cast<unsigned char>( nVirtualTriangles );
	packed.m_Header[2] = static_cast<unsigned char>( nEdgeCount );
	packed.m_Header[3] = static_cast<unsigned char>( nVirtualEdges );
	packed.m_Header[4] = static_cast<unsigned char>( nBase );

	for ( int nTriangle : order )
	{
		for ( int nEdge = 0; nEdge < 3; ++nEdge )
			packed.m_Payload.push_back( static_cast<unsigned char>( slotIds[4 * nTriangle + 1 + nEdge] ) );
		packed.m_Payload.push_back( static_cast<unsigned char>( newIndices[( getWord( nTriangle, 0 ) >> 12 ) & 0xFFF] ) );
	}

	for ( int nId = 0; nId < nEdgeCount; ++nId )
	{
		if ( nId >= static_cast<int>( owners.size() ) )
		{
			packed.m_Payload.push_back( 0 );
			packed.m_Payload.push_back( 0 );
			continue;
		}

		const int nSlot = owners[nId];
		const int nTriangle = ( nSlot - 1 ) / 4;
		const int nEdge = ( nSlot - 1 ) % 4;
		const unsigned char *pTriangle = GetLedgeTriangle( pLedge, nTriangle );
		packed.m_Payload.push_back( static_cast<unsigned char>( vertices.GetVertex( GetEdgeStartPoint( pTriangle, nEdge ) ) - nBase ) );
		packed.m_Payload.push_back( static_cast<unsigned char>( vertices.GetVertex( GetEdgeStartPoint( pTriangle, ( nEdge + 1 ) % 3 ) ) - nBase ) );
	}

	return packed;
}

static std::vector<unsigned char> PackHull( const std::vector<const unsigned char *> &ledges, const virtualmeshlist_t &list )
{
	std::vector<PackedHullLedge_t> packed;
	for ( const unsigned char *pLedge : ledges )
		packed.push_back( PackHullLedge( pLedge, list ) );

	std::vector<unsigned char> blob( 4, 0 );
	blob[0] = static_cast<unsigned char>( packed.size() );
	for ( const PackedHullLedge_t &ledge : packed )
		blob.insert( blob.end(), std::begin( ledge.m_Header ), std::end( ledge.m_Header ) );
	for ( const PackedHullLedge_t &ledge : packed )
		blob.insert( blob.end(), ledge.m_Payload.begin(), ledge.m_Payload.end() );
	return blob;
}

CPhysCollideVirtualMesh::CPhysCollideVirtualMesh( const virtualmeshparams_t &params )
: m_pHandler( params.pMeshEventHandler )
, m_pUserData( params.userData )
{
	if ( !params.buildOuterHull )
		return;

	auto pList = std::make_unique<virtualmeshlist_t>();
	m_pHandler->GetVirtualMesh( m_pUserData, pList.get() );
	if ( pList->triangleCount == 0 )
		return;

	const std::vector<unsigned char> surface = BuildHullSurface( *pList, 0, pList->indexCount );
	const unsigned char *pRoot = GetRootLedge( surface );
	if ( !pRoot )
		return;

	if ( IsPackable( pRoot, *pList ) )
	{
		m_PackedHull = PackHull( { pRoot }, *pList );
		return;
	}

	const int nHalf = pList->indexCount / 2;
	const std::vector<unsigned char> lowSurface = BuildHullSurface( *pList, 0, nHalf );
	const std::vector<unsigned char> highSurface = BuildHullSurface( *pList, nHalf, nHalf );
	const unsigned char *pLowRoot = GetRootLedge( lowSurface );
	const unsigned char *pHighRoot = GetRootLedge( highSurface );
	if ( pLowRoot && pHighRoot )
		m_PackedHull = PackHull( { pLowRoot, pHighRoot }, *pList );
}

const std::vector<std::unique_ptr<CPhysConvex>> &CPhysCollideVirtualMesh::GetConvexes() const
{
	if ( m_bConvexesBuilt.load( std::memory_order_acquire ) )
		return m_Convexes;

	static std::mutex s_BuildMutex;
	std::lock_guard lock( s_BuildMutex );
	if ( m_bConvexesBuilt.load( std::memory_order_relaxed ) )
		return m_Convexes;

	auto pList = std::make_unique<virtualmeshlist_t>();
	m_pHandler->GetVirtualMesh( m_pUserData, pList.get() );

	m_Convexes.reserve( pList->triangleCount );
	for ( int nTriangle = 0; nTriangle < pList->triangleCount; ++nTriangle )
	{
		const unsigned short *pIndices = &pList->indices[nTriangle * 3];
		const std::vector<DoubleVec3_t> points = { ToDouble( SourceToIVP( pList->pVerts[pIndices[0]] ) ), ToDouble( SourceToIVP( pList->pVerts[pIndices[1]] ) ), ToDouble( SourceToIVP( pList->pVerts[pIndices[2]] ) ) };
		if ( CPhysConvex *pConvex = CreateConvexFromLedge( WriteCompactLedge( points, { { 0, 1, 2 }, { 0, 2, 1 } } ) ) )
			m_Convexes.emplace_back( pConvex );
	}
	m_bConvexesBuilt.store( true, std::memory_order_release );
	return m_Convexes;
}

void CPhysCollideVirtualMesh::Serialize( char *pDest, bool ) const
{
	std::memcpy( pDest, m_PackedHull.data(), m_PackedHull.size() );
}

Vector CPhysCollideVirtualMesh::GetMassCenter() const
{
	Vector mins, maxs;
	m_pHandler->GetWorldspaceBounds( m_pUserData, &mins, &maxs );
	return ( mins + maxs ) * 0.5f;
}

void CPhysCollideVirtualMesh::SetTriangleMaterial( int nConvex, int nTriangle, int nMaterial )
{
	GetConvexes()[nConvex]->GetTriangles()[nTriangle].m_nMaterialIndex = static_cast<unsigned char>( nMaterial & 0x7F );
}

void CPhysCollideVirtualMesh::OutputDebugInfo() const
{
	Msg( "Virtual mesh!\n" );
}
