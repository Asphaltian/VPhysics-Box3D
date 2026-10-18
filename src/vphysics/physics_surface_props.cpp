#include "physics_surface_props.h"

#include "parse_file.h"
#include "physics.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <mutex>

CPhysicsSurfaceProps g_PhysicsSurfaceProps;

static char FoldCase( char c )
{
	return ( c >= 'A' && c <= 'Z' ) ? static_cast<char>( c + ( 'a' - 'A' ) ) : c;
}

size_t CPhysicsSurfaceProps::CaseInsensitiveHash::operator()( std::string_view string ) const
{
	size_t nHash = 14695981039346656037ull;
	for ( char c : string )
	{
		nHash ^= static_cast<unsigned char>( FoldCase( c ) );
		nHash *= 1099511628211ull;
	}
	return nHash;
}

bool CPhysicsSurfaceProps::CaseInsensitiveEqual::operator()( std::string_view lhs, std::string_view rhs ) const
{
	return std::ranges::equal( lhs, rhs, []( char a, char b ) { return FoldCase( a ) == FoldCase( b ); } );
}

CPhysicsSurfaceProps::CPhysicsSurfaceProps()
{
	for ( int i = 0; i < MAX_MATERIAL_INDEX; ++i )
		m_WorldMaterialIndices[i] = static_cast<unsigned short>( i );

	AddString( "" );
}

unsigned short CPhysicsSurfaceProps::AddString( std::string_view string )
{
	std::unique_lock lock( m_StringMutex );

	if ( auto it = m_StringIndices.find( string ); it != m_StringIndices.end() )
		return it->second;

	if ( m_Strings.size() >= INVALID_STRING_INDEX )
		return INVALID_STRING_INDEX;

	const auto nIndex = static_cast<unsigned short>( m_Strings.size() );
	const std::string &stored = m_Strings.emplace_back( string );
	m_StringIndices.emplace( stored, nIndex );
	return nIndex;
}

unsigned short CPhysicsSurfaceProps::FindString( std::string_view string ) const
{
	std::shared_lock lock( m_StringMutex );

	auto it = m_StringIndices.find( string );
	return it != m_StringIndices.end() ? it->second : INVALID_STRING_INDEX;
}

int CPhysicsSurfaceProps::ResolveIndex( int surfaceDataIndex ) const
{
	if ( surfaceDataIndex == MATERIAL_INDEX_SHADOW )
		surfaceDataIndex = m_nShadowIndex;
	else if ( surfaceDataIndex >= MAX_MATERIAL_INDEX )
		surfaceDataIndex = 0;

	if ( surfaceDataIndex < 0 || surfaceDataIndex >= static_cast<int>( m_SurfaceProps.size() ) )
		return -1;

	return surfaceDataIndex;
}

const CPhysicsSurfaceProps::SurfaceProp_t *CPhysicsSurfaceProps::GetSurfaceProp( int surfaceDataIndex ) const
{
	const int nIndex = ResolveIndex( surfaceDataIndex );
	return nIndex >= 0 ? &m_SurfaceProps[nIndex] : nullptr;
}

CPhysicsSurfaceProps::SurfaceProp_t *CPhysicsSurfaceProps::GetSurfaceProp( int surfaceDataIndex )
{
	const int nIndex = ResolveIndex( surfaceDataIndex );
	return nIndex >= 0 ? &m_SurfaceProps[nIndex] : nullptr;
}

void CPhysicsSurfaceProps::StoreSurfaceProp( unsigned short nName, const surfacedata_t &data )
{
	const int nIndex = GetSurfaceIndex( GetString( nName ) );

	if ( nIndex == MATERIAL_INDEX_SHADOW )
	{
		if ( SurfaceProp_t *pShadow = GetSurfaceProp( nIndex ) )
			pShadow->m_Data = data;
		return;
	}

	if ( nIndex >= 0 )
	{
		m_SurfaceProps[nIndex].m_Data = data;
		return;
	}

	m_SurfaceIndices.emplace( nName, static_cast<int>( m_SurfaceProps.size() ) );
	m_SurfaceProps.push_back( { nName, data } );
}

int CPhysicsSurfaceProps::ParseSurfaceData( const char *pFilename, const char *pTextfile )
{
	const unsigned short nFilename = AddString( pFilename );
	if ( std::ranges::find( m_ParsedFiles, nFilename ) != m_ParsedFiles.end() )
		return 0;

	m_ParsedFiles.push_back( nFilename );

	char key[1024];
	char value[1024];

	const char *pCursor = pTextfile;
	do
	{
		pCursor = ParseKeyValue( pCursor, key, value, sizeof( key ) );
		if ( std::strcmp( value, "{" ) )
			continue;

		const unsigned short nName = AddString( key );

		surfacedata_t data = {};
		int nBaseIndex = GetSurfaceIndex( key );
		if ( nBaseIndex < 0 )
			nBaseIndex = GetSurfaceIndex( "default" );
		if ( const SurfaceProp_t *pBase = GetSurfaceProp( nBaseIndex ) )
			data = pBase->m_Data;

		do
		{
			pCursor = ParseKeyValue( pCursor, key, value, sizeof( key ) );

			if ( !std::strcmp( key, "}" ) )
			{
				StoreSurfaceProp( nName, data );
				break;
			}

			if ( !std::strcmp( key, "base" ) )
			{
				if ( const SurfaceProp_t *pBase = GetSurfaceProp( GetSurfaceIndex( value ) ) )
					data = pBase->m_Data;
			}
			else if ( !std::strcmp( key, "thickness" ) )
				data.physics.thickness = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "density" ) )
				data.physics.density = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "elasticity" ) )
				data.physics.elasticity = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "friction" ) )
				data.physics.friction = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "maxspeedfactor" ) )
				data.game.maxSpeedFactor = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "jumpfactor" ) )
				data.game.jumpFactor = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "climbable" ) )
				data.game.climbable = static_cast<unsigned char>( std::atoi( value ) );
			else if ( !std::strcmp( key, "audioreflectivity" ) )
				data.audio.reflectivity = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "audiohardnessfactor" ) )
				data.audio.hardnessFactor = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "audiohardminvelocity" ) )
				data.audio.hardVelocityThreshold = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "audioroughnessfactor" ) )
				data.audio.roughnessFactor = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "scraperoughthreshold" ) )
				data.audio.roughThreshold = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "impacthardthreshold" ) )
				data.audio.hardThreshold = static_cast<float>( std::atof( value ) );
			else if ( !std::strcmp( key, "stepleft" ) )
				data.sounds.stepleft = AddString( value );
			else if ( !std::strcmp( key, "stepright" ) )
				data.sounds.stepright = AddString( value );
			else if ( !std::strcmp( key, "impactsoft" ) )
				data.sounds.impactSoft = AddString( value );
			else if ( !std::strcmp( key, "impacthard" ) )
				data.sounds.impactHard = AddString( value );
			else if ( !std::strcmp( key, "scrapesmooth" ) )
				data.sounds.scrapeSmooth = AddString( value );
			else if ( !std::strcmp( key, "scraperough" ) )
				data.sounds.scrapeRough = AddString( value );
			else if ( !std::strcmp( key, "bulletimpact" ) )
				data.sounds.bulletImpact = AddString( value );
			else if ( !std::strcmp( key, "break" ) )
				data.sounds.breakSound = AddString( value );
			else if ( !std::strcmp( key, "strain" ) )
				data.sounds.strainSound = AddString( value );
			else if ( !std::strcmp( key, "rolling" ) )
				data.sounds.rolling = AddString( value );
			else if ( !std::strcmp( key, "gamematerial" ) )
			{
				if ( value[0] && !value[1] && ( value[0] < '0' || value[0] > '9' ) )
					data.game.material = static_cast<unsigned short>( std::toupper( static_cast<unsigned char>( value[0] ) ) );
				else
					data.game.material = static_cast<unsigned short>( std::atoi( value ) );
			}
			else if ( !std::strcmp( key, "dampening" ) )
				data.physics.dampening = static_cast<float>( std::atof( value ) );
		} while ( pCursor );
	} while ( pCursor );

	if ( !m_bInitialized )
	{
		m_bInitialized = true;

		surfacedata_t shadow = {};
		if ( const SurfaceProp_t *pDefault = GetSurfaceProp( GetSurfaceIndex( "default" ) ) )
			shadow = pDefault->m_Data;

		shadow.physics.friction = 0.8f;
		shadow.physics.elasticity = 0.001f;

		m_nShadowIndex = static_cast<int>( m_SurfaceProps.size() );
		m_SurfaceProps.push_back( { AddString( "$MATERIAL_INDEX_SHADOW" ), shadow } );
	}

	return SurfacePropCount();
}

int CPhysicsSurfaceProps::SurfacePropCount() const
{
	return static_cast<int>( m_SurfaceProps.size() );
}

int CPhysicsSurfaceProps::GetSurfaceIndex( const char *pSurfacePropName ) const
{
	if ( pSurfacePropName[0] == '$' && CaseInsensitiveEqual()( pSurfacePropName, "$MATERIAL_INDEX_SHADOW" ) )
		return MATERIAL_INDEX_SHADOW;

	const unsigned short nName = FindString( pSurfacePropName );
	if ( nName == INVALID_STRING_INDEX )
		return -1;

	auto it = m_SurfaceIndices.find( nName );
	return it != m_SurfaceIndices.end() ? it->second : -1;
}

void CPhysicsSurfaceProps::GetPhysicsProperties( int surfaceDataIndex, float *density, float *thickness, float *friction, float *elasticity ) const
{
	const SurfaceProp_t *pSurfaceProp = GetSurfaceProp( surfaceDataIndex );
	if ( !pSurfaceProp )
		pSurfaceProp = GetSurfaceProp( GetSurfaceIndex( "default" ) );
	if ( !pSurfaceProp )
		return;

	const surfacephysicsparams_t &physics = pSurfaceProp->m_Data.physics;
	if ( friction )
		*friction = physics.friction;
	if ( elasticity )
		*elasticity = physics.elasticity;
	if ( density )
		*density = physics.density;
	if ( thickness )
		*thickness = physics.thickness;
}

surfacedata_t *CPhysicsSurfaceProps::GetSurfaceData( int surfaceDataIndex )
{
	if ( SurfaceProp_t *pSurfaceProp = GetSurfaceProp( surfaceDataIndex ) )
		return &pSurfaceProp->m_Data;

	return m_SurfaceProps.empty() ? nullptr : &m_SurfaceProps.front().m_Data;
}

const char *CPhysicsSurfaceProps::GetString( unsigned short stringTableIndex ) const
{
	std::shared_lock lock( m_StringMutex );

	return stringTableIndex < m_Strings.size() ? m_Strings[stringTableIndex].c_str() : "";
}

const char *CPhysicsSurfaceProps::GetPropName( int surfaceDataIndex ) const
{
	const SurfaceProp_t *pSurfaceProp = GetSurfaceProp( surfaceDataIndex );
	return pSurfaceProp ? GetString( pSurfaceProp->m_nName ) : nullptr;
}

void CPhysicsSurfaceProps::SetWorldMaterialIndexTable( int *pMapArray, int mapSize )
{
	const int nCount = std::min( mapSize, MAX_MATERIAL_INDEX );
	for ( int i = 0; i < nCount; ++i )
		m_WorldMaterialIndices[i] = static_cast<unsigned short>( pMapArray[i] );

	g_Physics.UpdateWorldMaterials();
}

int CPhysicsSurfaceProps::GetWorldMaterialIndex( int nMaterial ) const
{
	const int nIndex = nMaterial >= 0 && nMaterial < MAX_MATERIAL_INDEX ? m_WorldMaterialIndices[nMaterial] : nMaterial;
	return GetSurfaceProp( nIndex ) ? nIndex : GetSurfaceIndex( "default" );
}

void CPhysicsSurfaceProps::GetPhysicsParameters( int surfaceDataIndex, surfacephysicsparams_t *pParamsOut ) const
{
	if ( !pParamsOut )
		return;

	if ( const SurfaceProp_t *pSurfaceProp = GetSurfaceProp( surfaceDataIndex ) )
		*pParamsOut = pSurfaceProp->m_Data.physics;
}
