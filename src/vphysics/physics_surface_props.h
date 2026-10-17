#pragma once

#include "vphysics_interface.h"
#include "tier0/valve_minmax_off.h"

#include <array>
#include <deque>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

class CPhysicsSurfaceProps final : public IPhysicsSurfaceProps
{
public:
	CPhysicsSurfaceProps();

	int ParseSurfaceData( const char *pFilename, const char *pTextfile ) override;
	int SurfacePropCount() const override;
	int GetSurfaceIndex( const char *pSurfacePropName ) const override;
	void GetPhysicsProperties( int surfaceDataIndex, float *density, float *thickness, float *friction, float *elasticity ) const override;
	surfacedata_t *GetSurfaceData( int surfaceDataIndex ) override;
	const char *GetString( unsigned short stringTableIndex ) const override;
	const char *GetPropName( int surfaceDataIndex ) const override;
	void SetWorldMaterialIndexTable( int *pMapArray, int mapSize ) override;
	int GetWorldMaterialIndex( int nMaterial ) const;
	void GetPhysicsParameters( int surfaceDataIndex, surfacephysicsparams_t *pParamsOut ) const override;

	static constexpr int MATERIAL_INDEX_SHADOW = 0xF000;

private:
	static constexpr int MAX_MATERIAL_INDEX = 128;
	static constexpr unsigned short INVALID_STRING_INDEX = 0xFFFF;

	struct SurfaceProp_t
	{
		unsigned short m_nName;
		surfacedata_t m_Data;
	};

	struct CaseInsensitiveHash
	{
		using is_transparent = void;
		size_t operator()( std::string_view string ) const;
	};

	struct CaseInsensitiveEqual
	{
		using is_transparent = void;
		bool operator()( std::string_view lhs, std::string_view rhs ) const;
	};

	unsigned short AddString( std::string_view string );
	unsigned short FindString( std::string_view string ) const;

	int ResolveIndex( int surfaceDataIndex ) const;
	const SurfaceProp_t *GetSurfaceProp( int surfaceDataIndex ) const;
	SurfaceProp_t *GetSurfaceProp( int surfaceDataIndex );
	void StoreSurfaceProp( unsigned short nName, const surfacedata_t &data );

	mutable std::shared_mutex m_StringMutex;
	std::deque<std::string> m_Strings;
	std::unordered_map<std::string_view, unsigned short, CaseInsensitiveHash, CaseInsensitiveEqual> m_StringIndices;

	std::deque<SurfaceProp_t> m_SurfaceProps;
	std::unordered_map<unsigned short, int> m_SurfaceIndices;
	std::vector<unsigned short> m_ParsedFiles;
	std::array<unsigned short, MAX_MATERIAL_INDEX> m_WorldMaterialIndices;
	int m_nShadowIndex = 0;
	bool m_bInitialized = false;
};

extern CPhysicsSurfaceProps g_PhysicsSurfaceProps;
