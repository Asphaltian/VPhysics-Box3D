if( NOT EXISTS "${SOURCE_SDK_DIR}/src/public/vphysics_interface.h" )
	message( FATAL_ERROR "SOURCE_SDK_DIR does not point at a Source SDK 2013 checkout: ${SOURCE_SDK_DIR}" )
endif()

set( SOURCE_SDK_SRC "${SOURCE_SDK_DIR}/src" )

add_library( source_sdk INTERFACE )

target_include_directories( source_sdk SYSTEM INTERFACE
	"${SOURCE_SDK_SRC}/public"
	"${SOURCE_SDK_SRC}/public/tier0"
	"${SOURCE_SDK_SRC}/public/tier1"
	"${SOURCE_SDK_SRC}/common"
)

target_compile_definitions( source_sdk INTERFACE
	$<$<EQUAL:${CMAKE_SIZEOF_VOID_P},8>:PLATFORM_64BITS>
	DLLNAME=vphysics
	$<$<CONFIG:Debug>:DEBUG;_DEBUG>
	$<$<NOT:$<CONFIG:Debug>>:NDEBUG>
)

if( WIN32 )
	target_compile_definitions( source_sdk INTERFACE
		WIN32 _WIN32 _WINDOWS _USRDLL
		$<IF:$<EQUAL:${CMAKE_SIZEOF_VOID_P},8>,WIN64;_WIN64;COMPILER_MSVC64,COMPILER_MSVC32>
		COMPILER_MSVC
		_DLL_EXT=.dll
		_CRT_SECURE_NO_DEPRECATE _CRT_NONSTDC_NO_DEPRECATE
	)
	target_compile_options( source_sdk INTERFACE
		/Zc:__cplusplus
		/Zc:threadSafeInit-
		/Zc:inline
		/GF
		/Gw
		/fp:precise
	)
elseif( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
	target_compile_definitions( source_sdk INTERFACE
		GNUC POSIX _POSIX LINUX _LINUX
		COMPILER_GCC
		_DLL_EXT=.so
	)
else()
	message( FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}" )
endif()
