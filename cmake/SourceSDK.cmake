if( NOT EXISTS "${SOURCE_SDK_DIR}/src/public/vphysics_interface.h" )
	message( FATAL_ERROR "SOURCE_SDK_DIR does not point at a Source SDK 2013 checkout: ${SOURCE_SDK_DIR}" )
endif()

if( NOT CMAKE_SIZEOF_VOID_P EQUAL 8 )
	message( FATAL_ERROR "Only 64-bit targets are supported by the Source SDK 2013 libraries" )
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
	PLATFORM_64BITS
	DLLNAME=vphysics
	$<$<CONFIG:Debug>:DEBUG;_DEBUG>
	$<$<NOT:$<CONFIG:Debug>>:NDEBUG>
)

if( WIN32 )
	target_compile_definitions( source_sdk INTERFACE
		WIN32 _WIN32 WIN64 _WIN64 _WINDOWS _USRDLL
		COMPILER_MSVC COMPILER_MSVC64
		_DLL_EXT=.dll
		_CRT_SECURE_NO_DEPRECATE _CRT_NONSTDC_NO_DEPRECATE
	)
	target_compile_options( source_sdk INTERFACE
		/Zc:__cplusplus
		/Zc:threadSafeInit-
		/Zc:inline
		/GF
		/Gw
		/fp:fast
	)
	target_link_libraries( source_sdk INTERFACE "${SOURCE_SDK_SRC}/lib/public/x64/tier0.lib" )
elseif( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
	target_compile_definitions( source_sdk INTERFACE
		GNUC POSIX _POSIX LINUX _LINUX
		COMPILER_GCC
		_DLL_EXT=.so
	)
	target_link_libraries( source_sdk INTERFACE "${SOURCE_SDK_SRC}/lib/public/linux64/libtier0.so" )
else()
	message( FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}" )
endif()
