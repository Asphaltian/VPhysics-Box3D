cmake_minimum_required( VERSION 3.25 )

project( vphysics LANGUAGES C CXX )

set( CMAKE_CXX_STANDARD 20 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )
set( CMAKE_C_STANDARD 17 )
set( CMAKE_C_STANDARD_REQUIRED ON )
set( CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>" )
set( CMAKE_POSITION_INDEPENDENT_CODE ON )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

set( SOURCE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/source-sdk-2013" CACHE PATH "Source SDK 2013 checkout" )

option( VPHYSICS_DEDICATED "Build the Linux dedicated server module" OFF )

if( VPHYSICS_DEDICATED AND NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" )
	message( FATAL_ERROR "VPHYSICS_DEDICATED only applies to Linux, Windows dedicated servers load the same vphysics.dll as the game" )
endif()

include( cmake/Box3D.cmake )
include( cmake/SourceSDK.cmake )
include( cmake/Qhull.cmake )

add_subdirectory( src )
