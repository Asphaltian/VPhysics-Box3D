set( VPHYSICS_GAME "sdk2013" CACHE STRING "Game the module is built for" )
set_property( CACHE VPHYSICS_GAME PROPERTY STRINGS sdk2013 gmod )

add_library( vphysics_game INTERFACE )

if( VPHYSICS_GAME STREQUAL "gmod" )
	target_compile_definitions( vphysics_game INTERFACE VPHYSICS_GMOD )
elseif( NOT VPHYSICS_GAME STREQUAL "sdk2013" )
	message( FATAL_ERROR "Unknown VPHYSICS_GAME '${VPHYSICS_GAME}', expected sdk2013 or gmod" )
endif()
