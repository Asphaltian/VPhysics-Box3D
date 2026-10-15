add_library( vphysics SHARED
	box3d_setup.cpp
	compact_surface.cpp
	create_interface.cpp
	parse_file.cpp
	physics_collide.cpp
	physics_collision.cpp
	physics_collision_set.cpp
	physics_object_pair_hash.cpp
	physics_surface_props.cpp
)

target_link_libraries( vphysics PRIVATE source_sdk box3d::box3d )

set_target_properties( vphysics PROPERTIES
	PREFIX ""
	CXX_VISIBILITY_PRESET hidden
	VISIBILITY_INLINES_HIDDEN ON
)

if( MSVC )
	target_compile_options( vphysics PRIVATE /W4 /permissive- /utf-8 )
else()
	target_compile_options( vphysics PRIVATE -Wall -Wextra )
endif()
