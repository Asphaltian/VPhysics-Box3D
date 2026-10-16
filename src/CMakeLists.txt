add_library( vphysics SHARED
	vphysics/box3d_setup.cpp
	vphysics/conversion.cpp
	vphysics/create_interface.cpp
	vphysics/parse_file.cpp
	vphysics/physics.cpp
	vphysics/physics_car_system.cpp
	vphysics/physics_collide.cpp
	vphysics/physics_collision.cpp
	vphysics/physics_collision_set.cpp
	vphysics/physics_constraint.cpp
	vphysics/physics_constraint_solver.cpp
	vphysics/physics_controller.cpp
	vphysics/physics_environment.cpp
	vphysics/physics_fluid_controller.cpp
	vphysics/physics_friction_snapshot.cpp
	vphysics/physics_motion_controller.cpp
	vphysics/physics_object.cpp
	vphysics/physics_object_pair_hash.cpp
	vphysics/physics_player_controller.cpp
	vphysics/physics_polyhedron.cpp
	vphysics/physics_shadow_controller.cpp
	vphysics/physics_spring.cpp
	vphysics/physics_surface_props.cpp
	vphysics/physics_trace.cpp
	vphysics/physics_vehicle_controller.cpp
	vphysics/physics_virtual_mesh.cpp
	vphysics/vphysics_key_parser.cpp
	vphysics/vphysics_saverestore.cpp
	compact_builder/compact_surface.cpp
	compact_builder/surface_builder_ledge_soup.cpp
	compact_builder/surface_builder_pointsoup.cpp
	compact_builder/template_polygon.cpp
	"${SOURCE_SDK_SRC}/public/tier0/memoverride.cpp"
)

target_include_directories( vphysics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} )
target_link_libraries( vphysics PRIVATE vphysics_game source_sdk box3d::box3d qhull )

set_source_files_properties( "${SOURCE_SDK_SRC}/public/tier0/memoverride.cpp" PROPERTIES COMPILE_OPTIONS "$<IF:$<CXX_COMPILER_ID:MSVC>,/W0,-w>" )

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
