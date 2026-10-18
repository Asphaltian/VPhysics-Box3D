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
	vphysics/physics_filter_table.cpp
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
	vphysics/physics_task_pool.cpp
	vphysics/physics_trace.cpp
	vphysics/physics_vehicle_controller.cpp
	vphysics/physics_virtual_mesh.cpp
	vphysics/vphysics_key_parser.cpp
	vphysics/vphysics_saverestore.cpp
	compact_builder/compact_surface.cpp
	compact_builder/qhull_build.c
	compact_builder/surface_builder_ledge_soup.cpp
	compact_builder/surface_builder_pointsoup.cpp
	compact_builder/template_polygon.cpp
	"${SOURCE_SDK_SRC}/public/tier0/memoverride.cpp"
)

add_library( tier0 SHARED tier0/tier0.cpp )
target_link_libraries( tier0 PRIVATE source_sdk )
target_compile_definitions( tier0 PRIVATE TIER0_DLL_EXPORT )
set_target_properties( tier0 PROPERTIES
	OUTPUT_NAME "$<IF:$<BOOL:${VPHYSICS_DEDICATED}>,tier0_srv,tier0>"
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tier0"
	LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tier0"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tier0"
)

target_include_directories( vphysics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} )
find_package( Threads REQUIRED )
target_link_libraries( vphysics PRIVATE source_sdk tier0 box3d::box3d qhull Threads::Threads )

set_source_files_properties( "${SOURCE_SDK_SRC}/public/tier0/memoverride.cpp" PROPERTIES COMPILE_OPTIONS "$<IF:$<CXX_COMPILER_ID:MSVC>,/W0,-w>" )

set_target_properties( vphysics PROPERTIES
	PREFIX ""
	OUTPUT_NAME "$<IF:$<BOOL:${VPHYSICS_DEDICATED}>,vphysics_srv,vphysics>"
	CXX_VISIBILITY_PRESET hidden
	VISIBILITY_INLINES_HIDDEN ON
	SKIP_BUILD_RPATH ON
)

if( MSVC )
	target_compile_options( vphysics PRIVATE /W4 /permissive- /utf-8 )
else()
	target_compile_options( vphysics PRIVATE -Wall -Wextra )
	target_link_options( vphysics PRIVATE LINKER:--no-undefined "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/vphysics/vphysics.map" )
	set_property( TARGET vphysics APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/vphysics/vphysics.map" )
endif()
