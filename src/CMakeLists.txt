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

add_library( tier1 STATIC
	"${SOURCE_SDK_SRC}/tier1/characterset.cpp"
	"${SOURCE_SDK_SRC}/tier1/convar.cpp"
	"${SOURCE_SDK_SRC}/tier1/strtools.cpp"
	"${SOURCE_SDK_SRC}/tier1/strtools_unicode.cpp"
	"${SOURCE_SDK_SRC}/tier1/tier1.cpp"
	"${SOURCE_SDK_SRC}/tier1/utlbuffer.cpp"
	"${SOURCE_SDK_SRC}/tier1/utlstring.cpp"
)
if( NOT WIN32 )
	target_sources( tier1 PRIVATE "${SOURCE_SDK_SRC}/tier1/qsort_s.cpp" )
endif()
target_link_libraries( tier1 PRIVATE source_sdk )
target_compile_options( tier1 PRIVATE "$<IF:$<CXX_COMPILER_ID:MSVC>,/W0,-w>" )
set_target_properties( tier1 PROPERTIES CXX_STANDARD 17 )

find_package( Git QUIET )
set( VPHYSICS_VERSION "unknown" )
set( VPHYSICS_BOX3D_VERSION "unknown" )
if( GIT_FOUND )
	execute_process( COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" OUTPUT_VARIABLE VPHYSICS_GIT_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE VPHYSICS_GIT_RESULT )
	if( VPHYSICS_GIT_RESULT EQUAL 0 )
		set( VPHYSICS_VERSION "${VPHYSICS_GIT_VERSION}" )
	endif()
	execute_process( COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/external/box3d" OUTPUT_VARIABLE VPHYSICS_GIT_BOX3D_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE VPHYSICS_GIT_BOX3D_RESULT )
	if( VPHYSICS_GIT_BOX3D_RESULT EQUAL 0 )
		set( VPHYSICS_BOX3D_VERSION "${VPHYSICS_GIT_BOX3D_VERSION}" )
	endif()
	execute_process( COMMAND "${GIT_EXECUTABLE}" rev-parse --git-path logs/HEAD WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" OUTPUT_VARIABLE VPHYSICS_GIT_LOG OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET )
	get_filename_component( VPHYSICS_GIT_LOG "${VPHYSICS_GIT_LOG}" ABSOLUTE BASE_DIR "${PROJECT_SOURCE_DIR}" )
	if( EXISTS "${VPHYSICS_GIT_LOG}" )
		set_property( DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${VPHYSICS_GIT_LOG}" )
	endif()
endif()

if( VPHYSICS_DEDICATED )
	set( VPHYSICS_PLATFORM "dedicated-linux" )
elseif( WIN32 )
	set( VPHYSICS_PLATFORM "windows" )
else()
	set( VPHYSICS_PLATFORM "linux" )
endif()
string( APPEND VPHYSICS_PLATFORM "$<IF:$<EQUAL:${CMAKE_SIZEOF_VOID_P},8>,-x64,-x86>" )

set_source_files_properties( vphysics/physics.cpp PROPERTIES COMPILE_DEFINITIONS "VPHYSICS_VERSION=\"${VPHYSICS_VERSION}\";VPHYSICS_BOX3D_VERSION=\"${VPHYSICS_BOX3D_VERSION}\";VPHYSICS_PLATFORM=\"${VPHYSICS_PLATFORM}\"" )

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
target_link_libraries( vphysics PRIVATE source_sdk tier0 tier1 box3d::box3d qhull Threads::Threads )

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
	add_custom_command( TARGET vphysics POST_BUILD
		COMMAND "${CMAKE_OBJCOPY}" --only-keep-debug "$<TARGET_FILE_NAME:vphysics>" "$<TARGET_FILE_NAME:vphysics>.debug"
		COMMAND "${CMAKE_OBJCOPY}" --discard-all "--add-gnu-debuglink=$<TARGET_FILE_NAME:vphysics>.debug" "$<TARGET_FILE_NAME:vphysics>"
		WORKING_DIRECTORY "$<TARGET_FILE_DIR:vphysics>"
		VERBATIM
	)
endif()
