set(_sdk_public "${CMAKE_SOURCE_DIR}/sdk/mini-source-sdk/${VOX_SDK}/src/public")

set(_sources
    vbox_collide.cpp
    vbox_constraints.cpp
    vbox_controller_fluid.cpp
    vbox_controller_motion.cpp
    vbox_controller_player.cpp
    vbox_controller_shadow.cpp
    vbox_environment.cpp
    vbox_friction.cpp
    vbox_interface.cpp
    vbox_keyvalues_schema.cpp
    vbox_object.cpp
    vbox_objectpairhash.cpp
    vbox_parse.cpp
    vbox_surfaceprops.cpp
    compat/box3d_msvc32_compat.c
)

set(_headers
    cbase.h
    vbox_collide.h
    vbox_constraints.h
    vbox_controller_fluid.h
    vbox_controller_motion.h
    vbox_controller_player.h
    vbox_controller_shadow.h
    vbox_environment.h
    vbox_friction.h
    vbox_interface.h
    vbox_keyvalues_schema.h
    vbox_object.h
    vbox_objectpairhash.h
    vbox_parse.h
    vbox_surfaceprops.h
    vbox_util.h
    compat/branch_overrides.h
    compat/compat_asw.h
    compat/compat_sdk2013.h
)

set(_sdk_sources
    "${_sdk_public}/collisionutils.cpp"
    "${_sdk_public}/filesystem_helpers.cpp"
)

add_library(vphysics SHARED ${_sources} ${_headers} ${_sdk_sources})

target_include_directories(vphysics PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(vphysics PRIVATE box3d Vox::sdk)

target_precompile_headers(vphysics PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/cbase.h")
set_source_files_properties(${_sdk_sources} "${CMAKE_CURRENT_SOURCE_DIR}/compat/box3d_msvc32_compat.c"
    TARGET_DIRECTORY vphysics
    PROPERTIES SKIP_PRECOMPILE_HEADERS ON)

set_target_properties(vphysics PROPERTIES
    PREFIX ""
    OUTPUT_NAME "vphysics"
    FOLDER "vphysics"
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${_sources} ${_headers})
source_group("sdk" FILES ${_sdk_sources})
