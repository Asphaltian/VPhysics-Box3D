cmake_minimum_required(VERSION 3.25)
project(VPhysicsBox3D LANGUAGES C CXX)

set(VOX_SDK "" CACHE STRING "mini-source-sdk variant: gmod, sdk2013-mp, sdk2013-sp, asw")
set_property(CACHE VOX_SDK PROPERTY STRINGS gmod sdk2013-mp sdk2013-sp asw)

if(NOT VOX_SDK)
    message(FATAL_ERROR
        "VOX_SDK is unset. Configure with a preset from CMakePresets.json, "
        "e.g. `cmake --preset gmod-x64`.")
endif()

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/sdk/mini-source-sdk/${VOX_SDK}/src")
    message(FATAL_ERROR
        "SDK variant '${VOX_SDK}' not found. Expected "
        "sdk/mini-source-sdk/${VOX_SDK}/src. Did you run "
        "`git submodule update --init --recursive`?")
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/box3d/src/core.h")
    message(FATAL_ERROR
        "Box3D submodule is missing (box3d/src). Run "
        "`git submodule update --init --recursive`.")
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(VOX_ARCH x64)
else()
    set(VOX_ARCH x86)
endif()
message(STATUS "Vox3D: SDK '${VOX_SDK}', ${VOX_ARCH} (${CMAKE_GENERATOR})")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

if(MSVC)
    add_compile_options(/MP /W3 /Zc:strictStrings- /permissive)
else()
    add_compile_options(-msse2 $<$<COMPILE_LANGUAGE:CXX>:-fpermissive>)
endif()

set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT
    "$<$<CONFIG:Debug,RelWithDebInfo,Release>:ProgramDatabase>")
if(MSVC)
    add_link_options("$<$<CONFIG:Release>:/DEBUG;/OPT:REF;/OPT:ICF>")
endif()

add_compile_definitions("$<$<CONFIG:Debug>:DEBUG>")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(_box3d_root "${CMAKE_CURRENT_SOURCE_DIR}/box3d")
set(_box3d_sources
    aabb.c
    arena_allocator.c
    bitset.c
    block_allocator.c
    body.c
    broad_phase.c
    capsule.c
    compound.c
    constraint_graph.c
    contact.c
    contact_solver.c
    convex_manifold.c
    core.c
    distance.c
    distance_joint.c
    dynamic_tree.c
    height_field.c
    hull.c
    id_pool.c
    island.c
    joint.c
    manifold.c
    math_functions.c
    mesh.c
    mesh_contact.c
    motor_joint.c
    mover.c
    parallel_for.c
    parallel_joint.c
    physics_world.c
    prismatic_joint.c
    recording.c
    recording_replay.c
    revolute_joint.c
    scheduler.c
    sensor.c
    shape.c
    simd.c
    solver.c
    solver_set.c
    sphere.c
    spherical_joint.c
    table.c
    timer.c
    triangle_manifold.c
    types.c
    weld_joint.c
    wheel_joint.c
    world_snapshot.c
)
list(TRANSFORM _box3d_sources PREPEND "${_box3d_root}/src/")

add_library(box3d STATIC ${_box3d_sources})

target_include_directories(box3d
    PUBLIC  "${_box3d_root}/include"
    PRIVATE "${_box3d_root}/src"
)

set_target_properties(box3d PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED ON
    FOLDER "box3d"
    SYSTEM TRUE
)

if(MSVC)
    target_compile_options(box3d PRIVATE /Gw /Zc:inline)
    if(VOX_ARCH STREQUAL x86)
        target_compile_options(box3d PRIVATE /arch:SSE2)
    endif()
else()
    target_compile_options(box3d PRIVATE -msse2)
endif()

add_subdirectory(sdk)
add_subdirectory(vphysics_box3d)
