set(_sdk_src "${CMAKE_CURRENT_SOURCE_DIR}/mini-source-sdk/${VOX_SDK}/src")

if(VOX_ARCH STREQUAL x64)
    set(_lib_dir "${_sdk_src}/lib/public/x64")
else()
    set(_lib_dir "${_sdk_src}/lib/public")
endif()

add_library(vox_sdk_headers INTERFACE)

target_include_directories(vox_sdk_headers SYSTEM INTERFACE
    "${_sdk_src}/public"
    "${_sdk_src}/public/tier0"
    "${_sdk_src}/public/tier1"
    "${_sdk_src}/public/mathlib"
    "${_sdk_src}/common"
)

set(_defs
    COMPILER_MSVC
    _DLL_EXT=dll
    RAD_TELEMETRY_DISABLED
    WIN32 _WIN32 _WINDOWS
    NOMINMAX
    _ALLOW_KEYWORD_MACROS
    _CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_WARNINGS
    _CRT_SECURE_NO_DEPRECATE _CRT_NONSTDC_NO_DEPRECATE
    _ALLOW_RUNTIME_LIBRARY_MISMATCH
    _ALLOW_ITERATOR_DEBUG_LEVEL_MISMATCH
    _ALLOW_MSC_VER_MISMATCH
)
if(VOX_ARCH STREQUAL x64)
    list(APPEND _defs PLATFORM_64BITS WIN64 _WIN64 COMPILER_MSVC64)
else()
    list(APPEND _defs COMPILER_MSVC32)
endif()
if(VOX_SDK STREQUAL gmod)
    list(APPEND _defs GAME_GMOD GAME_SDK2013)
elseif(VOX_SDK MATCHES "^sdk2013")
    list(APPEND _defs GAME_SDK2013)
elseif(VOX_SDK STREQUAL asw)
    list(APPEND _defs GAME_ASW)
endif()
target_compile_definitions(vox_sdk_headers INTERFACE ${_defs})

set(_tier1_sources
    bitbuf.cpp
    byteswap.cpp
    characterset.cpp
    checksum_crc.cpp
    checksum_md5.cpp
    checksum_sha1.cpp
    commandbuffer.cpp
    convar.cpp
    datamanager.cpp
    diff.cpp
    generichash.cpp
    ilocalize.cpp
    interface.cpp
    KeyValues.cpp
    kvpacker.cpp
    mempool.cpp
    memstack.cpp
    NetAdr.cpp
    newbitbuf.cpp
    processor_detect.cpp
    rangecheckedvar.cpp
    reliabletimer.cpp
    snappy-sinksource.cpp
    snappy-stubs-internal.cpp
    snappy.cpp
    sparsematrix.cpp
    splitstring.cpp
    stringpool.cpp
    strtools.cpp
    strtools_unicode.cpp
    tier1.cpp
    tokenreader.cpp
    uniqueid.cpp
    utlbinaryblock.cpp
    utlbuffer.cpp
    utlbufferutil.cpp
    utlstring.cpp
    utlsymbol.cpp
)

set(_mathlib_sources
    3dnow.cpp
    almostequal.cpp
    anorms.cpp
    bumpvects.cpp
    color_conversion.cpp
    halton.cpp
    IceKey.cpp
    imagequant.cpp
    lightdesc.cpp
    mathlib_base.cpp
    polyhedron.cpp
    powsse.cpp
    quantize.cpp
    randsse.cpp
    simdvectormatrix.cpp
    sparse_convolution_noise.cpp
    spherical.cpp
    sse.cpp
    sseconst.cpp
    ssenoise.cpp
    vector.cpp
    vmatrix.cpp
)

set(_tier2_sources
    tier2.cpp
)

function(vox_add_sdk_lib name define)
    set(_dir "${_sdk_src}/${name}")
    set(_srcs ${ARGN})
    list(TRANSFORM _srcs PREPEND "${_dir}/")
    add_library(vox_${name} STATIC ${_srcs})
    target_link_libraries(vox_${name} PRIVATE vox_sdk_headers)
    target_compile_definitions(vox_${name} PRIVATE ${define})
    target_include_directories(vox_${name} PRIVATE "${_dir}")
    set_target_properties(vox_${name} PROPERTIES FOLDER "sdk" SYSTEM TRUE)
    if(MSVC)
        target_compile_options(vox_${name} PRIVATE /W0)
        if(VOX_ARCH STREQUAL x86)
            target_compile_options(vox_${name} PRIVATE /arch:SSE)
        endif()
    endif()
endfunction()

set(_sdk_link
    "${_lib_dir}/tier0.lib"
    "${_lib_dir}/vstdlib.lib"
)

if(IS_DIRECTORY "${_sdk_src}/tier1")
    vox_add_sdk_lib(tier1 TIER1_STATIC_LIB ${_tier1_sources})
    list(APPEND _sdk_link vox_tier1)
else()
    list(APPEND _sdk_link "${_lib_dir}/tier1.lib")
endif()

if(IS_DIRECTORY "${_sdk_src}/tier2")
    vox_add_sdk_lib(tier2 TIER2_STATIC_LIB ${_tier2_sources})
    list(APPEND _sdk_link vox_tier2)
else()
    list(APPEND _sdk_link "${_lib_dir}/tier2.lib")
endif()

if(IS_DIRECTORY "${_sdk_src}/mathlib")
    vox_add_sdk_lib(mathlib MATHLIB_LIB ${_mathlib_sources})
    list(APPEND _sdk_link vox_mathlib)
else()
    list(APPEND _sdk_link "${_lib_dir}/mathlib.lib")
endif()

if(EXISTS "${_lib_dir}/interfaces.lib")
    list(APPEND _sdk_link "${_lib_dir}/interfaces.lib")
endif()

if(WIN32)
    list(APPEND _sdk_link Rpcrt4.lib)
endif()

add_library(vox_sdk INTERFACE)
add_library(Vox::sdk ALIAS vox_sdk)
target_link_libraries(vox_sdk INTERFACE vox_sdk_headers ${_sdk_link})
