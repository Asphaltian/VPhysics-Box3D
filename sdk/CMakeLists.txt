set(_sdk_src "${CMAKE_CURRENT_SOURCE_DIR}/mini-source-sdk/${VOX_SDK}/src")

if(WIN32)
    if(VOX_ARCH STREQUAL x64)
        set(_lib_dir "${_sdk_src}/lib/public/x64")
    else()
        set(_lib_dir "${_sdk_src}/lib/public")
    endif()
else()
    if(VOX_ARCH STREQUAL x64)
        set(_lib_dir "${_sdk_src}/lib/public/linux64")
    else()
        set(_lib_dir "${_sdk_src}/lib/public/linux32")
    endif()
endif()

add_library(vox_sdk_headers INTERFACE)

target_include_directories(vox_sdk_headers SYSTEM INTERFACE
    "${_sdk_src}/public"
    "${_sdk_src}/public/tier0"
    "${_sdk_src}/public/tier1"
    "${_sdk_src}/public/mathlib"
    "${_sdk_src}/common"
)

set(_defs RAD_TELEMETRY_DISABLED)
if(WIN32)
    list(APPEND _defs
        COMPILER_MSVC
        _DLL_EXT=dll
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
else()
    list(APPEND _defs GNUC POSIX _POSIX COMPILER_GCC _DLL_EXT=so _LINUX LINUX)
    if(VOX_ARCH STREQUAL x64)
        list(APPEND _defs PLATFORM_64BITS)
    endif()
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
if(WIN32)
    list(APPEND _tier1_sources processor_detect.cpp)
else()
    list(APPEND _tier1_sources processor_detect_linux.cpp qsort_s.cpp pathmatch.cpp)
endif()

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
    else()
        target_compile_options(vox_${name} PRIVATE -w -msse2)
    endif()
endfunction()

function(vox_prebuilt out name)
    if(WIN32)
        set(${out} "${_lib_dir}/${name}.lib" PARENT_SCOPE)
    elseif(name STREQUAL tier2)
        set(${out} "${_lib_dir}/${name}.a" PARENT_SCOPE)
    else()
        set(${out} "${_lib_dir}/lib${name}.so" PARENT_SCOPE)
    endif()
endfunction()

vox_prebuilt(_tier0_lib tier0)
vox_prebuilt(_vstdlib_lib vstdlib)
set(_sdk_link "${_tier0_lib}" "${_vstdlib_lib}")

foreach(_pair "tier1;_tier1_sources;TIER1_STATIC_LIB" "tier2;_tier2_sources;TIER2_STATIC_LIB" "mathlib;_mathlib_sources;MATHLIB_LIB")
    list(GET _pair 0 _lib)
    list(GET _pair 1 _srcvar)
    list(GET _pair 2 _def)
    if(IS_DIRECTORY "${_sdk_src}/${_lib}")
        vox_add_sdk_lib(${_lib} ${_def} ${${_srcvar}})
        list(APPEND _sdk_link vox_${_lib})
    else()
        vox_prebuilt(_pl ${_lib})
        list(APPEND _sdk_link "${_pl}")
    endif()
endforeach()

if(WIN32 AND EXISTS "${_lib_dir}/interfaces.lib")
    list(APPEND _sdk_link "${_lib_dir}/interfaces.lib")
endif()

if(WIN32)
    list(APPEND _sdk_link Rpcrt4.lib)
else()
    list(APPEND _sdk_link ${CMAKE_DL_LIBS} pthread)
endif()

add_library(vox_sdk INTERFACE)
add_library(Vox::sdk ALIAS vox_sdk)
target_link_libraries(vox_sdk INTERFACE vox_sdk_headers ${_sdk_link})
