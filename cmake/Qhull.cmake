set( QHULL_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../external/qhull" )

add_library( qhull STATIC
	${QHULL_SOURCE_DIR}/geom.c
	${QHULL_SOURCE_DIR}/geom2.c
	${QHULL_SOURCE_DIR}/global.c
	${QHULL_SOURCE_DIR}/io.c
	${QHULL_SOURCE_DIR}/mem.c
	${QHULL_SOURCE_DIR}/merge.c
	${QHULL_SOURCE_DIR}/poly.c
	${QHULL_SOURCE_DIR}/poly2.c
	${QHULL_SOURCE_DIR}/qhull.c
	${QHULL_SOURCE_DIR}/qset.c
	${QHULL_SOURCE_DIR}/stat.c
	${QHULL_SOURCE_DIR}/user.c
	${CMAKE_CURRENT_LIST_DIR}/../src/compact_builder/qhull_sort.cpp
)

target_include_directories( qhull SYSTEM PUBLIC ${QHULL_SOURCE_DIR} )
target_compile_definitions( qhull PUBLIC qh_NOtrace )
set_target_properties( qhull PROPERTIES C_VISIBILITY_PRESET hidden )

set( QHULL_SORT_HEADER "${CMAKE_CURRENT_LIST_DIR}/../src/compact_builder/qhull_sort.h" )

if( MSVC )
	target_compile_definitions( qhull PRIVATE _CRT_SECURE_NO_WARNINGS )
	target_compile_options( qhull PRIVATE /W0 /fp:precise "/FI${QHULL_SORT_HEADER}" )
else()
	target_compile_options( qhull PRIVATE -w "SHELL:-include ${QHULL_SORT_HEADER}" )
endif()
