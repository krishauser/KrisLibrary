project(QHULL_LIB CXX C)
add_library(qhull OBJECT
	geom.c
	geom.h
	geom2.c
	global.c
	io.c
	io.h
	mem.c
	mem.h
	merge.c
	merge.h
	poly.c
	poly.h
	poly2.c
	qhull.c
	qhull.h
	qhull_a.h
	qset.c
	qset.h
	stat.c
	stat.h
	user.c
	user.h
)

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/*.c ${RPOJECT_SOURCE_DIR}/*.h)
set(QHULL_INCLUDE_DIR ${PROJECT_SOURCE_DIR} PARENT_SCOPE)

set(QHULL_SRC_FILES
	${SRC_FILES}
	PARENT_SCOPE
)



