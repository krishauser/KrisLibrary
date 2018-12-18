#Finds KrisLibrary package.
#If KRISLIBRARY_ROOT is set, searches for a KrisLibrary install there
#defines 
# - KRISLIBRARY_LIBRARY
# - KRISLIBRARY_LIBRARIES
# - KRISLIBRARY_INCLUDE_DIRS
# - KRISLIBRARY_DEFINITIONS
#
# On Windows this will also define
# - KRISLIBRARY_LIBRARY_DEBUG / RELEASE
#
# This will properly configure a build to include all external libraries:
# KrisLibrary and its dependencies

#this will get everything but KrisLibrary
INCLUDE(KrisLibraryDependencies)

FIND_PATH(KRISLIBRARY_INCLUDE_DIR
  	KrisLibrary/File.h
  PATHS /usr/include /usr/local/include ${KRISLIBRARY_ROOT}
  DOC "Directory where KrisLibrary header files are stored" )

# Find KrisLibrary
IF(WIN32)
  include(FindPackageHandleStandardArgs)
  FIND_PATH(KRISLIBRARY_INCLUDE_DIR KrisLibrary/File.h
    PATHS ${KRISLIBRARY_ROOT}  )
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(KRISLIBRARY_LIBDIR ${KRISLIBRARY_ROOT}/x64 CACHE PATH "KrisLibrary lib directory" FORCE)
  ELSE()
    SET(KRISLIBRARY_LIBDIR ${KRISLIBRARY_ROOT} CACHE PATH "KrisLibrary lib directory" FORCE)
  ENDIF()
  
  FIND_LIBRARY(KRISLIBRARY_LIBRARY_DEBUG 
	NAMES KrisLibraryd
	PATHS ${KRISLIBRARY_LIBDIR})
  FIND_LIBRARY(KRISLIBRARY_LIBRARY_RELEASE
	NAMES KrisLibrary
	PATHS ${KRISLIBRARY_LIBDIR})

	#Note: do not use SelectLibraryConfigurations, this clobbers KRISLIBRARY_LIBRARIES
	SET(KRISLIBRARY_LIBRARY debug ${KRISLIBRARY_LIBRARY_DEBUG} optimized ${KRISLIBRARY_LIBRARY_RELEASE})
	

  find_package_handle_standard_args(KRISLIBRARY
	DEFAULT_MSG
	KRISLIBRARY_INCLUDE_DIR
	KRISLIBRARY_LIBRARY_DEBUG
	KRISLIBRARY_LIBRARY_RELEASE)
	
  if(NOT KRISLIBRARY_FOUND)
    MESSAGE("KrisLibrary not found!")
  endif( )
ELSE(WIN32)
  FIND_LIBRARY( KRISLIBRARY_LIBRARY KrisLibrary PATHS /usr/local/lib "${KRISLIBRARY_ROOT}/KrisLibrary/lib" )
  
  #do the find_package call...
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(KRISLIBRARY "Could not find KrisLibrary " KRISLIBRARY_INCLUDE_DIR KRISLIBRARY_LIBRARY)
ENDIF(WIN32)

SET(KRISLIBRARY_INCLUDE_DIRS ${KRISLIBRARY_INCLUDE_DIRS} ${KRISLIBRARY_INCLUDE_DIR})
SET(KRISLIBRARY_LIBRARIES ${KRISLIBRARY_LIBRARY} ${KRISLIBRARY_LIBRARIES})


SET(KRISLIBRARY_INCLUDE_DIRS ${KRISLIBRARY_INCLUDE_DIRS} CACHE STRING "KrisLibrary include dirs")
SET(KRISLIBRARY_LIBRARY ${KRISLIBRARY_LIBRARY} CACHE  STRING "KrisLibrary link library")
SET(KRISLIBRARY_LIBRARIES ${KRISLIBRARY_LIBRARIES} CACHE  STRING "KrisLibrary link libraries")



