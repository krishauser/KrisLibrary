#Finds KrisLibrary package.
#If KRISLIBRARY_ROOT is set, searches for a KrisLibrary install there
#defines 
# - KRISLIBRARY_LIBRARIES
# - KRISLIBRARY_INCLUDE_DIRS
# - KRISLIBRARY_DEFINITIONS
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
  FIND_LIBRARY(KRISLIBRARY_LIBRARY_DEBUG 
	NAMES KrisLibraryd
	PATHS ${KRISLIBRARY_ROOT})
  FIND_LIBRARY(KRISLIBRARY_LIBRARY_RELEASE
	NAMES KrisLibrary
	PATHS ${KRISLIBRARY_ROOT})
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
  # show the BERKELEY_DB_INCLUDE_DIR and BERKELEY_DB_LIBRARIES variables only in the advanced view
  ENDIF(WIN32)

SET(KRISLIBRARY_INCLUDE_DIRS ${KRISLIBRARY_INCLUDE_DIRS} ${KRISLIBRARY_INCLUDE_DIR})
SET(KRISLIBRARY_LIBRARIES ${KRISLIBRARY_LIBRARY} ${KRISLIBRARY_LIBRARIES})

MARK_AS_ADVANCED(KRISLIBRARY_INCLUDE_DIR KRISLIBRARY_LIBRARY )
MARK_AS_ADVANCED(KRISLIBRARY_INCLUDE_DIRS KRISLIBRARY_LIBRARIES )


