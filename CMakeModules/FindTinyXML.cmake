# - Find TinyXML
# Find the native TinyXML includes and library
#
# TINYXML_FOUND - True if TinyXML found.
# TINYXML_INCLUDE_DIR - where to find tinyxml.h, etc.
# TINYXML_LIBRARY - List of libraries when using TinyXML.
#

IF( TINYXML_INCLUDE_DIR )
# Already in cache, be silent
SET( TinyXML_FIND_QUIETLY TRUE )
ENDIF( TINYXML_INCLUDE_DIR )

FIND_PATH( TINYXML_INCLUDE_DIR "tinyxml.h"
  PATHS ${TINYXML_ROOT} ${TINYXML_ROOT}/tinyxml)

IF(WIN32)
  FIND_LIBRARY( TINYXML_LIBRARY_RELEASE
    NAMES "tinyxml_STL"
    PATHS ${TINYXML_ROOT} ${TINYXML_ROOT}/Release_STL )
  FIND_LIBRARY( TINYXML_LIBRARY_DEBUG
    NAMES "tinyxmld_STL"
    PATHS ${TINYXML_ROOT} ${TINYXML_ROOT}/Debug_STL )

    #this is used to pick between RELEASE and DEBUG library
    include(SelectLibraryConfigurations)
    select_library_configurations(TINYXML)
ELSE(WIN32)
  FIND_LIBRARY( TINYXML_LIBRARY
    NAMES "tinyxml"
    PATHS ${TINYXML_ROOT} )
ENDIF(WIN32)

# handle the QUIETLY and REQUIRED arguments and set TINYXML_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE( "FindPackageHandleStandardArgs" )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( "TinyXML" DEFAULT_MSG TINYXML_INCLUDE_DIR TINYXML_LIBRARY )

MARK_AS_ADVANCED( TINYXML_INCLUDE_DIR TINYXML_LIBRARY )

