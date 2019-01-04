# -*- mode: cmake -*-
#
# Looks for CMake variable FREE_IMAGE_ROOT
#
# this files defines
# - FREE_IMAGE_INCLUDE_DIR
# - FREE_IMAGE_LIBRARY
# - FREE_IMAGE_FOUND

INCLUDE(CheckIncludeFileCXX)
CHECK_INCLUDE_FILE_CXX(FreeImage.h FREE_IMAGE_FOUND)

IF(WIN32)
  #not available on windows
ELSE (WIN32)
  FIND_LIBRARY( FREE_IMAGE_LIBRARY freeimage PATHS /usr/lib ${FREE_IMAGE_ROOT}/lib)

  FIND_PATH(FREE_IMAGE_INCLUDE_DIR
  FreeImage.h
  PATHS /usr/include/ /usr/include/FREE_IMAGE ${FREE_IMAGE_ROOT}/include
  DOC "Directory where FREE_IMAGE header files are stored" )
ENDIF(WIN32)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FREE_IMAGE "Could not find FREE_IMAGE " FREE_IMAGE_INCLUDE_DIR FREE_IMAGE_LIBRARY)
# show the FREE_IMAGE_INCLUDE_DIR and FREE_IMAGE_LIBRARY variables only in the advanced view
MARK_AS_ADVANCED(FREE_IMAGE_INCLUDE_DIR FREE_IMAGE_LIBRARY )

