# -*- mode: cmake -*-
#
# Looks for CMake variable FreeImage_ROOT
#
# this files defines
# - FreeImage_INCLUDE_DIR
# - FreeImage_LIBRARY
# - FreeImage_FOUND

INCLUDE(CheckIncludeFileCXX)
CHECK_INCLUDE_FILE_CXX(FreeImage.h FreeImage_FOUND)

IF(WIN32)
  #not available on windows
ELSE (WIN32)
  FIND_LIBRARY( FreeImage_LIBRARY freeimage PATHS /usr/lib ${FreeImage_ROOT}/lib)

  FIND_PATH(FreeImage_INCLUDE_DIR
  FreeImage.h
  PATHS /usr/include/ /usr/include/FreeImage ${FreeImage_ROOT}/include
  DOC "Directory where FreeImage header files are stored" )
ENDIF(WIN32)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FreeImage "Could not find FreeImage " FreeImage_INCLUDE_DIR FreeImage_LIBRARY)
# show the FreeImage_INCLUDE_DIR and FreeImage_LIBRARY variables only in the advanced view
MARK_AS_ADVANCED(FreeImage_INCLUDE_DIR FreeImage_LIBRARY )

