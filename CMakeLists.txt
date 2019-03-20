CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)
PROJECT( KrisLibrary )

# version (e.g. for packaging)
set(KRISLIBRARY_MAJOR_VERSION 1)
set(KRISLIBRARY_MINOR_VERSION 0)
set(KRISLIBRARY_PATCH_VERSION 0)
set(KRISLIBRARY_VERSION ${KRISLIBRARY_MAJOR_VERSION}.${KRISLIBRARY_MINOR_VERSION}.${KRISLIBRARY_PATCH_VERSION})
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_BINARY_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" ..)
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

LINK_DIRECTORIES(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# KrisLibrary source subdirectories
SET(SUBDIRS camera geometry GLdraw graph image math math3d meshing optimization planning robotics spline statistics structs utils)

# Get all compilation files in subdirectories
file(GLOB KrisLibrary_SRCS  ${PROJECT_SOURCE_DIR}/*.cpp )
foreach(s ${SUBDIRS})
  file(GLOB SUBDIR_SRCS  ${PROJECT_SOURCE_DIR}/${s}/*.cpp )
  SET(KrisLibrary_SRCS ${KrisLibrary_SRCS} ${SUBDIR_SRCS})
  SET(KrisLibrary_${s}_SRCS ${SUBDIR_SRCS})
endforeach(s)
file(GLOB PQP_SRCS ${PROJECT_SOURCE_DIR}/geometry/PQP/src/*.cpp)
SET(KrisLibrary_SRCS ${KrisLibrary_SRCS} ${PQP_SRCS})

IF(NOT WIN32)
  list(REMOVE_ITEM KrisLibrary_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/image/gdi.cpp)
ENDIF(NOT WIN32)

IF(WIN32)
  #output lib/Release/KrisLibrary.lib and lib/Debug/KrisLibraryd.lib
  if(NOT CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX d)
  endif()
ENDIF(WIN32)

#add all compilation files to the library
ADD_LIBRARY( KrisLibrary STATIC ${KrisLibrary_SRCS})




#Configure isinf / isnan etc using the CheckInfNan script
IF(NOT WIN32)
  INCLUDE (CheckInfNan) 
  IF(HAS_STD_CMATH)
    ADD_DEFINITIONS(-DHAS_STD_CMATH=1)
  ELSE(HAS_STD_CMATH)
    IF(HAS_DECL_ISINF)
      ADD_DEFINITIONS(-DHAVE_DECL_ISINF=1)
    ENDIF(HAS_DECL_ISINF)
    IF(HAS_DECL_ISNAN)
      ADD_DEFINITIONS(-DHAVE_DECL_ISNAN=1)
    ENDIF(HAS_DECL_ISNAN)
    IF(HAS_DECL_ISFINITE)
      ADD_DEFINITIONS(-DHAVE_DECL_ISFINITE=1)
    ENDIF(HAS_DECL_ISFINITE)
    IF(HAS_DECL_FINITE)
      ADD_DEFINITIONS(-DHAVE_DECL_FINITE=1)
    ENDIF(HAS_DECL_FINITE)
    IF(HAS_IEEE_COMPARISONS)
      ADD_DEFINITIONS(-DHAVE_IEEE_COMPARISONS=1)
    ENDIF(HAS_IEEE_COMPARISONS)
  ENDIF(HAS_STD_CMATH)
ENDIF(NOT WIN32)


# Configure external libraries using the KrisLibraryDependencies script #
INCLUDE(KrisLibraryDependencies)
ADD_DEFINITIONS(${KRISLIBRARY_DEFINITIONS})
INCLUDE_DIRECTORIES(${KRISLIBRARY_INCLUDE_DIRS})
IF(NOT WIN32)
  #windows has some weird stuff with link libraries
  TARGET_LINK_LIBRARIES(KrisLibrary ${KRISLIBRARY_LIBRARIES})
ENDIF(NOT WIN32)


# Documentation 
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.conf.in ${CMAKE_CURRENT_BINARY_DIR}/doxygen.conf @ONLY)
  ADD_CUSTOM_TARGET(docs ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxygen.conf
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating documentation (Doxygen) at location ${PROJECT_SOURCE_DIR}/doc/html/")
ENDIF(DOXYGEN_FOUND)


# uninstall target
configure_file(
    "${CMAKE_MODULE_PATH}/CMakeUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

#install targets
install(TARGETS KrisLibrary
  ARCHIVE
  DESTINATION lib
  COMPONENT libraries)
file(GLOB KrisLibrary_HDRS  ${PROJECT_SOURCE_DIR}/*.h )
install(FILES ${KrisLibrary_HDRS}
  DESTINATION include/KrisLibrary
  COMPONENT headers)
foreach(s ${SUBDIRS})
  file(GLOB SUBDIRFILES  ${PROJECT_SOURCE_DIR}/${s}/*.h )
  install(FILES ${SUBDIRFILES}
    DESTINATION include/KrisLibrary/${s}
    COMPONENT headers)
endforeach(s)

# Needs to be last statement:
INCLUDE(CPackSettings)

# Finished:
MESSAGE ("\n")
MESSAGE (STATUS "Compile KrisLibrary using: make")
MESSAGE (STATUS "Compile API-documentation using: make docs\n")
MESSAGE (STATUS "Install KrisLibrary using: make install")
MESSAGE (STATUS "Uninstall KrisLibrary using: make uninstall")
