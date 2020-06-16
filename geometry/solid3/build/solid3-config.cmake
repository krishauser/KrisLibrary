set(SOLID3_VERSION "3.5.8")
set(SOLID3_VERSION_MAJOR "3")
set(SOLID3_VERSION_MINOR "5")
set(SOLID3_VERSION_PATCH "8")


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was solid3-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include("${CMAKE_CURRENT_LIST_DIR}/solid3-export.cmake")

set(SOLID3_DEFINITIONS "")
set(SOLID3_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include")
set(SOLID3_LIBRARY_DIRS "${PACKAGE_PREFIX_DIR}/lib")
set(SOLID3_LIBRARIES "solid3::solid3")
