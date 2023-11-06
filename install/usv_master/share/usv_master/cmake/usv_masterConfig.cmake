# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_usv_master_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED usv_master_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(usv_master_FOUND FALSE)
  elseif(NOT usv_master_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(usv_master_FOUND FALSE)
  endif()
  return()
endif()
set(_usv_master_CONFIG_INCLUDED TRUE)

# output package information
if(NOT usv_master_FIND_QUIETLY)
  message(STATUS "Found usv_master: 0.0.0 (${usv_master_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'usv_master' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${usv_master_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(usv_master_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${usv_master_DIR}/${_extra}")
endforeach()
