##======================================================================
#
# PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
# Please see our website at <http://pixhawk.ethz.ch>
# 
#
# Original Authors:
#   @author Reto Grieder <www.orxonox.net>
#   @author Fabian Landau <www.orxonox.net>
# Contributing Authors (in alphabetical order):
#  
# Todo:
#
#
# (c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>
# 
# This file is part of the PIXHAWK project
# 
#     PIXHAWK is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
# 
#     PIXHAWK is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.
# 
##========================================================================

################# Misc Options ##################

# Standard path suffixes, might not hold everywhere though
SET(DEFAULT_RUNTIME_PATH bin)
SET(DEFAULT_LIBRARY_PATH lib)
SET(DEFAULT_ARCHIVE_PATH lib/static)
SET(DEFAULT_DOC_PATH     doc)
SET(DEFAULT_MEDIA_PATH   media)
SET(DEFAULT_CONFIG_PATH  config)
SET(DEFAULT_LOG_PATH     log)
SET(DEFAULT_CAPTURE_PATH capture)

# Set output directories
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${DEFAULT_RUNTIME_PATH})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${DEFAULT_LIBRARY_PATH})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${DEFAULT_ARCHIVE_PATH})
# Do not set doc and media, rather check in the two subdirectories
# whether they concur with the DEFAULT_..._PATH
SET(CMAKE_MEDIA_OUTPUT_DIRECTORY   ${CMAKE_BINARY_DIR}/${DEFAULT_MEDIA_PATH})
SET(CMAKE_CONFIG_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/${DEFAULT_CONFIG_PATH})
SET(CMAKE_LOG_OUTPUT_DIRECTORY     ${CMAKE_BINARY_DIR}/${DEFAULT_LOG_PATH})
SET(CMAKE_CAPTURE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${DEFAULT_CAPTURE_PATH})

# Take care of some CMake 2.6.0 leftovers
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH LIBRARY_OUTPUT_PATH)

# Check if SSE 4.1 is supported

INCLUDE (CheckSymbolExists)
CHECK_SYMBOL_EXISTS(__SSE__ "xmmintrin.h" SUPPORTS_SSE1)
SET(SUPPORTS_SSE1 CACHE BOOL "supports SSE" ${SUPPORTS_SSE1})
SET(SUPPORTS_SSE1 TRUE)

CHECK_SYMBOL_EXISTS(__SSE2__ "emmintrin.h" SUPPORTS_SSE2)
SET(SUPPORTS_SSE2 CACHE BOOL "supports SSE2" ${SUPPORTS_SSE2})
SET(SUPPORTS_SSE2 TRUE)

CHECK_SYMBOL_EXISTS(__SSE4_1__ "smmintrin.h" SUPPORTS_SSE41)
SET(SUPPORTS_SSE41 CACHE BOOL "supports SSE4.1" ${SUPPORTS_SSE41})
#SET(SUPPORTS_SSE41 TRUE)

# Sets where to find the external libraries at runtime
# On Unix you should not have to change this at all.
IF(NOT PIXHAWK_RUNTIME_LIBRARY_DIRECTORY)
  SET(PIXHAWK_RUNTIME_LIBRARY_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
ENDIF()

# Set Debug build to default when not having multi-config generator like msvc
IF(NOT CMAKE_CONFIGURATION_TYPES)
  IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug CACHE STRING
        "Build types are: Debug, Release, MinSizeRel, RelWithDebInfo" FORCE)
  ENDIF()
  MARK_AS_ADVANCED(CLEAR CMAKE_BUILD_TYPE)

  MESSAGE(STATUS "*** Build type is ${CMAKE_BUILD_TYPE} ***")
ELSE()
  IF(CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE CACHE STRING FORCE)
  ENDIF()
  MARK_AS_ADVANCED(CMAKE_BUILD_TYPE)
ENDIF()


################ Compiler Config ################

OPTION(EXTRA_COMPILER_WARNINGS "Enable some extra warnings (heavily pollutes the output)" FALSE)

INCLUDE(FlagUtilities)

# Configure the compiler specific build options
IF(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUC)
  INCLUDE(BuildConfigGCC)
ELSEIF(MSVC)
  INCLUDE(BuildConfigMSVC)
ELSE()
  MESSAGE(STATUS "Warning: Your compiler is not officially supported.")
ENDIF()

SET(USER_SCRIPT_BUILD_CONFIG "" CACHE FILEPATH
    "Specify a CMake script if you wish to write your own build config.
     See BuildConfigGCC.cmake or BuildConfigMSVC.cmake for examples.")
IF(USER_SCRIPT_BUILD_CONFIG)
  IF(EXISTS ${CMAKE_MODULE_PATH}/${USER_SCRIPT_BUILD_CONFIG}.cmake)
    INCLUDE(${USER_SCRIPT_BUILD_CONFIG})
  ELSEIF(EXISTS ${USER_SCRIPT_BUILD_CONFIG})
    INCLUDE(${USER_SCRIPT_BUILD_CONFIG})
  ELSEIF(EXISTS ${CMAKE_MODULE_PATH}/${USER_SCRIPT_BUILD_CONFIG})
    INCLUDE(${CMAKE_MODULE_PATH}/${USER_SCRIPT_BUILD_CONFIG})
  ENDIF()
ENDIF(USER_SCRIPT_BUILD_CONFIG)


################# Test options ##################


############# Installation Settings #############

SET(_info_text "Puts all installed files in subfolders of the install prefix path. That root folder can then be moved, copied and renamed as you wish. The executable will not write to folders like ~/.pixhawk or \"Applictation Data\"")
IF(UNIX)
  OPTION(INSTALL_COPYABLE "${_info_text}" FALSE)
ELSE()
  OPTION(INSTALL_COPYABLE "${_info_text}" TRUE)
ENDIF()

IF(INSTALL_COPYABLE)
  # Note the relative paths. They will be resolved at runtime.
  # For CMake operations CMAKE_INSTALL_PREFIX is always appended.
  SET(PIXHAWK_RUNTIME_INSTALL_PATH ${DEFAULT_RUNTIME_PATH})
  SET(PIXHAWK_LIBRARY_INSTALL_PATH ${DEFAULT_LIBRARY_PATH})
  SET(PIXHAWK_ARCHIVE_INSTALL_PATH ${DEFAULT_ARCHIVE_PATH})
  SET(PIXHAWK_DOC_INSTALL_PATH     ${DEFAULT_DOC_PATH})
  SET(PIXHAWK_MEDIA_INSTALL_PATH   ${DEFAULT_MEDIA_PATH})
  SET(PIXHAWK_CONFIG_INSTALL_PATH  ${DEFAULT_CONFIG_PATH})
  SET(PIXHAWK_LOG_INSTALL_PATH     ${DEFAULT_LOG_PATH})
  SET(PIXHAWK_CAPTURE_INSTALL_PATH ${DEFAULT_CAPTURE_PATH})

ELSEIF(UNIX) # Apple too?
  # Using absolute paths
  SET(PIXHAWK_RUNTIME_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)
  SET(PIXHAWK_LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/pixhawk)
  SET(PIXHAWK_ARCHIVE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/pixhawk/static)
  SET(PIXHAWK_DOC_INSTALL_PATH     ${CMAKE_INSTALL_PREFIX}/share/doc/pixhawk)
  SET(PIXHAWK_MEDIA_INSTALL_PATH   ${CMAKE_INSTALL_PREFIX}/share/pixhawk)
  # These two paths are user and therefore runtime dependent --> only set relatively
  SET(PIXHAWK_MEDIA_INSTALL_PATH   ${DEFAULT_MEDIA_PATH})
  SET(PIXHAWK_CONFIG_INSTALL_PATH  ${DEFAULT_CONFIG_PATH})
  SET(PIXHAWK_LOG_INSTALL_PATH     ${DEFAULT_LOG_PATH})
  SET(PIXHAWK_CAPTURE_INSTALL_PATH ${DEFAULT_CAPTURE_PATH})

ELSEIF(WIN32)
  SET(PIXHAWK_RUNTIME_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${DEFAULT_RUNTIME_PATH})
  SET(PIXHAWK_LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${DEFAULT_LIBRARY_PATH})
  SET(PIXHAWK_ARCHIVE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${DEFAULT_ARCHIVE_PATH})
  SET(PIXHAWK_DOC_INSTALL_PATH     ${CMAKE_INSTALL_PREFIX}/${DEFAULT_DOC_PATH})
  SET(PIXHAWK_MEDIA_INSTALL_PATH   ${CMAKE_INSTALL_PREFIX}/${DEFAULT_MEDIA_PATH})
  # Leave empty because it is user and therefore runtime dependent
  SET(PIXHAWK_CONFIG_INSTALL_PATH  ${DEFAULT_CONFIG_PATH})
  SET(PIXHAWK_LOG_INSTALL_PATH     ${DEFAULT_LOG_PATH})
  SET(PIXHAWK_CAPTURE_INSTALL_PATH ${DEFAULT_CAPTURE_PATH})
ENDIF()


################## Unix rpath ###################

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# the RPATH to be used when installing
SET(CMAKE_INSTALL_RPATH ${PIXHAWK_LIBRARY_INSTALL_PATH})

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


######## Static/Dynamic linking defines #########

# Disable Boost auto linking completely
ADD_COMPILER_FLAGS("-DBOOST_ALL_NO_LIB")

# If no defines are specified, these libs get linked statically
ADD_COMPILER_FLAGS("-DBOOST_ALL_DYN_LINK" WIN32 LINK_BOOST_DYNAMIC)

