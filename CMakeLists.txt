##======================================================================## PIXHAWK Micro Air Vehicle Flying Robotics Toolkit# Please see our website at <http://pixhawk.ethz.ch># ## Original Authors:#   @author Reto Grieder <www.orxonox.net>#   @author Fabian Landau <www.orxonox.net># Contributing Authors (in alphabetical order):#  # Todo:### (c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch># # This file is part of the PIXHAWK project# #     PIXHAWK is free software: you can redistribute it and/or modify#     it under the terms of the GNU General Public License as published by#     the Free Software Foundation, either version 3 of the License, or#     (at your option) any later version.# #     PIXHAWK is distributed in the hope that it will be useful,#     but WITHOUT ANY WARRANTY; without even the implied warranty of#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the#     GNU General Public License for more details.# #     You should have received a copy of the GNU General Public License#     along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.# ##========================================================================

CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)

# Keep devs from using the root directory as binary directory (messes up the source tree)
IF(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_BINARY_DIR})
  MESSAGE(FATAL_ERROR "Please do not use the root directory as CMake output directory!
  mkdir build; cd build; cmake ..
  And you will have to clean the source directory by deleting CMakeCache.txt and the folder CMakeFiles")
ENDIF()


PROJECT(Pixhawk C CXX)

# If this is set to true, CMake shows the compiler and linker commands
#SET(CMAKE_VERBOSE_MAKEFILE true)

# This sets where to look for modules (e.g. "Find*.cmake" files)
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Library finding
INCLUDE(LibraryConfig)

# General build and compiler options and configurations
INCLUDE(BuildConfig)

# PRINT macro
MACRO(PRINT)
  MESSAGE(STATUS ${ARGN})
ENDMACRO(PRINT)

# Create the actual project
ADD_SUBDIRECTORY(src)

# Configure the binary output directory. Do this after src!
ADD_SUBDIRECTORY(cmake/bin)

# Configure the media file directory
ADD_SUBDIRECTORY(cmake/media)

# Configure the config file directory
ADD_SUBDIRECTORY(cmake/config)

# Last but not least: Try to make doxygen target
ADD_SUBDIRECTORY(cmake/doc)

# Output
SET(_output_blanks "                              ")
MESSAGE(STATUS "")
MESSAGE(STATUS "---------------------------------------------------")
MESSAGE(STATUS "Executable:                   Build:    Condition:")
MESSAGE(STATUS "---------------------------------------------------")
FOREACH(_name ${__executables})
  STRING(TOUPPER ${_name} _name_upper)
  STRING(LENGTH ${_name} _name_length)
  MATH(EXPR _name_length "30 - ${_name_length}")
  STRING(SUBSTRING ${_output_blanks} 0 ${_name_length} _blanks)
  IF (NOT ${_name_upper}_BUILD)
    SET(_condition ${${_name_upper}_CONDITION})
  ELSE ()
    SET(_condition " (satisfied)")
  ENDIF ()
  MESSAGE(STATUS "${_name}${_blanks}${${_name_upper}_BUILD}     ${_condition}")
ENDFOREACH(_name)

MESSAGE(STATUS "")
MESSAGE(STATUS "---------------------------------------------------")
MESSAGE(STATUS "Library:                      Build:    Condition:")
MESSAGE(STATUS "---------------------------------------------------")
FOREACH(_name ${__libraries})
  STRING(TOUPPER ${_name} _name_upper)
  STRING(TOUPPER ${_name} _name_upper)
  STRING(LENGTH ${_name} _name_length)
  MATH(EXPR _name_length "30 - ${_name_length}")
  STRING(SUBSTRING ${_output_blanks} 0 ${_name_length} _blanks)
  IF (NOT ${_name_upper}_BUILD)
    SET(_condition ${${_name_upper}_CONDITION})
  ELSE ()
    SET(_condition " (satisfied)")
  ENDIF ()
  MESSAGE(STATUS "${_name}${_blanks}${${_name_upper}_BUILD}     ${_condition}")
ENDFOREACH(_name)
MESSAGE(STATUS "")

