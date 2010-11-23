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
 #  Description:
 #    Several functions that help organising the source tree.
 #    [ADD/SET]_SOURCE_FILES - Writes source files to the cache by force and
 #                             adds the current directory.
 #    GET_ALL_HEADER_FILES - Finds all header files recursively.
 #    GENERATE_SOURCE_GROUPS - Set Visual Studio source groups.
 #

# Adds source files with the full path to a list
FUNCTION(ADD_SOURCE_FILES _varname)
  # Prefix the full path
  SET(_fullpath_sources)
  FOREACH(_file ${ARGN})
    GET_SOURCE_FILE_PROPERTY(_filepath ${_file} LOCATION)
    LIST(APPEND _fullpath_sources ${_filepath})
  ENDFOREACH(_file)
  # Write into the cache to avoid variable scoping in subdirs
  SET(${_varname} ${${_varname}} ${_fullpath_sources} CACHE INTERNAL "Do not edit")
ENDFUNCTION(ADD_SOURCE_FILES)


# Sets source files with the full path
FUNCTION(SET_SOURCE_FILES _varname)
  # Prefix the full path
  SET(_fullpath_sources)
  FOREACH(_file ${ARGN})
    GET_SOURCE_FILE_PROPERTY(_filepath ${_file} LOCATION)
    LIST(APPEND _fullpath_sources ${_filepath})
  ENDFOREACH(_file)
  # Write into the cache to avoid variable scoping in subdirs
  SET(${_varname} ${_fullpath_sources} CACHE INTERNAL "Do not edit")
ENDFUNCTION(SET_SOURCE_FILES)


# Search the entire directory tree for header files and add them to a variable
MACRO(GET_ALL_HEADER_FILES _target_varname)
  FILE(GLOB_RECURSE ${_target_varname} ${CMAKE_CURRENT_SOURCE_DIR} "*.h")
ENDMACRO(GET_ALL_HEADER_FILES)


# Generate source groups according to the directory structure
FUNCTION(GENERATE_SOURCE_GROUPS)

  FOREACH(_file ${ARGN})
    GET_SOURCE_FILE_PROPERTY(_full_filepath ${_file} LOCATION)
    FILE(RELATIVE_PATH _relative_path ${CMAKE_CURRENT_SOURCE_DIR} ${_full_filepath})
    GET_FILENAME_COMPONENT(_relative_path ${_relative_path} PATH)
    STRING(REPLACE "/" "\\\\" _group_path "${_relative_path}")
    SOURCE_GROUP("Source\\${_group_path}" FILES ${_file})
  ENDFOREACH(_file)

ENDFUNCTION(GENERATE_SOURCE_GROUPS)
