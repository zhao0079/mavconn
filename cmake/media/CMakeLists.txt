##======================================================================
#
# PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
# Please see our website at <http://pixhawk.ethz.ch>
#
#
# Original Authors:
#   @author Reto Grieder
#   @author Fabian Landau
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

FILE(GLOB configfiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)
LIST(REMOVE_ITEM configfiles "CMakeLists.txt")

IF(NOT CMAKE_CONFIGURATION_TYPES)
  SET(_subdirs .)
  SEt(_default_subdir .)
ELSE()
  SET(_subdirs ${CMAKE_CONFIGURATION_TYPES})
  SEt(_default_subdir Debug)
ENDIF()

FOREACH(_subdir ${_subdirs})
  GET_FILENAME_COMPONENT(CURRENT_RUNTIME_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${_subdir} ABSOLUTE)

  FOREACH(_file ${configfiles})
    IF(NOT _file STREQUAL "characters")
      CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${_file} ${CURRENT_RUNTIME_DIR}/${DEFAULT_MEDIA_PATH}/${_file} COPYONLY)

      IF(_subdir STREQUAL _default_subdir)
        # Convenience script to be used when sitting in the binary directory. Defaults to Debug for msvc.
        CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${_file} ${CMAKE_BINARY_DIR}/${DEFAULT_MEDIA_PATH}/${_file} COPYONLY)
      ENDIF()
    ENDIF()
  ENDFOREACH(_file)
ENDFOREACH(_subdir)

# Configure the character media files
ADD_SUBDIRECTORY(characters)
