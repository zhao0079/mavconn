##======================================================================## PIXHAWK Micro Air Vehicle Flying Robotics Toolkit# Please see our website at <http://pixhawk.ethz.ch># ## Original Authors:
#   @author Jan Woetzel 2004-2006#   @author Reto Grieder <www.orxonox.net>#   @author Fabian Landau <www.orxonox.net># Contributing Authors (in alphabetical order):#  # Todo:### (c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch># # This file is part of the PIXHAWK project# #     PIXHAWK is free software: you can redistribute it and/or modify#     it under the terms of the GNU General Public License as published by#     the Free Software Foundation, either version 3 of the License, or#     (at your option) any later version.# #     PIXHAWK is distributed in the hope that it will be useful,#     but WITHOUT ANY WARRANTY; without even the implied warranty of#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the#     GNU General Public License for more details.# #     You should have received a copy of the GNU General Public License#     along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.# ##========================================================================

FIND_PACKAGE(Doxygen QUIET)

IF (DOXYGEN_FOUND)
  # click+jump in Emacs and Visual Studio (for doxy.config) (jw)
  IF(CMAKE_BUILD_TOOL MATCHES "msdev|devenv")
    SET(DOXY_WARN_FORMAT "\"$file($line) : $text \"")
  ELSE()
    SET(DOXY_WARN_FORMAT "\"$file:$line: $text \"")
  ENDIF()

  SET(DOXY_INPUT_DIR  ${CMAKE_CURRENT_SOURCE_DIR})
  SET(DOXY_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
  
  # we need latex for doxygen because of the formulas
  # Orxonox doesn't
  #FIND_PACKAGE(LATEX)
  #IF    (NOT LATEX_COMPILER)
  #  MESSAGE(STATUS "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user inetraction on doxy run.")
  #ENDIF (NOT LATEX_COMPILER)
  #IF    (NOT MAKEINDEX_COMPILER)
  #  MESSAGE(STATUS "makeindex command MAKEINDEX_COMPILER not found but usually required.")
  #ENDIF (NOT MAKEINDEX_COMPILER)
  #IF    (NOT DVIPS_CONVERTER)
  #  MESSAGE(STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
  #ENDIF (NOT DVIPS_CONVERTER)
  
  IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/doxy.config.in)
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxy.config.in ${CMAKE_CURRENT_BINARY_DIR}/doxy.config @ONLY)
    # use (configured) doxy.config from (out of place) BUILD tree:
    SET(DOXY_CONFIG ${CMAKE_CURRENT_BINARY_DIR}/doxy.config)
  ELSE()
    MESSAGE(STATUS "Warning: Could not find dox.config.in in the root directory.")
  ENDIF()
  
  ADD_CUSTOM_TARGET(doc ${DOXYGEN_EXECUTABLE} ${DOXY_CONFIG})
  
  # create a windows help .chm file using hhc.exe
  # HTMLHelp DLL must be in path!
  # fallback: use hhw.exe interactively
  IF(WIN32)
    FIND_PACKAGE(HTMLHelp)
    IF(HTML_HELP_COMPILER)      
      SET(TMP ${DOXY_OUTPUT_DIR}/html/index.hhp)
      STRING(REGEX REPLACE "/" "\\\\" HHP_FILE ${TMP})
      ADD_CUSTOM_TARGET(chmdoc ${HTML_HELP_COMPILER} ${HHP_FILE})
      ADD_DEPENDENCIES(chmdoc doc)
      # Adding a dependency somehow adds doc target as default build target
      SET_TARGET_PROPERTIES(doc chmdoc PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD TRUE)
    ENDIF(HTML_HELP_COMPILER)
  ENDIF (WIN32) 


  # Installation
  INSTALL(
    FILES ${DOXY_CONFIG} ${CMAKE_CURRENT_BINARY_DIR}/doxy.log
    DESTINATION ${PIXHAWK_DOC_INSTALL_PATH}/api
    OPTIONAL
  )
  # Install command always needs the directory to work
  IF(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/html)
    FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html)
  ENDIF()
  INSTALL(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
    DESTINATION ${PIXHAWK_DOC_INSTALL_PATH}/api
  )
ENDIF(DOXYGEN_FOUND)
