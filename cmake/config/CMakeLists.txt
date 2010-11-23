 # 5/9/2009: Adapted to the Pixhawk Project by Fabian Landau

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
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${_file} ${CURRENT_RUNTIME_DIR}/${DEFAULT_CONFIG_PATH}/${_file} COPYONLY)
    IF(_subdir STREQUAL _default_subdir)
      # Convenience script to be used when sitting in the binary directory. Defaults to Debug for msvc.
      CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${_file} ${CMAKE_BINARY_DIR}/${DEFAULT_CONFIG_PATH}/${_file} COPYONLY)
    ENDIF()
  ENDFOREACH(_file)
ENDFOREACH(_subdir)

