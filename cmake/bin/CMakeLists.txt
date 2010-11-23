SET(RUN_SCRIPTS ferns)

# Create run scripts for Windows to manually add the DLL path when executing
IF(WIN32)
  FOREACH(_script ${RUN_SCRIPTS})
    LIST(REMOVE_ITEM RUN_SCRIPTS ${_script})
    LIST(APPEND RUN_SCRIPTS ${_script}.bat)
  ENDFOREACH(_script)
ENDIF()

GET_FILENAME_COMPONENT(CURRENT_RUNTIME_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ABSOLUTE)

FOREACH(_script ${RUN_SCRIPTS})
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${_script}.in ${CMAKE_BINARY_DIR}/${_script} @ONLY)
ENDFOREACH(_script)

# In order to evaluate at run time whether it is a dev build in the build tree,
# we plant a special file
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/pixhawk_dev_build.keep_me ${CURRENT_RUNTIME_DIR}/pixhawk_dev_build.keep_me)


# Make a symlink to be able to run from the root directory when installing copyable on Unix
#IF(UNIX AND INSTALL_COPYABLE)
#  INSTAll(CODE "EXECUTE_PROCESS(COMMAND ln -sf ${PIXHAWK_RUNTIME_INSTALL_PATH}/${PIXHAWK_EXECUTABLE_NAME} ${CMAKE_INSTALL_PREFIX}/pixhawk)")
#ENDIF()
