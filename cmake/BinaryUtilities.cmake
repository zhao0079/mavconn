SET(__libraries CACHE INTERNAL "list of all libraries")
SET(__executables CACHE INTERNAL "list of all executables")

FUNCTION(_PIXHAWK_LIBRARY_INTERN _build _name _mode)
  IF (_build)
    SET(_build TRUE)
  ELSE ()
    SET(_build FALSE)
  ENDIF ()

  IF (_build)
    ADD_LIBRARY(${_name} ${_mode} ${ARGN})

    IF (NOT _mode STREQUAL "STATIC")
      PIXHAWK_INSTALL(${_name})
    ENDIF ()
  ENDIF ()

  STRING(TOUPPER ${_name} _name_upper)
  SET(${_name_upper}_FOUND ${_build} CACHE INTERNAL "TRUE if the library ${_name} will be built.")
  SET(${_name_upper}_BUILD ${_build} CACHE INTERNAL "TRUE if the library ${_name} will be built.")

  LIST(APPEND __libraries ${_name})
  SET(__libraries ${__libraries} CACHE INTERNAL "list of all libraries")
ENDFUNCTION(_PIXHAWK_LIBRARY_INTERN)

FUNCTION(_PIXHAWK_EXECUTABLE_INTERN _build _name)
  IF (_build)
    SET(_build TRUE)
  ELSE ()
    SET(_build FALSE)
  ENDIF ()

  IF (_build)
    ADD_EXECUTABLE(${_name} ${ARGN})
    PIXHAWK_INSTALL(${_name})
  ENDIF ()

  STRING(TOUPPER ${_name} _name_upper)
  SET(${_name_upper}_BUILD ${_build} CACHE INTERNAL "TRUE if the library ${_name} will be built.")

  LIST(APPEND __executables ${_name})
  SET(__executables ${__executables} CACHE INTERNAL "list of all executables")
ENDFUNCTION(_PIXHAWK_EXECUTABLE_INTERN)

FUNCTION(PIXHAWK_LIBRARY _name _mode)
  _PIXHAWK_LIBRARY_INTERN(TRUE ${_name} ${_mode} ${ARGN})
ENDFUNCTION(PIXHAWK_LIBRARY)

FUNCTION(PIXHAWK_EXECUTABLE _name)
  _PIXHAWK_EXECUTABLE_INTERN(TRUE ${_name} ${ARGN})
ENDFUNCTION(PIXHAWK_EXECUTABLE)

FUNCTION(PIXHAWK_LINK_LIBRARIES _name)
  STRING(TOUPPER ${_name} _name_upper)
  IF (${_name_upper}_BUILD)
    TARGET_LINK_LIBRARIES(${_name} ${ARGN})
  ENDIF ()
ENDFUNCTION(PIXHAWK_LINK_LIBRARIES)

FUNCTION(_PIXHAWK_GET_FILES_AND_CONDITIONS _conditionvar _filesvar _name)
  SET(_firstarg 1)

  SET(_condition)
  SET(_files)

  SET(_parsing_condition FALSE)
  SET(_parsing_files FALSE)

  FOREACH(_arg ${ARGN})
    STRING(TOUPPER ${_arg} _arg_upper)

    IF (_firstarg EQUAL 1 AND NOT _arg_upper STREQUAL "CONDITION" AND NOT _arg_upper STREQUAL "FILES")
      SET(_parsing_files TRUE)
    ENDIF ()
    SET(_firstarg 0)
    IF (_arg_upper STREQUAL "FILES")
      SET(_parsing_condition FALSE)
    ENDIF ()

    IF (_parsing_condition)
      IF (_arg_upper STREQUAL "TRUE")
        LIST(APPEND _condition 1)
      ELSEIF (_arg_upper STREQUAL "FALSE")
        LIST(APPEND _condition 0)
      ELSE ()
        LIST(APPEND _condition ${_arg})
      ENDIF ()
    ELSEIF (_parsing_files)
      LIST(APPEND _files ${_arg})
    ENDIF ()

    IF (_arg_upper STREQUAL "CONDITION")
      SET(_condition)
      SET(_parsing_condition TRUE)
    ENDIF ()
    IF (_arg_upper STREQUAL "FILES")
      SET(_files)
      SET(_parsing_files TRUE)
    ENDIF ()
  ENDFOREACH(_arg)

  IF (NOT DEFINED _condition)
    SET(_condition 1)
  ENDIF ()

  SET(${_filesvar} ${_files} PARENT_SCOPE)

  IF (${_condition})
     SET(${_conditionvar} 1 PARENT_SCOPE)
  ELSE ()
     SET(${_conditionvar} 0 PARENT_SCOPE)
  ENDIF ()   

  STRING(TOUPPER ${_name} _name_upper)
  SET(${_name_upper}_CONDITION ${_condition} CACHE INTERNAL "The condition of the binary ${_name}.")
ENDFUNCTION(_PIXHAWK_GET_FILES_AND_CONDITIONS)

FUNCTION(PIXHAWK_LIBRARY_CONDITIONAL _name _mode)
  _PIXHAWK_GET_FILES_AND_CONDITIONS(_condition _files ${_name} ${ARGN})
  _PIXHAWK_LIBRARY_INTERN(${_condition} ${_name} ${_mode} ${_files})
ENDFUNCTION(PIXHAWK_LIBRARY_CONDITIONAL _name _mode)

FUNCTION(PIXHAWK_EXECUTABLE_CONDITIONAL _name)
  _PIXHAWK_GET_FILES_AND_CONDITIONS(_condition _files ${_name} ${ARGN})
  _PIXHAWK_EXECUTABLE_INTERN(${_condition} ${_name} ${_files})
ENDFUNCTION(PIXHAWK_EXECUTABLE_CONDITIONAL _name _mode)


