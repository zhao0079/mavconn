INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

SET(DC1394_IncludeSearchPaths
  /usr/include/dc1394/
  /usr/local/include/dc1394/
  /opt/local/include/dc1394/
)

SET(DC1394_LibrarySearchPaths
  /usr/lib/
  /usr/local/lib/
  /opt/local/lib/
)

FIND_PATH(DC_INCLUDE_DIR
  NAMES dc1394.h
  PATHS ${DC1394_IncludeSearchPaths}
)
FIND_LIBRARY(DC_LIBRARY_OPTIMIZED
  NAMES dc1394
  PATHS ${DC1394_LibrarySearchPaths}
)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
# The package is found if all variables listed are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(DC1394 "Could NOT find dc1394 library"
  DC_LIBRARY_OPTIMIZED
  DC_INCLUDE_DIR
)


# Collect optimized and debug libraries
HANDLE_LIBRARY_TYPES(DC)

MARK_AS_ADVANCED(
  DC_INCLUDE_DIR
  DC_LIBRARY_OPTIMIZED
)
