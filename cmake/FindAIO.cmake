INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

SET(AIO_IncludeSearchPaths
  /usr/include/
  /usr/local/include/
  /opt/local/include/
)

SET(AIO_LibrarySearchPaths
  /lib/
  /usr/lib/
  /usr/local/lib/
  /opt/local/lib/
)

FIND_PATH(AIO_INCLUDE_DIR
  NAMES aio.h
  PATHS ${AIO_IncludeSearchPaths}
)
FIND_LIBRARY(AIO_LIBRARY_OPTIMIZED
  NAMES aio
  PATHS ${AIO_LibrarySearchPaths}
)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
# The package is found if all variables listed are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(AIO "Could NOT find aio library"
  AIO_LIBRARY_OPTIMIZED
  AIO_INCLUDE_DIR
)


# Collect optimized and debug libraries
HANDLE_LIBRARY_TYPES(AIO)

MARK_AS_ADVANCED(
  AIO_INCLUDE_DIR
  AIO_LIBRARY_OPTIMIZED
)
