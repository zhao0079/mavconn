INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

SET(GLIBTOP_IncludeSearchPaths
  /usr/include/
  /usr/include/libgtop-2.0/
  /opt/local/include/libgtop-2.0/
)

SET(GLIBTOP_LibrarySearchPaths
  /usr/lib/
  /opt/local/lib/
)

FIND_PATH(GLIBTOP_INCLUDE_DIR
  NAMES glibtop.h
  PATHS ${GLIBTOP_IncludeSearchPaths}
)
FIND_LIBRARY(GLIBTOP_LIBRARY_OPTIMIZED
  NAMES gtop gtop-2.0 gtop-2.1 gtop-2.2 gtop-2.3 gtop-2.4 gtop-2.5 gtop-2.6 gtop-2.7 gtop-2.8 gtop-2.9 gtop-3.0
  PATHS ${GLIBTOP_LibrarySearchPaths}
)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
# The package is found if all variables listed are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GLIBTOP "Could NOT find glibtop library. Install using sudo apt-get install libgtop2-dev. Read Ubuntu installation instructions at http://pixhawk.ethz.ch/software/installation/ubuntu"
  GLIBTOP_LIBRARY_OPTIMIZED
  GLIBTOP_INCLUDE_DIR
)


# Collect optimized and debug libraries
HANDLE_LIBRARY_TYPES(GLIBTOP)

MARK_AS_ADVANCED(
  GLIBTOP_INCLUDE_DIR
  GLIBTOP_LIBRARY_OPTIMIZED
)
