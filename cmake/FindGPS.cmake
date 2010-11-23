INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

SET(GPS_IncludeSearchPaths
  /usr/include/
  /opt/local/include/
)

SET(GPS_LibrarySearchPaths
  /usr/lib/
  /opt/local/lib/
)

FIND_PATH(GPS_INCLUDE_DIR
  NAMES gps.h
  PATHS ${GPS_IncludeSearchPaths}
)
FIND_LIBRARY(GPS_LIBRARY_OPTIMIZED
  NAMES gps
  PATHS ${GPS_LibrarySearchPaths}
)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
# The package is found if all variables listed are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GPS "Could NOT find gps/gpsd library. Install using sudo apt-get install gpsd libgps-dev. Read Ubuntu installation instructions at http://pixhawk.ethz.ch/software/installation/ubuntu"
  GPS_LIBRARY_OPTIMIZED
  GPS_INCLUDE_DIR
)


# Collect optimized and debug libraries
HANDLE_LIBRARY_TYPES(GPS)

MARK_AS_ADVANCED(
  GPS_INCLUDE_DIR
  GPS_LIBRARY_OPTIMIZED
)
