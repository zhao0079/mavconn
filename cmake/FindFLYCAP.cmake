INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

SET(FlyCapture_IncludeSearchPaths
  /usr/include/flycapture/
)

SET(FlyCapture_LibrarySearchPaths
  /usr/lib/
)

FIND_PATH(FLYCAPTURE2_INCLUDE_DIR FlyCapture2.h
  PATHS ${FlyCapture_IncludeSearchPaths}
)
FIND_LIBRARY(FLYCAPTURE2_LIBRARY_OPTIMIZED
  NAMES flycapture
  PATHS ${FlyCapture_LibrarySearchPaths}
)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
FIND_PACKAGE_HANDLE_STANDARD_ARGS(FlyCapture2 "Could NOT find Flycapture2. Only required for testing purposes. Please continue."
  FLYCAPTURE2_LIBRARY_OPTIMIZED
  FLYCAPTURE2_INCLUDE_DIR
)

IF(FLYCAPTURE2_FOUND)
 FIND_PACKAGE_MESSAGE(FLYCAPTURE2_FOUND "Found Fly Capture SDK  ${FLYCAPTURE2_LIBRARY_OPTIMIZED}" "[${FLYCAPTURE2_LIBRARY_OPTIMIZED}][${FLYCAPTURE2_INCLUDE_DIR}]")
ENDIF(FLYCAPTURE2_FOUND)

# Collect optimized and debug libraries
HANDLE_LIBRARY_TYPES(FLYCAPTURE2)

MARK_AS_ADVANCED(
  FLYCAPTURE2_INCLUDE_DIR
  FLYCAPTURE2_LIBRARY_OPTIMIZED
)
