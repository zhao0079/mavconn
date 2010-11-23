INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

SET(GSL_IncludeSearchPaths
  /usr/include/gsl/
  /usr/local/include/gsl/
  ../src/ARToolKit/gsl/
)

SET(GSL_LibrarySearchPaths
  /usr/lib/
  /usr/local/lib/
  ../src/ARToolKit/gsl/
)

FIND_PATH(GSL_INCLUDE_DIR gsl_test.h
  PATHS  ${GSL_IncludeSearchPaths}
)

FIND_LIBRARY(GSL_LIBRARY_OPTIMIZED
  NAMES gsl
  PATHS ${GSL_LibrarySearchPaths}
)

FIND_LIBRARY(GSLCBLAS_LIBRARY_OPTIMIZED
  NAMES gslcblas
  PATHS ${GSL_LibrarySearchPaths}
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(GSL "Could NOT find gsl"
  GSL_LIBRARY_OPTIMIZED
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(GSL "Could NOT find gslcblas (GSL)"
  GSLCBLAS_LIBRARY_OPTIMIZED
)

HANDLE_LIBRARY_TYPES(GSL)
HANDLE_LIBRARY_TYPES(GSLCBLAS)

MARK_AS_ADVANCED(
  GSL_LIBRARY_OPTIMIZED
  GSLCBLAS_LIBRARY_OPTIMIZED
)

