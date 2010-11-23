# - Try to find Xmu library
# Once done, this will define
#
#  XMU_FOUND - system has Xmu
#  XMU_LIBRARY - link this to use Xmu / GLUT

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(XMU_PKGCONF xmu)

# Finally the library itself
find_library(XMU_LIBRARY
  NAMES xmu
  PATHS ${XMU_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(XMU_PROCESS_LIBS XMU_LIBRARY)
libfind_process(XMU)

#INCLUDE(FindPackageHandleStandardArgs)
#INCLUDE(HandleLibraryTypes)
#
#SET(XMU_LibrarySearchPaths
#  /usr/lib/
#  /usr/local/lib/
#  /opt/local/lib/
#)
#
#FIND_LIBRARY(XMU_LIBRARY_OPTIMIZED
#  NAMES Xmu
#  PATHS ${XMU_LibrarySearchPaths}
#)
#
## Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
## The package is found if all variables listed are TRUE
#FIND_PACKAGE_HANDLE_STANDARD_ARGS(XMU "Could NOT find libXmu library. Install using sudo apt-get #install libxmu-dev. Only needed for visualization purposes."
#  XMU_LIBRARY_OPTIMIZED
#)
#
#
## Collect optimized and debug libraries
#HANDLE_LIBRARY_TYPES(XMU)
#
#MARK_AS_ADVANCED(
#  XMU_LIBRARY_OPTIMIZED
#)
