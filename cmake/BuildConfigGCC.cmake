##======================================================================
#
# PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
# Please see our website at <http://pixhawk.ethz.ch>
# 
#
# Original Authors:
#   @author Reto Grieder <www.orxonox.net>
#   @author Adrian Friedli <www.orxonox.net>
# Contributing Authors (in alphabetical order):
#  
# Todo:
#
#
# (c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>
# 
# This file is part of the PIXHAWK project
# 
#     PIXHAWK is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
# 
#     PIXHAWK is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.
# 
##========================================================================

# Determine compiler version
EXEC_PROGRAM(
  ${CMAKE_CXX_COMPILER}
  ARGS ${CMAKE_CXX_COMPILER_ARG1} -dumpversion
  OUTPUT_VARIABLE GCC_VERSION
)

# GCC may not support #pragma GCC system_header correctly when using
# templates. According to Bugzilla, it was fixed March 07 but tests
# have confirmed that GCC 4.0.0 does not pose a problem for our cases.
INCLUDE(CompareVersionStrings)
COMPARE_VERSION_STRINGS("${GCC_VERSION}" "4.0.0" _compare_result)
IF(_compare_result LESS 0)
#IF("${GCC_VERSION}" VERSION_LESS "4.0.0")
  SET(GCC_NO_SYSTEM_HEADER_SUPPORT TRUE)
ENDIF()

# Also include environment flags. Could cause conflicts though
SET_COMPILER_FLAGS("$ENV{CXXFLAGS}" CXX CACHE)
SET_COMPILER_FLAGS("$ENV{CFLAGS}"   C   CACHE)

# These flags get added to the flags above
SET_COMPILER_FLAGS("    -g -ggdb -D_DEBUG" Debug          CACHE)
SET_COMPILER_FLAGS("             -DNDEBUG -march=native" ReleaseAll     CACHE)
ADD_COMPILER_FLAGS("-O3                   -march=native" Release        CACHE)
ADD_COMPILER_FLAGS("-O2 -g -ggdb          -march=native" RelWithDebInfo CACHE)
ADD_COMPILER_FLAGS("-Os                   -march=native" MinSizeRel     CACHE)

IF (CMAKE_SYSTEM_PROCESSOR STREQUAL "i686" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
# ADD_COMPILER_FLAGS("-mtune=pentium4 -march=pentium4 -ftree-vectorize -msse2 -ffast-math -fexpensive-optimizations -fomit-frame-pointer -funroll-loops" Release CACHE)
  ADD_COMPILER_FLAGS("-fomit-frame-pointer -ftree-vectorize -ftree-vectorizer-verbose=1" Release CACHE)
ELSEIF (CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
  ADD_COMPILER_FLAGS("-mtune=cortex-a8 -march=armv7-a -ftree-vectorize -mfpu=neon -mfloat-abi=softfp -fexpensive-optimizations -fomit-frame-pointer -funroll-loops -ftree-vectorizer-verbose=1 -pthread" Release CACHE)
ENDIF ()

# CMake doesn't seem to set the PIC flags right on certain 64 bit systems
IF(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  ADD_COMPILER_FLAGS("-fPIC" CACHE)
ENDIF()

# We have some uncoformant code, disable an optimisation feature
REMOVE_COMPILER_FLAGS("-fno-strict-aliasing" CACHE)

# For GCC older than version 4, do not display sign compare warnings
# because of boost::filesystem (which creates about a hundred per include)
ADD_COMPILER_FLAGS("-Wno-sign-compare" GCC_NO_SYSTEM_HEADER_SUPPORT CACHE)

# For newer GCC (4.3 and above), don't display hundreds of annoying deprecated
# messages. Other versions don't seem to show any such warnings at all.
ADD_COMPILER_FLAGS("-Wno-deprecated" CXX CACHE)

COMPARE_VERSION_STRINGS("${GCC_VERSION}" "4.2.9" _compare_result)
IF(_compare_result GREATER 0)
#IF("${GCC_VERSION}" VERSION_GREATER "4.2.9")
# For GCC 4.4 and GCC 4.3 enable experimental C++0x support
  ADD_COMPILER_FLAGS("-std=gnu++0x" CXX CACHE)
ENDIF()

# Increase warning level if requested
IF(EXTRA_COMPILER_WARNINGS)
  ADD_COMPILER_FLAGS("-Wall -Wextra -Wno-unused-parameter" CACHE)
ELSE()
  REMOVE_COMPILER_FLAGS("-Wextra -Wno-unused-parameter" CACHE)
  ADD_COMPILER_FLAGS("-Wall" CACHE)
ENDIF()

# General linker flags
SET_LINKER_FLAGS("--no-undefined" CACHE)
