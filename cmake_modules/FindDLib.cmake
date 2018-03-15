# - Find DLib library
# Find the native DLib includes and library
# This module defines
#  DLIB_INCLUDE_DIRS, where to find ceres.h, Set when
#                      DLIB_INCLUDE_DIR is found.
#  DLIB_LIBRARIES, libraries to link against to use DLib.
#  DLIB_ROOT_DIR, The base directory to search for DLib.
#                  This can also be an environment variable.
#  DLIB_FOUND, If false, do not try to use DLib.
#
# also defined, but not for general use are
#  DLIB_LIBRARY, where to find the DLib library.

# If DLIB_ROOT_DIR was defined in the environment, use it.
IF(NOT DLIB_ROOT_DIR AND NOT $ENV{DLIB_ROOT_DIR} STREQUAL "")
  SET(DLIB_ROOT_DIR $ENV{DLIB_ROOT_DIR})
ENDIF()

SET(_dlib_SEARCH_DIRS
  ${DLIB_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/dlib
)

FIND_PATH(DLIB_INCLUDE_DIR
  NAMES
    DUtils/DUtils.h
  HINTS
    ${_dlib_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/include    
  PATH_SUFFIXES
    include
)

FIND_LIBRARY(DLIB_LIBRARY
  NAMES
      DLib
  HINTS
    ${_dlib_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/lib
  PATH_SUFFIXES
    lib64 lib
  )

# handle the QUIETLY and REQUIRED arguments and set DLIB_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(DLib DEFAULT_MSG
    DLIB_LIBRARY DLIB_INCLUDE_DIR)

IF(DLIB_FOUND)
  SET(DLib_LIBRARIES ${DLIB_LIBRARY})
  SET(DLib_LIBS ${DLIB_LIBRARY})
  SET(DLib_INCLUDE_DIRS ${DLIB_INCLUDE_DIR})
  set(DLIB_VERSION 1.11.0)
ENDIF(DLIB_FOUND)

MARK_AS_ADVANCED(
  DLIB_INCLUDE_DIR
  DLIB_LIBRARY
)
