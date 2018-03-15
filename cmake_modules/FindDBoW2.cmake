# - Find DBoW2 library
# Find the native DBoW2 includes and library
# This module defines
#  DBoW2_INCLUDE_DIRS, where to find DBoW2.h, Set when
#                      DBoW2_INCLUDE_DIR is found.
#  DBoW2_LIBRARIES, libraries to link against to use DBoW2.
#  DBoW2_ROOT_DIR, The base directory to search for DBoW2.
#                  This can also be an environment variable.
#  DBoW2_FOUND, If false, do not try to use DBoW2.
#
# also defined, but not for general use are
#  DBoW2_LIBRARY, where to find the DBoW2 library.

# If DBoW2_ROOT_DIR was defined in the environment, use it.
IF(NOT DBoW2_ROOT_DIR AND NOT $ENV{DBoW2_ROOT_DIR} STREQUAL "")
  SET(DBoW2_ROOT_DIR $ENV{DBoW2_ROOT_DIR})
ENDIF()

SET(_DBoW2_SEARCH_DIRS
  ${DBoW2_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/DBoW2
)

FIND_PATH(DBoW2_INCLUDE_DIR
  NAMES
    DBoW2.h
  HINTS
    ${_DBoW2_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/include
    ${ARK_DEPENDENCY_DIR}/include/DBoW2
  PATH_SUFFIXES
    include
)

FIND_LIBRARY(DBoW2_LIBRARY
  NAMES
    DBoW2
  HINTS
    ${_DBoW2_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/lib
  PATH_SUFFIXES
    lib64 lib
  )

# handle the QUIETLY and REQUIRED arguments and set DBoW2_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(DBoW2 DEFAULT_MSG
    DBoW2_LIBRARY DBoW2_INCLUDE_DIR)

IF(DBoW2_FOUND)
  SET(DBoW2_LIBRARIES ${DBoW2_LIBRARY})
  SET(DBoW2_INCLUDE_DIRS ${DBoW2_INCLUDE_DIR})
  set(DBoW2_INCLUDES ${DBoW2_INCLUDE_DIRS})
  set(DBoW2_VERSION 1.11.0)
ENDIF(DBoW2_FOUND)

MARK_AS_ADVANCED(
  DBoW2_INCLUDE_DIR
  DBoW2_LIBRARY
)
