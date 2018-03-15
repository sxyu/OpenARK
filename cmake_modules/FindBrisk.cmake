# - Find Brisk library
# Find the native Brisk includes and library
# This module defines
#  BRISK_INCLUDE_DIRS, where to find brisk.h, Set when
#                      BRISK_INCLUDE_DIR is found.
#  BRISK_LIBRARIES, libraries to link against to use Brisk.
#  BRISK_ROOT_DIR, The base directory to search for Brisk.
#                  This can also be an environment variable.
#  BRISK_FOUND, If false, do not try to use Brisk.
#
# also defined, but not for general use are
#  BRISK_LIBRARY, where to find the Brisk library.

# If BRISK_ROOT_DIR was defined in the environment, use it.
IF(NOT BRISK_ROOT_DIR AND NOT $ENV{BRISK_ROOT_DIR} STREQUAL "")
  SET(BRISK_ROOT_DIR $ENV{BRISK_ROOT_DIR})
ENDIF()

SET(_brisk_SEARCH_DIRS
  ${BRISK_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/brisk
)

FIND_PATH(BRISK_INCLUDE_DIR
  NAMES
    brisk/brisk.h
  HINTS
    ${_brisk_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/include
  PATH_SUFFIXES
    include
)

FIND_LIBRARY(BRISK_LIBRARY
  NAMES
    brisk
  HINTS
    ${_brisk_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/lib
  PATH_SUFFIXES
    lib64 lib
  )

# handle the QUIETLY and REQUIRED arguments and set BRISK_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Brisk DEFAULT_MSG
    BRISK_LIBRARY BRISK_INCLUDE_DIR)

IF(BRISK_FOUND)
  SET(BRISK_LIBRARIES ${BRISK_LIBRARY})
  SET(BRISK_INCLUDE_DIRS ${BRISK_INCLUDE_DIR})
  SET(BRISK_INCLUDES ${BRISK_INCLUDE_DIRS})
ENDIF(BRISK_FOUND)

MARK_AS_ADVANCED(
  BRISK_INCLUDE_DIR
  BRISK_LIBRARY
)
