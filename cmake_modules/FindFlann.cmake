# - Find Flann library
# Find the native Flann includes and library
# This module defines
#  FLANN_INCLUDE_DIRS, where to find flann.hpp, Set when
#                      FLANN_INCLUDE_DIR is found.
#  FLANN_LIBRARIES, libraries to link against to use Flann.
#  FLANN_ROOT_DIR, The base directory to search for Flann.
#                  This can also be an environment variable.
#  FLANN_FOUND, If false, do not try to use Flann.
#
# also defined, but not for general use are
#  FLANN_LIBRARY, where to find the Flann library.

# If FLANN_ROOT_DIR was defined in the environment, use it.
IF(NOT FLANN_ROOT_DIR AND NOT $ENV{FLANN_ROOT_DIR} STREQUAL "")
  SET(FLANN_ROOT_DIR $ENV{FLANN_ROOT_DIR})
ENDIF()

unset(FLANN_LIBRARY)
unset(FLANN_INCLUDE_DIRS)
unset(FLANN_LIBRARIES)

SET(_flann_SEARCH_DIRS
  ${FLANN_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/flann
)

FIND_PATH(FLANN_INCLUDE_DIR
  NAMES
    flann/flann.hpp
  HINTS
    ${_flann_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/include
  PATH_SUFFIXES
    include
)

find_library(FLANN_LIBRARY
    NAMES flann_cpp_s flann_cpp
    HINTS ${ARK_DEPENDENCY_DIR}/lib ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}" ${_flann_SEARCH_DIRS}
    PATHS ARK_DEPENDENCY_DIR}/lib "$ENV{PROGRAMFILES}/flann 1.6.9" "$ENV{PROGRAMW6432}/flann 1.6.9" 
      "$ENV{PROGRAMFILES}/flann" "$ENV{PROGRAMW6432}/flann"
    PATH_SUFFIXES lib64 lib
)

# handle the QUIETLY and REQUIRED arguments and set FLANN_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Flann DEFAULT_MSG
    FLANN_LIBRARY FLANN_INCLUDE_DIR)

IF(FLANN_FOUND)
  SET(FLANN_LIBRARIES ${FLANN_LIBRARY})
  SET(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
ENDIF(FLANN_FOUND)

MARK_AS_ADVANCED(
  FLANN_INCLUDE_DIR
  FLANN_LIBRARY
)
