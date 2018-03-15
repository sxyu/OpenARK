# - Find OpenGV library
# Find the native OpenGV includes and library
# This module defines
#  OpenGV_INCLUDE_DIRS, where to find OpenGV.h, Set when
#                      OpenGV_INCLUDE_DIR is found.
#  OpenGV_LIBRARIES, libraries to link against to use OpenGV.
#  OpenGV_ROOT_DIR, The base directory to search for OpenGV.
#                  This can also be an environment variable.
#  OpenGV_FOUND, If false, do not try to use OpenGV.
#
# also defined, but not for general use are
#  OpenGV_LIBRARY, where to find the OpenGV library.

# If OpenGV_ROOT_DIR was defined in the environment, use it.
IF(NOT OpenGV_ROOT_DIR AND NOT $ENV{OpenGV_ROOT_DIR} STREQUAL "")
  SET(OpenGV_ROOT_DIR $ENV{OpenGV_ROOT_DIR})
ENDIF()

SET(_OpenGV_SEARCH_DIRS
  ${OpenGV_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/glog
)

FIND_PATH(OPENGV_INCLUDE_DIR
  NAMES
    opengv/Indices.hpp
  HINTS
    ${_OPENGV_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/include
  PATH_SUFFIXES
    include
)

FIND_LIBRARY(OPENGV_LIBRARY
  NAMES
    opengv
  HINTS
    ${_OPENGV_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/lib
  PATH_SUFFIXES
    lib64 lib
  )

# handle the QUIETLY and REQUIRED arguments and set OpenGV_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OpenGV DEFAULT_MSG
    OPENGV_LIBRARY OPENGV_INCLUDE_DIR)

IF(OpenGV_FOUND)
  SET(OPENGV_LIBRARIES ${OPENGV_LIBRARY})
  SET(OPENGV_INCLUDE_DIRS ${OPENGV_INCLUDE_DIR})
  SET(OPENGV_INCLUDES ${OPENGV_INCLUDE_DIRS})
ENDIF(OpenGV_FOUND)
