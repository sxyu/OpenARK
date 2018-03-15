# - Find Intel librealsense 2 (https://github.com/IntelRealSense/librealsense)
#
# Usage of this module as follows:
#
#     find_package(REALSENSE2)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
#  REALSENSE2_DIR  Set this variable to the root installation of
#                   REALSENSE2 if the module has problems finding
#                   the proper installation path.
#  REALSENSE2_LIBRARY_DIR  Set this variable to where the RSSDK library is located
#
# Variables defined by this module:
#
#  REALSENSE2_FOUND              REALSENSE2_LIBRARY is set correctly
#
# All that is needed to use REALSENSE2 is that the environment variable 
# REALSENSE2_LIBRARY is set to the path to the REALSENSE2 library
# REALSENSE2_INCLUDE_DIR is set to
# This file simply checks that the environment variable is set correctly
#

if(NOT REALSENSE2_ROOT_DIR)
    set(REALSENSE2_ROOT_DIR "C:/Program Files/RealSense2/")
endif()

find_path(REALSENSE2_INCLUDE_DIRS
    NAMES librealsense2/rs.hpp
    PATHS $ENV{REALSENSE2_ROOT_DIR}/include ${REALSENSE2_ROOT_DIR}/include ${ARK_DEPENDENCY_DIR}/include
)

if(CMAKE_CL_64)
    find_library(REALSENSE2_LIBRARY
        NAMES realsense2
        PATHS $ENV{REALSENSE2_ROOT_DIR}/bin $ENV{REALSENSE2_ROOT_DIR}/Release $ENV{REALSENSE2_ROOT_DIR}/bin/x64
              ${REALSENSE2_ROOT_DIR}/bin ${REALSENSE2_ROOT_DIR}/Release ${REALSENSE2_ROOT_DIR}/bin/x64 ${ARK_DEPENDENCY_DIR}/lib
        PATH_SUFFIXES lib lib64
    )
else()
    find_library(REALSENSE2_LIBRARY
        NAMES realsense2
        PATHS $ENV{REALSENSE2_ROOT_DIR}/bin $ENV{REALSENSE2_ROOT_DIR}/Release $ENV{REALSENSE2_ROOT_DIR}/bin/Win32
              ${REALSENSE2_ROOT_DIR}/bin ${REALSENSE2_ROOT_DIR}/Release ${REALSENSE2_ROOT_DIR}/bin/Win32 ${ARK_DEPENDENCY_DIR}/lib
        PATH_SUFFIXES lib
    )
endif()

IF (REALSENSE2_INCLUDE_DIRS AND REALSENSE2_LIBRARY)
   SET(REALSENSE2_FOUND TRUE)
ENDIF (REALSENSE2_INCLUDE_DIRS AND REALSENSE2_LIBRARY)

IF (REALSENSE2_FOUND)
   # show which REALSENSE was found only if not quiet
   SET(REALSENSE2_LIBRARIES ${REALSENSE2_LIBRARY})
   IF (NOT REALSENSE_FIND_QUIETLY)
      MESSAGE(STATUS "Found librealsense2: ${REALSENSE2_LIBRARIES}")
   ENDIF (NOT REALSENSE2_FIND_QUIETLY)
ELSE (REALSENSE2_FOUND)
   # fatal error if REALSENSE is required but not found
   IF (REALSENSE2_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find librealsense2")
   ENDIF (REALSENSE2_FIND_REQUIRED)
ENDIF (REALSENSE2_FOUND)
