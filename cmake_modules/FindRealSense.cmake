# - Find Intel librealsense 1 (https://github.com/IntelREALSENSE/librealsense)
#
#  REALSENSE_ROOT_DIR environment variable can be set to find the library.
#
# It sets the following variables:
#  REALSENSE_FOUND       - Set to false, or undefined, if RealSense isn't found.
#  REALSENSE_INCLUDE_DIRS - The REALSENSE include directory.
#  REALSENSE_LIBRARIES     - The REALSENSE library to link against.

if(NOT REALSENSE_ROOT_DIR)
    set(REALSENSE_ROOT_DIR "C:/Program Files/RealSense/")
endif()

find_path(REALSENSE_INCLUDE_DIRS
    NAMES librealsense/rs.hpp
    PATHS $ENV{REALSENSE_ROOT_DIR}/include ${REALSENSE_ROOT_DIR}/include ${ARK_DEPENDENCY_DIR}/include
)

if(CMAKE_CL_64)
    find_library(REALSENSE_LIBRARY
        NAMES realsense
        PATHS $ENV{REALSENSE_ROOT_DIR}/bin $ENV{REALSENSE_ROOT_DIR}/Release $ENV{REALSENSE_ROOT_DIR}/bin/x64
              ${REALSENSE_ROOT_DIR}/bin ${REALSENSE_ROOT_DIR}/Release ${REALSENSE_ROOT_DIR}/bin/x64 ${ARK_DEPENDENCY_DIR}/lib
        PATH_SUFFIXES lib lib64
    )
else()
    find_library(REALSENSE_LIBRARY
        NAMES realsense
        PATHS $ENV{REALSENSE_ROOT_DIR}/bin $ENV{REALSENSE_ROOT_DIR}/Release $ENV{REALSENSE_ROOT_DIR}/bin/Win32
              ${REALSENSE_ROOT_DIR}/bin ${REALSENSE_ROOT_DIR}/Release ${REALSENSE_ROOT_DIR}/bin/Win32 ${ARK_DEPENDENCY_DIR}/lib
        PATH_SUFFIXES lib
    )
endif()

IF (REALSENSE_INCLUDE_DIRS AND REALSENSE_LIBRARY)
   SET(REALSENSE_FOUND TRUE)
ENDIF (REALSENSE_INCLUDE_DIRS AND REALSENSE_LIBRARY)

IF (REALSENSE_FOUND)
   # show which REALSENSE was found only if not quiet
   SET(REALSENSE_LIBRARIES ${REALSENSE_LIBRARY})
   IF (NOT REALSENSE_FIND_QUIETLY)
      MESSAGE(STATUS "Found librealsense: ${REALSENSE_LIBRARIES}")
   ENDIF (NOT REALSENSE_FIND_QUIETLY)
ELSE (REALSENSE_FOUND)
   # fatal error if REALSENSE is required but not found
   IF (REALSENSE_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find librealsense")
   ENDIF (REALSENSE_FIND_REQUIRED)
ENDIF (REALSENSE_FOUND)

