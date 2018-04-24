# - Find OpenCV library
# Find the native OpenCV includes and library
# This module defines
#  OPENCV_INCLUDE_DIRS, where to find opencv.h, Set when
#                      OPENCV_INCLUDE_DIR is found.
#  OPENCV_LIBRARIES, libraries to link against to use OpenCV.
#  OPENCV_ROOT_DIR, The base directory to search for OpenCV.
#                  This can also be an environment variable.
#  OPENCV_FOUND, If false, do not try to use OpenCV.
#
# also defined, but not for general use are
#  OPENCV_LIBRARY, where to find the OpenCV library.

# If OPENCV_ROOT_DIR was defined in the environment, use it.
if(NOT OPENCV_ROOT_DIR AND NOT $ENV{OPENCV_ROOT_DIR} STREQUAL "")
  set(OPENCV_ROOT_DIR $ENV{OPENCV_ROOT_DIR})
endif()

set(_opencv_SEARCH_DIRS
  ${OPENCV_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/opencv
)

find_path(OPENCV_INCLUDE_DIR
  NAMES
    opencv2/core.hpp
  HINTS
    ${_opencv_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/include
  PATH_SUFFIXES
    include
)

unset(OPENCV_LIBRARIES)
unset(OPENCV_DEPS)
find_library(OPENCV_LIBRARY
  NAMES
    cv opencv opencv_world341 opencv_world340 opencv_world330 opencv_world320
  HINTS
    ${_opencv_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/lib
  PATH_SUFFIXES
    lib64 lib
)
set(OPENCV_LIBRARIES ${OPENCV_LIBRARY})

set (OPENCV_DEP_NAMES "IlmImf;ittnotify;libjpeg;libprotobuf;libwebp;ippiw;ippicvmt;libjasper;libpng;libtiff;zlib")
foreach (dep ${OPENCV_DEP_NAMES})
    unset (CV_DEP_${dep}_LIBRARY)
    find_library(CV_DEP_${dep}_LIBRARY
      NAMES ${dep}
      HINTS ${ARK_DEPENDENCY_DIR}/lib ${_opencv_SEARCH_DIRS}
      PATH_SUFFIXES lib64 lib
    )
    list (APPEND OPENCV_DEPS "CV_DEP_${dep}_LIBRARY")
    list (APPEND OPENCV_LIBRARIES "${CV_DEP_${dep}_LIBRARY}")
    mark_as_advanced (CV_DEP_${dep}_LIBRARY)
endforeach (dep)

# handle the QUIETLY and REQUIRED arguments and set OPENCV_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OpenCV DEFAULT_MSG
    OPENCV_LIBRARY ${OPENCV_DEPS} OPENCV_INCLUDE_DIR)

IF(OPENCV_FOUND)
  set(OPENCV_INCLUDE_DIRS ${OPENCV_INCLUDE_DIR})
ENDIF(OPENCV_FOUND)

mark_as_advanced(
  OPENCV_INCLUDE_DIR
  OPENCV_LIBRARY
  OPENCV_DEP_NAMES
  OPENCV_DEPS
)
