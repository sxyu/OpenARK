# - Find PCL
# Find the native Point Cloud Library includes and library
# This module defines
#  PCL_INCLUDE_DIRS, where to find pcl/ headers
#  PCL_LIBRARIES, libraries to link against to use PCL.
#  PCL_COMPONENTS, names of available components (upper case, PCL_XYZ_LIBRARY)
#  PCL_FOUND, If false, do not try to use PCL.
#
# PCL Libraries we look for:
# - common
# - octree
# - io
# - io_ply (optional)
# - kdtree
# - search
# - sample_consensus
# - filters
# - 2d (optional)
# - geometry (optional)
# - features
# - ml
# - segmentation
# - visualization
# - surface
# - registration
# - keypoints
# - tracking
# - recognition
# - stereo
# - outofcore
# - people
#
# If PCL_ROOT_DIR was defined in the environment, use it.
IF(NOT PCL_ROOT_DIR AND NOT $ENV{PCL_ROOT_DIR} STREQUAL "")
  SET(PCL_ROOT_DIR $ENV{PCL_ROOT_DIR})
ENDIF()

unset(PCL_INCLUDE_DIRS)
unset(PCL_COMPONENTS)
unset(PCL_LIBRARIES)

SET(_pcl_SEARCH_DIRS
  ${PCL_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/pcl
)

FIND_PATH(PCL_INCLUDE_DIR
  NAMES
    pcl/pcl_base.h
  HINTS
    ${_pcl_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/include
  PATH_SUFFIXES
    include
)

macro(find_pcl_comp COMP_NAME REQUIRE)
    string(TOUPPER ${COMP_NAME} COMP_NAME_UPPER)
    string(TOLOWER ${COMP_NAME} COMP_NAME_LOWER)
    find_library(PCL_${COMP_NAME_UPPER}_LIBRARY
      NAMES pcl_${COMP_NAME_LOWER}_release
      HINTS ${_pcl_SEARCH_DIRS}
      PATHS ${ARK_DEPENDENCY_DIR}/lib
      PATH_SUFFIXES lib64 lib
    )
    
    if (REQUIRE OR PCL_${COMP_NAME_UPPER}_LIBRARY)
        list(APPEND PCL_COMPONENTS "PCL_${COMP_NAME_UPPER}_LIBRARY")
    endif (REQUIRE OR PCL_${COMP_NAME_UPPER}_LIBRARY)
    
    if (PCL_${COMP_NAME_UPPER}_LIBRARY)
        list(APPEND PCL_LIBRARIES "${PCL_${COMP_NAME_UPPER}_LIBRARY}")
    endif (PCL_${COMP_NAME_UPPER}_LIBRARY)
    mark_as_advanced(PCL_${COMP_NAME_UPPER}_LIBRARY)
endmacro(find_pcl_comp)

find_pcl_comp(common ON)
find_pcl_comp(octree ON)
find_pcl_comp(io ON)
find_pcl_comp(io_ply OFF)
find_pcl_comp(kdtree ON)
find_pcl_comp(search ON)
find_pcl_comp(sample_consensus ON)
find_pcl_comp(filters ON)
find_pcl_comp(2d OFF)
find_pcl_comp(geometry OFF)
find_pcl_comp(features ON)
find_pcl_comp(ml ON)
find_pcl_comp(segmentation ON)
find_pcl_comp(visualization ON)
find_pcl_comp(surface ON)
find_pcl_comp(registration ON)
find_pcl_comp(keypoints ON)
find_pcl_comp(tracking ON)
find_pcl_comp(recognition ON)
find_pcl_comp(stereo ON)
find_pcl_comp(outofcore ON)
find_pcl_comp(people ON)

# handle the QUIETLY and REQUIRED arguments and set PCL_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PCL DEFAULT_MSG ${PCL_LIBRARY_NAMES} PCL_INCLUDE_DIR)

IF(PCL_FOUND)
  set(PCL_DEFINITIONS "-DEIGEN_USE_NEW_STDVECTOR;-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET;-DBOOST_ALL_NO_LIB;-DFLANN_STATIC")
  set(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIR})
ENDIF(PCL_FOUND)

unset(COMP_NAME_LOWER)
unset(COMP_NAME_UPPER)

mark_as_advanced(
  PCL_INCLUDE_DIR
  PCL_COMPONENTS
)
