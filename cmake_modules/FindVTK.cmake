# - Find VTK library
# Find the native VTK includes and library
# This module defines
#  VTK_INCLUDE_DIRS, where to find VTK headers, Set when
#                      VTK_INCLUDE_DIR is found.
#  VTK_LIBRARIES, libraries to link against to use VTK.
#  VTK_COMPONENTS names of VTK components found
#  VTK_FOUND, If false, do not try to use VTK.
#
# also defined, but not for general use are
#  VTK_LIBRARY, where to find the VTK library.

# If VTK_ROOT_DIR was defined in the environment, use it.
IF(NOT VTK_ROOT_DIR AND NOT $ENV{VTK_ROOT_DIR} STREQUAL "")
  SET(VTK_ROOT_DIR $ENV{VTK_ROOT_DIR})
ENDIF()

SET(_vtk_SEARCH_DIRS
  ${VTK_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/vtk
)

unset(VTK_INCLUDE_DIRS)
unset(VTK_LIBRARIES)
unset(VTK_COMPONENTS)

FIND_PATH(VTK_INCLUDE_DIR
  NAMES
    vtkSmartPointer.h
  HINTS
    ${_vtk_SEARCH_DIRS}
  PATHS
    ${ARK_DEPENDENCY_DIR}/include/vtk ${ARK_DEPENDENCY_DIR}/include
  PATH_SUFFIXES
    include
)

set(VTK_COMPONENT_NAMES "vtkalglib;vtkChartsCore;vtkCommonColor;vtkCommonDataModel;vtkCommonMath;vtkCommonCore;vtksys;vtkCommonMisc;vtkCommonSystem;vtkCommonTransforms;vtkInfovisCore;vtkFiltersExtraction;vtkCommonExecutionModel;vtkFiltersCore;vtkFiltersGeneral;vtkCommonComputationalGeometry;vtkFiltersStatistics;vtkImagingFourier;vtkImagingCore;vtkRenderingContext2D;vtkRenderingCore;vtkFiltersGeometry;vtkFiltersSources;vtkRenderingFreeType;vtkfreetype;vtkzlib;vtkDICOMParser;vtkDomainsChemistry;vtkIOXML;vtkIOGeometry;vtkIOCore;vtkIOXMLParser;vtkexpat;vtkexoIIc;vtkNetCDF;vtkNetCDF_cxx;vtkhdf5_hl;vtkhdf5;vtkFiltersAMR;vtkParallelCore;vtkIOLegacy;vtkFiltersFlowPaths;vtkFiltersGeneric;vtkFiltersHybrid;vtkImagingSources;vtkFiltersHyperTree;vtkFiltersImaging;vtkImagingGeneral;vtkFiltersModeling;vtkFiltersParallel;vtkFiltersParallelImaging;vtkFiltersProgrammable;vtkFiltersSelection;vtkFiltersSMP;vtkFiltersTexture;vtkFiltersVerdict;verdict;vtkGeovisCore;vtkInfovisLayout;vtkImagingHybrid;vtkIOImage;vtkmetaio;vtkjpeg;vtkpng;vtktiff;vtkInteractionStyle;vtkInteractionWidgets;vtkRenderingAnnotation;vtkImagingColor;vtkRenderingVolume;vtkViewsCore;vtkproj4;vtkgl2ps;vtkImagingMath;vtkImagingMorphological;vtkImagingStatistics;vtkImagingStencil;vtkInteractionImage;vtkIOAMR;vtkIOEnSight;vtkIOExodus;vtkIOExport;vtkRenderingGL2PS;vtkRenderingContextOpenGL;vtkRenderingOpenGL;vtkRenderingLabel;vtkIOImport;vtkIOInfovis;vtklibxml2;vtkIOLSDyna;vtkIOMINC;vtkIOMovie;vtkoggtheora;vtkIONetCDF;vtkIOParallel;vtkjsoncpp;vtkIOParallelXML;vtkIOPLY;vtkIOSQL;vtksqlite;vtkIOVideo;vtkRenderingImage;vtkRenderingLIC;vtkRenderingLOD;vtkRenderingVolumeOpenGL;vtkViewsContext2D;vtkViewsInfovis")

foreach(component ${VTK_COMPONENT_NAMES})
    find_library(VTK_${component}_LIBRARY
      NAMES
        ${component} ${component}-7.0 vtk${component} vtk${component}-7.0
      HINTS
        ${_vtk_SEARCH_DIRS}
      PATHS
        ${ARK_DEPENDENCY_DIR}/lib
      PATH_SUFFIXES
        lib64 lib
    )
    list (APPEND VTK_COMPONENTS "VTK_${component}_LIBRARY")
    mark_as_advanced(VTK_${component}_LIBRARY)
    list (APPEND VTK_LIBRARIES "${VTK_${component}_LIBRARY}")
endforeach(component)

# handle the QUIETLY and REQUIRED arguments and set VTK_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(VTK DEFAULT_MSG
    ${VTK_COMPONENTS} VTK_LIBRARIES VTK_INCLUDE_DIR)

IF(VTK_FOUND)
  SET(VTK_INCLUDE_DIRS ${VTK_INCLUDE_DIR})
ENDIF(VTK_FOUND)

MARK_AS_ADVANCED(
  VTK_INCLUDE_DIR
  VTK_LIBRARY
  VTK_COMPONENT_NAMES
)
