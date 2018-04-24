cmake_minimum_required( VERSION 2.8 )

project( OpenARK )
set( OpenARK_VERSION_MAJOR 0 )
set( OpenARK_VERSION_MINOR 9 )
set( OpenARK_VERSION_PATCH 3 )
set( SVM_PATHS "\"svm/\", \"../../svm/\", \"../svm/\", \"\"" )
set( INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include" )

set( CMAKE_CXX_STACK_SIZE "10000000" )
set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" )

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release )
endif()

set( LIBRARY_NAME "OpenARK" )
set( TEST_NAME "OpenARK_test" )
set( UNITY_PLUGIN_NAME "UnityPlugin" )

set( HAND_DEMO_NAME "Hand_demo" )
set( SLAM_DEMO_NAME "SLAM_demo" )

set ( REQUIRED_BOOST_COMPONENTS "system;filesystem;thread;date_time;iostreams;serialization;chrono;regex" )

option( BUILD_DEMOS "BUILD_DEMOS" ON )
option( BUILD_TESTS "BUILD_TESTS" OFF )
option( BUILD_UNITY_PLUGIN "BUILD_UNITY_PLUGIN" ON ) # Windows only
option( USE_REALSENSE2 "USE_REALSENSE2" ON ) 
option( USE_REALSENSE "USE_REALSENSE" OFF ) 
option( USE_RSSDK "USE_RSSDK" OFF ) 
option( USE_PMDSDK "USE_PMDSDK" OFF ) 

include( CheckCXXCompilerFlag )
CHECK_CXX_COMPILER_FLAG( "-std=c++11" COMPILER_SUPPORTS_CXX11 )
CHECK_CXX_COMPILER_FLAG( "-std=c++0x" COMPILER_SUPPORTS_CXX0X )

if( COMPILER_SUPPORTS_CXX11 )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
elseif( COMPILER_SUPPORTS_CXX0X )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
else()
    message( STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler." )
endif()

if ( CMAKE_COMPILER_IS_GNUCXX )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wno-deprecated-declarations -O3 -g" )
endif ( CMAKE_COMPILER_IS_GNUCXX )

if( ARK_DEPENDENCY_DIR )
    message(STATUS "Using OpenARK Dependency Directory: ${ARK_DEPENDENCY_DIR}")
    # use included partial boost
    set(BOOST_LIBRARYDIR "${ARK_DEPENDENCY_DIR}/lib")
    set(BOOST_INCLUDEDIR "${ARK_DEPENDENCY_DIR}/include")
    set(Boost_NO_SYSTEM_PATHS  ON)
endif( ARK_DEPENDENCY_DIR )

# find Boost
if(WIN32)
    # use static Boost in Windows
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC ON)
    set(Boost_USE_MULTITHREAD )
endif(WIN32)

set(Boost_NO_BOOST_CMAKE ON)
find_package(Boost 1.40.0 ${QUIET_} REQUIRED COMPONENTS ${REQUIRED_BOOST_COMPONENTS})

set(BOOST_FOUND ${Boost_FOUND})
set(BOOST_INCLUDE_DIRS "${Boost_INCLUDE_DIR}")
set(BOOST_LIBRARY_DIRS "${Boost_LIBRARY_DIRS}")
set(BOOST_LIBRARIES ${Boost_LIBRARIES})
if(WIN32 AND NOT MINGW)
set(BOOST_DEFINITIONS ${BOOST_DEFINITIONS} -DBOOST_ALL_NO_LIB)
endif(WIN32 AND NOT MINGW)

if(BOOST_FOUND)
   message(STATUS "Found Boost Libraries: ${BOOST_LIBRARIES}")
endif(BOOST_FOUND)

# require VTK
find_package( VTK REQUIRED )
if(VTK_FOUND)
   message(STATUS "Found VTK Libraries: ${VTK_LIBRARIES}")
endif(VTK_FOUND)

# require PCL
find_package( PCL REQUIRED )
if(PCL_FOUND)
   message(STATUS "Found PCL Libraries: ${PCL_LIBRARIES}")
endif(PCL_FOUND)

# require OpenCV
find_package( OpenCV 3.2 REQUIRED )
if(OpenCV_FOUND)
   message(STATUS "Found OpenCV: ${OPENCV_INCLUDE_DIRS} ${OPENCV_LIBRARIES}")
endif(OpenCV_FOUND)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR}) 
if(EIGEN_FOUND)
   message(STATUS "Found Eigen: ${EIGEN_INCLUDE_DIRS}")
endif(EIGEN_FOUND)

# require FLANN
find_package( Flann REQUIRED )
include_directories(${FLANN_INCLUDE_DIR}) 
if(FLANN_FOUND)
   message(STATUS "Found FLANN: ${FLANN_INCLUDE_DIRS} ${FLANN_LIBRARIES}")
endif(FLANN_FOUND)

# require Ceres
find_package( Ceres REQUIRED )
IF(Ceres_FOUND)
   MESSAGE(STATUS "Found Ceres: ${CERES_INCLUDE_DIRS}")
ENDIF(Ceres_FOUND)
include_directories(${CERES_INCLUDE_DIRS})

# require Okvis
find_package( Okvis 1.1 REQUIRED )
include_directories(${OKVIS_INCLUDE_DIRS})
message(STATUS "Found Okvis: ${OKVIS_INCLUDE_DIRS}")

# require glog (should come with Okvis)
find_package( Glog REQUIRED )
message(STATUS "Found Glog: ${GLOG_LIBRARIES}")

# require OpenGV (should come with Okvis)
find_package( OpenGV REQUIRED )
message(STATUS "Found OpenGV: ${OPENGV_INCLUDE_DIRS}")

# require Brisk
find_package( Brisk 2 REQUIRED )
include_directories(${BRISK_INCLUDE_DIRS})
message(STATUS "Found Brisk: ${BRISK_INCLUDE_DIRS}")

# require DBoW2
find_package( DBoW2 REQUIRED )
IF( DBoW2_FOUND )
   MESSAGE( STATUS "Found DBoW2: ${DBoW2_INCLUDE_DIRS}" )
ENDIF( DBoW2_FOUND )

# require DLib
find_package( DLib REQUIRED )
IF( DLib_FOUND )
   MESSAGE( STATUS "Found DLib: ${DLib_INCLUDE_DIRS} ${DLib_LIBRARIES}" )
ENDIF( DLib_FOUND )

# Intel RealSense SDK 2
if( USE_REALSENSE2 )
  find_package( RealSense2 )
  if( REALSENSE2_FOUND )
      message( STATUS "Found librealsense2: ${REALSENSE2_INCLUDE_DIRS}" )
  endif( REALSENSE2_FOUND )
endif( USE_REALSENSE2 )

# Intel RealSense SDK 1
if( USE_REALSENSE OR NOT REALSENSE2_FOUND )
  find_package( RealSense )
  if( REALSENSE_FOUND )
      message( STATUS "Found librealsense: ${REALSENSE_INCLUDE_DIRS}" )
  endif( REALSENSE_FOUND )
endif( USE_REALSENSE OR NOT REALSENSE2_FOUND )

# Windows RealSense SDK
set(RSSDK_FOUND OFF)
if( USE_RSSDK OR ( NOT REALSENSE_FOUND AND NOT REALSENSE2_FOUND ) )
  find_package( RSSDK )
  if( RSSDK_FOUND )
       message( STATUS "Found Windows RSSDK: ${RSSDK_INCLUDE_DIR}" )
  endif( RSSDK_FOUND )
endif( USE_RSSDK OR ( NOT REALSENSE_FOUND AND NOT REALSENSE2_FOUND ) )

# PMD SDK
if( USE_PMDSDK OR ( NOT RSSDK_FOUND AND NOT REALSENSE_FOUND AND NOT REALSENSE2_FOUND ) )
  find_package( PMDSDK REQUIRED )
  if( PMDSDK_FOUND )
     message( STATUS "Found PMDSDK: ${PMDSDK_INCLUDE_DIR}" )
  endif( PMDSDK_FOUND )
endif( USE_PMDSDK OR ( NOT RSSDK_FOUND AND NOT REALSENSE_FOUND AND NOT REALSENSE2_FOUND ) )

include_directories(
  ${BOOST_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${DBoW2_INCLUDE_DIRS}
  ${DLib_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

set(
  DEPENDENCIES  
  ${BOOST_LIBRARIES}
  ${OPENCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${REALSENSE_LIBRARIES} 
  ${REALSENSE2_LIBRARIES} 
  ${OKVIS_LIBRARIES}
  ${DBoW2_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${BRISK_LIBRARIES}
  ${DLib_LIBRARIES}
  ${OPENGV_LIBRARIES}
  ${CERES_LIBRARIES}
  ${FLANN_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_definitions(
  ${PCL_DEFINITIONS}
  ${BOOST_DEFINITIONS}
)
foreach(DEFINITION ${PCL_DEFINITIONS})
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

set(
  SOURCES 
  OkvisSLAMSystem.cpp
  FrameObject.cpp 
  Hand.cpp FramePlane.cpp 
  Webcam.cpp 
  DepthCamera.cpp 
  RGBCamera.cpp
  StreamingAverager.cpp 
  Calibration.cpp 
  Util.cpp	
  Visualizer.cpp	
  HandClassifier.cpp
  Detector.cpp
  HandDetector.cpp
  PlaneDetector.cpp
)

set(
  HEADERS 
  ${INCLUDE_DIR}/OkvisSLAMSystem.h
  ${INCLUDE_DIR}/SLAMSystem.h
  ${INCLUDE_DIR}/FrameObject.h
  ${INCLUDE_DIR}/DetectionParams.h
  ${INCLUDE_DIR}/Hand.h ${INCLUDE_DIR}/FramePlane.h
  ${INCLUDE_DIR}/Webcam.h
  ${INCLUDE_DIR}/DepthCamera.h 
  ${INCLUDE_DIR}/RGBCamera.h
  ${INCLUDE_DIR}/StreamingAverager.h 
  ${INCLUDE_DIR}/Calibration.h 
  ${INCLUDE_DIR}/Util.h	
  ${INCLUDE_DIR}/Visualizer.h	
  ${INCLUDE_DIR}/HandClassifier.h
  ${INCLUDE_DIR}/SingleConsumerPriorityQueue.h
  ${INCLUDE_DIR}/Detector.h
  ${INCLUDE_DIR}/HandDetector.h
  ${INCLUDE_DIR}/Types.h
  ${INCLUDE_DIR}/PlaneDetector.h
)

if( NOT REALSENSE2_FOUND )
  set( _REALSENSE2_ "//" )
else()
  include_directories(${REALSENSE2_INCLUDE_DIRS}) 
  set( SOURCES ${SOURCES} RS2Camera.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/RS2Camera.h )
  set( DEPENDENCIES ${DEPENDENCIES} ${REALSENSE2_LIBRARIES} )
endif( NOT REALSENSE2_FOUND )

if( NOT REALSENSE_FOUND )
  set( _REALSENSE_ "//" )
else()
  include_directories(${REALSENSE_INCLUDE_DIRS}) 
  set( SOURCES ${SOURCES} ZR300Camera.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/ZR300Camera.h )
  set( DEPENDENCIES ${DEPENDENCIES} ${REALSENSE_LIBRARIES} )
endif( NOT REALSENSE_FOUND )

if( NOT RSSDK_FOUND )
  set( _RSSDK_ "//" )
else()
  include_directories( ${RSSDK_INCLUDE_DIR} )
  set( SOURCES ${SOURCES} SR300Camera.cpp Converter.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/SR300Camera.h ${INCLUDE_DIR}/Converter.h )
endif( NOT RSSDK_FOUND )

if( NOT PMDSDK_FOUND )
  set( _PMDSDK_ "//" )
else()
  include_directories( ${PMDSDK_INCLUDE_DIR} )
  set( SOURCES ${SOURCES} PMDCamera.cpp )
  set( HEADERS ${HEADERS} ${INCLUDE_DIR}/PMDCamera.h )
  set( DEPENDENCIES ${DEPENDENCIES} ${PMDSDK_LIBRARIES} )
endif( NOT PMDSDK_FOUND )

if ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )
  set( _DEBUG_ "//" )
endif ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )

configure_file(Version.h.in ${INCLUDE_DIR}/Version.h)
set(HEADERS ${HEADERS} ${INCLUDE_DIR}/Version.h)

set(LIB_OUTPUT_NAME "openark_${OpenARK_VERSION_MAJOR}_${OpenARK_VERSION_MINOR}_${OpenARK_VERSION_PATCH}")

if (MSVC)
    # MSVC precompiled header
    set( PRECOMPILED_BINARY "$(IntDir)/stdafx.pch" )
    set( PRECOMPILED_HEADER "stdafx.h" )
    set( PRECOMPILED_SOURCE "stdafx.cpp" )
    
    set_source_files_properties( ${PRECOMPILED_SOURCE}
                                PROPERTIES COMPILE_FLAGS "/Yc\"${PRECOMPILED_HEADER}\" /Fp\"${PRECOMPILED_BINARY}\""
                                           OBJECT_OUTPUTS "${PRECOMPILED_HEADER}" )
    set_source_files_properties( ${SOURCES}
                                PROPERTIES COMPILE_FLAGS "/Yu\"${PRECOMPILED_HEADER}\" /FI\"${PRECOMPILED_HEADER}\" /Fp\"${PRECOMPILED_BINARY}\""
                                           OBJECT_DEPENDS "${PRECOMPILED_HEADER}" )  
    set( SOURCES ${SOURCES} ${PRECOMPILED_SOURCE} )
    
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /Ot /Zm997" )
    set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:LIBCMT /DEBUG" )
    set( CMAKE_SHARED_LINKER_FLAGS  "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:LIBCMT /DEBUG" )
    set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi" )
    set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi" )
    if( ARK_DEPENDENCY_DIR )
        list( APPEND DEPENDENCIES ${ARK_DEPENDENCY_DIR}/lib/Vfw32.lib )
    else ( ARK_DEPENDENCY_DIR )
        list( APPEND DEPENDENCIES Vfw32.lib )
    endif( ARK_DEPENDENCY_DIR )
endif (MSVC)

add_library(${LIBRARY_NAME} STATIC "${INCLUDE_DIR}/Core.h" "stdafx.h" ${SOURCES} ${HEADERS})
set_target_properties( ${LIBRARY_NAME} PROPERTIES OUTPUT_NAME ${LIB_OUTPUT_NAME})
set_property(TARGET ${LIBRARY_NAME} PROPERTY DBG_POSTFIX d)
target_link_libraries( ${LIBRARY_NAME} ${DEPENDENCIES} )
target_include_directories( ${LIBRARY_NAME} PRIVATE ${INCLUDE_DIR} )

if(${BUILD_DEMOS})
    add_executable( ${HAND_DEMO_NAME} main.cpp )
    target_link_libraries( ${HAND_DEMO_NAME} ${DEPENDENCIES} ${LIBRARY_NAME} )
    set_target_properties( ${HAND_DEMO_NAME} PROPERTIES OUTPUT_NAME ${HAND_DEMO_NAME})
    set_target_properties( ${HAND_DEMO_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
    target_include_directories( ${HAND_DEMO_NAME} PRIVATE ${INCLUDE_DIR} )
    if( REALSENSE_FOUND )
        add_executable( ${SLAM_DEMO_NAME} SlamDemo.cpp )
        target_link_libraries( ${SLAM_DEMO_NAME} ${DEPENDENCIES} ${LIBRARY_NAME} )
        set_target_properties( ${SLAM_DEMO_NAME} PROPERTIES OUTPUT_NAME ${SLAM_DEMO_NAME})
        set_target_properties( ${SLAM_DEMO_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
        target_include_directories( ${SLAM_DEMO_NAME} PRIVATE ${INCLUDE_DIR} )
        set_target_properties( ${SLAM_DEMO_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
    endif( REALSENSE_FOUND )
endif(${BUILD_DEMOS})

# Unity plugin currently only supports Windows
if( ${BUILD_UNITY_PLUGIN} AND MSVC )
    add_library( ${UNITY_PLUGIN_NAME} SHARED "unity/native/UnityInterface.cpp" "unity/native/UnityInterface.h" "unity/README.md" )
    set( HEADERS ${HEADERS} "unity/native/UnityInterface.h" )
    target_link_libraries( ${UNITY_PLUGIN_NAME} ${DEPENDENCIES} ${LIBRARY_NAME} )
    set_property( TARGET ${UNITY_PLUGIN_NAME} PROPERTY DBG_POSTFIX d )
    target_include_directories( ${UNITY_PLUGIN_NAME} PRIVATE ${INCLUDE_DIR} "unity/native" )
    set_target_properties( ${UNITY_PLUGIN_NAME} PROPERTIES OUTPUT_NAME "openark_unity_${OpenARK_VERSION_MAJOR}_${OpenARK_VERSION_MINOR}_${OpenARK_VERSION_PATCH}_native" )
endif( ${BUILD_UNITY_PLUGIN} AND MSVC )

if( ${BUILD_TESTS} )
  if( NOT EXISTS ${PROJECT_SOURCE_DIR}/OpenARK_test )
    execute_process(
      COMMAND git clone https://github.com/augcog/OpenARK_test.git
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} )
  endif( NOT EXISTS ${PROJECT_SOURCE_DIR}/OpenARK_test )

  add_executable( ${TEST_NAME} "OpenARK_test/test.cpp" "OpenARK_test/TestCamera.cpp" "OpenARK_test/TestCamera.h" )
  set( HEADERS ${HEADERS} "OpenARK_test/TestCamera.h" )
  
  target_link_libraries( ${TEST_NAME} ${DEPENDENCIES} ${LIBRARY_NAME} )
  set_target_properties( ${TEST_NAME} PROPERTIES
    COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} )
    
  target_include_directories( ${TEST_NAME} PRIVATE ${INCLUDE_DIR} )
endif( ${BUILD_TESTS} )

# Create source group for headers
source_group( "Header Files" FILES ${HEADERS} )
