cmake_minimum_required(VERSION 3.16)
project(lsd_slam_core LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# When building inside a pixi/conda environment, make sure find_package()/find_path()
# prefer it over unrelated system installs (e.g. Homebrew) that may also be present.
if(DEFINED ENV{CONDA_PREFIX})
  list(PREPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
endif()

# Use the CMake config shipped by the pixi/conda Eigen package.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Sophus REQUIRED CONFIG)
find_package(CHOLMOD CONFIG REQUIRED)
find_package(CXSparse CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(Threads REQUIRED)

# OpenFABMAP does not provide a modern CMake package. Fetch its stable 2.04
# release and build the library sources as part of this project.
include(FetchContent)
FetchContent_Declare(
  openfabmap_external
  GIT_REPOSITORY https://github.com/arrenglover/openfabmap.git
  GIT_TAG 2.04
  GIT_SHALLOW TRUE
)
FetchContent_Populate(openfabmap_external)

add_library(openFABMAP STATIC
  ${openfabmap_external_SOURCE_DIR}/src/bowmsctrainer.cpp
  ${openfabmap_external_SOURCE_DIR}/src/chowliutree.cpp
  ${openfabmap_external_SOURCE_DIR}/src/fabmap.cpp
  ${openfabmap_external_SOURCE_DIR}/src/inference.cpp
)
target_include_directories(openFABMAP PUBLIC
  ${PROJECT_SOURCE_DIR}/third_party/opencv_compat
  ${openfabmap_external_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(openFABMAP PUBLIC ${OpenCV_LIBS})

# SSE flags (x86 only)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i.86|AMD64)$")
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE42)
  if(COMPILER_SUPPORTS_SSE42)
    set(SSE_FLAGS "-msse4.2")
    add_definitions("-DENABLE_SSE")
  endif()
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SSE_FLAGS}")

# ---------------------------------------------------------------------------
# g2o: prefer a system/conda install; fall back to fetching+building an old
# release (pre types/sim3 API changes) that matches the API used by this codebase.
# ---------------------------------------------------------------------------
find_package(g2o QUIET)

if(NOT g2o_FOUND AND NOT TARGET g2o::core)
  message(STATUS "system g2o not found - fetching g2o (20200410_git) as a subproject")
  include(FetchContent)

  set(G2O_BUILD_APPS OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(G2O_USE_OPENGL OFF CACHE BOOL "" FORCE)
  set(G2O_USE_OPENMP OFF CACHE BOOL "" FORCE)
  set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "" FORCE)
  set(BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)

  # g2o's manual (non-Linux) SSE detection defaults to "enabled" unless each
  # extension is explicitly disabled; on non-x86 targets (e.g. Apple Silicon)
  # none of these instruction sets exist, so force them off.
  if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i.86|AMD64)$")
    set(DISABLE_SSE2 ON CACHE BOOL "" FORCE)
    set(DISABLE_SSE3 ON CACHE BOOL "" FORCE)
    set(DISABLE_SSE4_1 ON CACHE BOOL "" FORCE)
    set(DISABLE_SSE4_2 ON CACHE BOOL "" FORCE)
    set(DISABLE_SSE4_A ON CACHE BOOL "" FORCE)
  endif()

  FetchContent_Declare(
    g2o_external
    GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
    GIT_TAG 20200410_git
    GIT_SHALLOW TRUE
  )
  # g2o's own CMakeLists.txt at this old tag predates CMake's minimum supported
  # policy version; tell CMake to accept it rather than erroring out.
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  FetchContent_MakeAvailable(g2o_external)

  set(G2O_INCLUDE_DIR ${g2o_external_SOURCE_DIR} ${g2o_external_BINARY_DIR})
  # note: the CMake *target* names in this g2o tag are unprefixed (e.g. "core"),
  # even though their installed/output library file names carry a "g2o_" prefix.
  set(G2O_LIBRARIES core stuff types_sim3 types_sba
                     solver_csparse solver_pcg csparse_extension)

  # On Apple, g2o's own CMakeLists.txt sets CMAKE_BUILD_WITH_INSTALL_RPATH TRUE,
  # which bakes the (never-run) "make install" path (e.g. /usr/local/lib) into
  # each dylib's install name. Since we only ever use the build tree, reset this
  # so the dylibs reference their actual build location instead.
  if(APPLE)
    foreach(_g2o_target IN ITEMS core stuff types_sim3 types_sba
                                 solver_csparse solver_pcg csparse_extension
                                 types_slam3d types_slam2d types_data types_icp
                                 types_sclam2d types_slam2d_addons types_slam3d_addons
                                 solver_cholmod solver_dense solver_eigen
                                 solver_slam2d_linear solver_structure_only)
      if(TARGET ${_g2o_target})
        set_target_properties(${_g2o_target} PROPERTIES
          BUILD_WITH_INSTALL_RPATH FALSE
          INSTALL_NAME_DIR "")
      endif()
    endforeach()
  endif()
else()
  message(STATUS "found system g2o")
endif()

# Set source files (ROS-free core library)
set(lsd_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FramePoseStruct.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FrameMemory.cpp
  ${PROJECT_SOURCE_DIR}/src/SlamSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/LiveSLAMWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMap.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalFuncs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/SophusUtil.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistorter.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Sim3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Relocalizer.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/SE3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/FabMap.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/KeyFrameGraph.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/TrackableKeyFrameSearch.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${openfabmap_external_SOURCE_DIR}/include
)

# build core (ROS-free) shared library.
add_library(lsdslam SHARED ${lsd_SOURCE_FILES})
target_compile_definitions(lsdslam PRIVATE HAVE_FABMAP)
target_link_libraries(lsdslam
  Eigen3::Eigen
  Sophus::Sophus
  SuiteSparse::CHOLMOD
  SuiteSparse::CXSparse
  ${OpenCV_LIBS}
  openFABMAP
  ${G2O_LIBRARIES}
  Boost::thread
  Boost::system
  Threads::Threads
)

# standalone, non-ROS executable for running LSD-SLAM on a folder/list of images.
add_executable(dataset_slam src/main_on_images.cpp)
target_link_libraries(dataset_slam lsdslam)

