cmake_minimum_required(VERSION 3.14)

project(SCFusion_Proejct CXX C)

#set (CMAKE_CXX_STANDARD 11)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
#set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG  -O3 -Wall")

if(UNIX)
  SET(CMAKE_MODULE_LINKER_FLAGS ${CMAKE_MODULE_LINKER_FLAGS} "-ldl")
endif()

OPTION(BUILD_DATA_GENERATOR "Build scannet data generator" OFF)
OPTION(WITH_GUI "BUILD GUI" OFF)

include(CMakeDependentOption)

IF(BUILD_DATA_GENERATOR)
  SET(WITH_GUI TRUE)
ENDIF(BUILD_DATA_GENERATOR)

MESSAGE("WITH_GUI: " ${WITH_GUI})

#SET(PYTORCH_PATH "/home/sc/installs/pytorch/torch" CACHE STRING "path to pytorchlib")
#################################
# Specify the CMake module path #
#################################
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

#################################
# Add additional compiler flags #
#################################
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
ENDIF()

IF(NOT MSVC_IDE)
  SET(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "-fPIC -O0 ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}") # remove -march=native. this will cause error in PCL with EIGEN. See https://github.com/PointCloudLibrary/pcl/issues/1791
ENDIF()

####
# External
####
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCxxTools.cmake)
INCLUDE_DIRECTORIES(${CxxTools_INCLUDE_DIRS})

######################################
# Define Voxel/Surfel types to build #
######################################
LIST(APPEND VOXELNAMES
        OFusion2Label # for data generation
        OFusionRGB1Label)
LIST(APPEND VOXELTYPES
        OFu_Voxel_f_2label
        OFu_Voxel_f_rgb_1label
        )
LIST(LENGTH VOXELNAMES VOXELCOUNTS)
MATH(EXPR VOXELCOUNTS "${VOXELCOUNTS}-1")

#LIST(APPEND SURFELNAMES
#        SurfelGray
#        SurfelRGB
#        SurfelRGB1Label
#        )
#LIST(APPEND SURFELTYPES
#        ITMSurfel_grey
#        ITMSurfel_rgb
#        Surfel_rgb_1label
#        )
#LIST(LENGTH SURFELNAMES SURFELCOUNTS)
#MATH(EXPR SURFELCOUNTS "${SURFELCOUNTS}-1")
SET(SURFELCOUNTS 0)

######################
# Add subdirectories #
######################
IF(WITH_PYTORCH)
  INCLUDE(UsePytorch)
  ADD_SUBDIRECTORY(PytorchCpp)
  SET(CUDA_NVCC_FLAGS)
ENDIF(WITH_PYTORCH)

IF(WITH_GUI)
  ADD_SUBDIRECTORY(libGUI3D/libGUI)
  ADD_SUBDIRECTORY(libGUI3D/libGUI3D)
ENDIF(WITH_GUI)

IF(BUILD_DATA_GENERATOR)
  ADD_SUBDIRECTORY(MeshVoxelizer)
  ADD_SUBDIRECTORY(ConnectedComponent)
  ADD_SUBDIRECTORY(cnpy)
ENDIF(BUILD_DATA_GENERATOR)

ADD_SUBDIRECTORY(ORUtils)
ADD_SUBDIRECTORY(ImageLoader)
ADD_SUBDIRECTORY(FernRelocLib)
ADD_SUBDIRECTORY(DenseCRF)
ADD_SUBDIRECTORY(MainLib)
ADD_SUBDIRECTORY(SLAMWrapper)
ADD_SUBDIRECTORY(App)
