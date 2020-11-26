###############################################################################
### -- 3rd party
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
INCLUDE(UseCUDA)
INCLUDE(UseFreetype)
INCLUDE(UseEigen3)

######################################### ScanNet + Scan2CAD ##########################################
## GT Generator
FIND_PACKAGE(ASSIMP REQUIRED)
set(targetname exe_GroundtruthGenerator_ScanNetScan2CAD)
file(GLOB srcs exe_GroundtruthGenerator_ScanNetScan2CAD.cpp
		ScanNetMeshLoader.cpp
		ScanNetScan2CADLoader.cpp
		Scan2CADObjectLoader.cpp
		assimp_obj_loader.cpp
		third_party/json11.cpp)
file(GLOB headers TrainingDataGenerator.h)
file(GLOB cuda TrainingDataGenerator.cu TrainingDataGeneratorInstantiation.cu)
SET(sources
		${srcs}
		${cuda}
		${glad}
		)
SET(headers
		${headers}
		)

ADD_DEFINITIONS(-DPCL_ENABLE_SSE=OFF)
INCLUDE(SetCUDAAppTarget)
target_link_libraries(${targetname}
#		PUBLIC GUI3D
#		PRIVATE Utils
		PRIVATE CxxTools
		PRIVATE ImageLoader
		PUBLIC SLAM_Wrapper
		PUBLIC SLAMBase_OFusion2Label
		PRIVATE cnpy
		PUBLIC MeshVoxelizer
		PUBLIC ConnectedComponent
		PUBLIC ${ASSIMP_LIBRARIES}
		PUBLIC GUI3D
		)
set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${targetname} PROPERTIES CXX_STANDARD 14)
INCLUDE(LinkPCL)
INCLUDE(LinkFreetype)
SOURCE_GROUP("Glad Files" FILES ${glad})

## OFusion
set(targetname exe_TrainingDataGenerator_ScanNetScan2CAD)
file(GLOB srcs exe_TrainingDataGenerator_ScanNetScan2CAD.cpp
		ScanNetMeshLoader.cpp
		ScanNetScan2CADLoader.cpp
		Scan2CADObjectLoader.cpp
		assimp_obj_loader.cpp
		third_party/json11.cpp)
file(GLOB headers TrainingDataGenerator.h)
file(GLOB cuda TrainingDataGenerator.cu TrainingDataGeneratorInstantiation.cu)
SET(sources
		${srcs}
		${cuda}
		${glad}
		)
SET(headers
		${headers}
		)

ADD_DEFINITIONS(-DPCL_ENABLE_SSE=OFF)
INCLUDE(SetCUDAAppTarget)
target_link_libraries(${targetname}
		PUBLIC GUI
		PRIVATE CxxTools
		PRIVATE ImageLoader
		PUBLIC SLAM_Wrapper
		PUBLIC SLAMBase_OFusion2Label
		PRIVATE cnpy
		PUBLIC MeshVoxelizer
		PUBLIC ${ASSIMP_LIBRARIES}
		PUBLIC ConnectedComponent
		)
set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${targetname} PROPERTIES CXX_STANDARD 14)
INCLUDE(LinkPCL)
INCLUDE(LinkFreetype)
SOURCE_GROUP("Glad Files" FILES ${glad})