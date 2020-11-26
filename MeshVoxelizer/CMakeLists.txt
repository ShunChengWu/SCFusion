set(targetname MeshVoxelizer)

INCLUDE(UseCUDA)
#FIND_PACKAGE(GLM REQUIRED)
file(GLOB_RECURSE headers *.h *.hpp *.tpp)
file(GLOB_RECURSE sources *.cpp *.c *.cc *.cu)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)
