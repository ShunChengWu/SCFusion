##########################
# SetCUDALibTarget.cmake #
##########################

INCLUDE(${CMAKE_CURRENT_LIST_DIR}/Flags.cmake)

OPTION(BUILD_STATIC "Build static library?" ON)
SET(LIBRARY_TYPE)
IF(BUILD_STATIC)
  SET(LIBRARY_TYPE STATIC)
ELSE()
  SET(LIBRARY_TYPE SHARED)
ENDIF()

IF(WITH_CUDA)
  CUDA_ADD_LIBRARY(${targetname} ${LIBRARY_TYPE} ${sources} ${headers} ${templates})
ELSE()
  ADD_LIBRARY(${targetname} ${LIBRARY_TYPE} ${sources} ${headers} ${templates})
ENDIF()
