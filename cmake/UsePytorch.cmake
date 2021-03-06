####################
# UsePytorch.cmake #
####################
SET(CMAKE_PREFIX_PATH_SAVE ${CMAKE_PREFIX_PATH})
set(CMAKE_PREFIX_PATH ${PYTORCH_PATH} ${CMAKE_PREFIX_PATH})
FIND_PACKAGE(Torch QUIET)

OPTION(WITH_PYTORCH "Build with Pytorch support? Need to pass PYTORCH_PATH to pytorchcpp folder" ${TORCH_FOUND})

IF(WITH_PYTORCH)
  FIND_PACKAGE(Torch REQUIRED)
  include_directories(${TORCH_INCLUDE_DIRS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
  ADD_DEFINITIONS(-DCOMPILE_WITH_PYTORCH)
  MESSAGE(STATUS "With Pytorch")
ENDIF(WITH_PYTORCH)

SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH_SAVE})
