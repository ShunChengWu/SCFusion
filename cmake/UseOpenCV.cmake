###################
# UseOpenCV.cmake #
###################

OPTION(WITH_OpenCV "Build with OpenCV support?" ON)

IF(WITH_OpenCV)
  find_package(OpenCV)
  INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIB_DIRS})
  ADD_DEFINITIONS(-DCOMPILE_WITH_OPENCV)
ELSE()
  ADD_DEFINITIONS(-DCOMPILE_WITHOUT_OPENCV)
ENDIF()
