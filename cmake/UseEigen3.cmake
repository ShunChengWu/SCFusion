###################
# UseEigen3.cmake #
###################

OPTION(WITH_EIGEN "Build with libEIGEN support?" ON)

IF(WITH_EIGEN)
  if(NOT EIGEN3_INCLUDE_DIR)
    find_package(Eigen3 REQUIRED)
  endif(NOT EIGEN3_INCLUDE_DIR)

  if(EIGEN3_INCLUDE_DIR)
    include_directories(${EIGEN_INCLUDE_DIRS})
  endif(EIGEN3_INCLUDE_DIR)

  ADD_DEFINITIONS(-DCOMPILE_WITH_EIGEN)
ENDIF()


