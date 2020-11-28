####################
# LinkEigen3.cmake #
####################

if(EIGEN3_INCLUDE_DIR)
  target_include_directories(${targetname} PUBLIC ${EIGEN3_INCLUDE_DIR})
  TARGET_COMPILE_DEFINITIONS(${targetname} PUBLIC COMPILE_WITH_EIGEN)
endif(EIGEN3_INCLUDE_DIR)