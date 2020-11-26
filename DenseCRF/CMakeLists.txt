SET(targetname DenseCRF)

OPTION(BUILD_DENSECRF_TEST "Build test of Dense Conditional Random Field" ON)


file(GLOB_RECURSE srcs
        densecrf.cpp
        util.cpp)

ADD_LIBRARY(${targetname} ${srcs})
SET_TARGET_PROPERTIES(${targetname} PROPERTIES
        CXX_STANDARD 11)


IF(BUILD_DENSECRF_TEST)
  ADD_SUBDIRECTORY(test)
ENDIF(BUILD_DENSECRF_TEST)
