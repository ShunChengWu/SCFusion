
set(targetname ConnectedComponent)

### includes ###
INCLUDE(UseCUDA)

file(GLOB_RECURSE headers ConnectedComponent/*.h)
file(GLOB_RECURSE templates)
file(GLOB_RECURSE sources
        ConnectedComponent/*.c
        ConnectedComponent/*.cpp
        ConnectedComponent/*.cu
        ConnectedComponent/*.tpp
        ConnectedComponent/*.tcu
)
INCLUDE(SetCUDALibTarget)
target_link_libraries(${targetname}
        PUBLIC ORUtils
        PUBLIC CxxTools
        )

FIND_PACKAGE(GTest)
IF(GTEST_INCLUDE_DIR)
  set(targetname TEST_CC)
  set(sources exe_cc.cpp)
  INCLUDE(SetCUDAAppTarget)
  target_link_libraries(${targetname}
		PUBLIC ConnectedComponent
  )
  INCLUDE(LinkGTest)
ENDIF(GTEST_INCLUDE_DIR)


