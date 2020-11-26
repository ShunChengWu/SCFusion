set(targetname SLAM_Wrapper)
INCLUDE(../cmake/UseCUDA.cmake)
file(GLOB_RECURSE headers *.h *.tpp)
file(GLOB_RECURSE sources *.cu)

INCLUDE(SetCUDALibTarget)
target_include_directories(${targetname} PUBLIC  ./include)
TARGET_LINK_LIBRARIES(${targetname} PUBLIC ImageLoader)


IF(WITH_GUI)
    TARGET_LINK_LIBRARIES(${targetname} PUBLIC GUI3D)
ENDIF(WITH_GUI)
