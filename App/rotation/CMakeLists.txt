###
## Find Module
###
FIND_PACKAGE(PCL 1.8 REQUIRED)

SET(targetname exe_rotate_mesh)
ADD_EXECUTABLE(${targetname} rotmesh.cpp pclloader.hpp)
SET_TARGET_PROPERTIES(${targetname} PROPERTIES CXX_STANDARD 11)
TARGET_LINK_LIBRARIES(${targetname}
        PUBLIC ${PCL_LIBRARIES}
        PUBLIC CxxTools)
TARGET_LINK_DIRECTORIES(${targetname} PUBLIC ${PCL_LIBRARY_DIRS})
TARGET_INCLUDE_DIRECTORIES(${targetname} PUBLIC ${PCL_INCLUDE_DIRS})