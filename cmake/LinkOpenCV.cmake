####################
# LinkOpenGL.cmake #
####################
IF(OpenCV_FOUND)
    TARGET_LINK_LIBRARIES(${targetname} PUBLIC ${OpenCV_LIBS})
    TARGET_INCLUDE_DIRECTORIES(${targetname} PUBLIC ${OpenCV_INCLUDE_DIRS})
    TARGET_LINK_DIRECTORIES(${targetname} PUBLIC ${OpenCV_LIB_DIRS})
    TARGET_COMPILE_DEFINITIONS(${targetname} PUBLIC COMPILE_WITH_OPENCV)
ENDIF()