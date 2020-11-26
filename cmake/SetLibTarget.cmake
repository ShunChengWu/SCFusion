######################
# SetLibTarget.cmake #
######################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/Flags.cmake)

OPTION(BUILD_STATIC "Build static library?" ON)
SET(LIBRARY_TYPE)
IF(BUILD_STATIC)
    SET(LIBRARY_TYPE STATIC)
ELSE()
    SET(LIBRARY_TYPE SHARED)
ENDIF()

ADD_LIBRARY(${targetname} ${LIBRARY_TYPE} ${sources} ${headers} ${templates})
