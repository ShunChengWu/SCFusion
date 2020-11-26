# Use FreeType2
###
# FreeType2_FOUND
# FreeType2_INCLUDE_DIRS
# FreeType2_LIBRARIES

SET(NAME FreeType2)
SET(URL "https://github.com/aseprite/freetype2.git")
SET(${NAME}_INSTALL_DIR  ${CMAKE_BINARY_DIR}/external/${NAME})
SET(${NAME}_DOWNLOADED 0)
IF(NOT ${NAME}_DOWNLOADED)
    find_package(Git)
    include(FindPythonInterp)
    file(MAKE_DIRECTORY ${${NAME}_INSTALL_DIR})
    execute_process(
            COMMAND ${GIT_EXECUTABLE} clone ${URL} ${NAME}
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external
    )

    file(MAKE_DIRECTORY ${${NAME}_INSTALL_DIR}/build)
    execute_process(
            COMMAND cmake -DCMAKE_INSTALL_PREFIX=${${NAME}_INSTALL_DIR}/bin ..
            WORKING_DIRECTORY ${${NAME}_INSTALL_DIR}/build
    )
    execute_process(
            COMMAND make install
            WORKING_DIRECTORY ${${NAME}_INSTALL_DIR}/build
    )
    SET(${NAME}_DOWNLOADED 1 CACHE STRING " ")
    SET(${NAME}_FOUND 1 CACHE STRING " ")
    #SET(${NAME}_INCLUDE_DIR ${${NAME}_INSTALL_DIR}/build/include CACHE STRING " ")
ENDIF()

FIND_PACKAGE(freetype REQUIRED PATHS ${${NAME}_INSTALL_DIR}/bin/lib/cmake NO_DEFAULT_PATH)

#FIND_PATH(${NAME}_INCLUDE_DIRS ft2build.h  DOC "path to ${NAME} directory"
#        PATHS ${${NAME}_INSTALL_DIR}/bin/include/freetype2)
#FIND_LIBRARY(${NAME}_LIBRARIES DOC "abs path to ${NAME} library."
#        NAMES libfreetype.a libfreetype.so
#        HINTS ${${NAME}_INSTALL_DIR}/bin/lib
#        PATHS ${${NAME}_INSTALL_DIR}/bin/lib)
#
#IF(${NAME}_LIBRARIES AND ${NAME}_INCLUDE_DIRS)
#    SET(${NAME}_FOUND 1 CACHE STRING " ")
#ELSE()
#    SET(${NAME}_FOUND 0 CACHE STRING " ")
#ENDIF()