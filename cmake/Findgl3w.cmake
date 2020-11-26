# Find gl3w
# Use gl3w as an external project
#
###
# gl3w_FOUND
# gl3w_SOURCE return the source file location
# gl3w_INCLUDE_DIRS

IF(NOT gl3w_FOUND)
    find_package(Git)
    include(FindPythonInterp)

    SET(NAME gl3w)
    SET(URL "https://github.com/skaslev/gl3w.git")

    SET(${NAME}_INSTALL_PREFIX  ${CMAKE_BINARY_DIR}/external/${NAME}-prefix)
    SET(${NAME}_INSTALL_DIR  ${CMAKE_BINARY_DIR}/external/${NAME}-install)
    file(MAKE_DIRECTORY ${${NAME}_INSTALL_DIR})
    file(MAKE_DIRECTORY ${${NAME}_INSTALL_PREFIX})
    execute_process(
            COMMAND ${GIT_EXECUTABLE} clone ${URL} ${NAME}
            WORKING_DIRECTORY ${${NAME}_INSTALL_PREFIX}
    )
    execute_process(
            COMMAND ${PYTHON_EXECUTABLE} gl3w_gen.py --root=${${NAME}_INSTALL_DIR}
            WORKING_DIRECTORY ${${NAME}_INSTALL_PREFIX}/${NAME}
    )

    SET(gl3w_INCLUDE_DIRS ${gl3w_INSTALL_DIR}/include CACHE STRING "gl3w source file")
    SET(gl3w_SOURCE ${gl3w_INSTALL_DIR}/src/gl3w.c CACHE STRING "gl3w source file")
    SET(gl3w_FOUND 1 CACHE STRING "Set to 1 if NanoGUI is found, 0 otherwise")
ENDIF()