# Find glm
# Use glm as an external project
#
###
# glm_FOUND
# glm_INCLUDE_DIRS
#set(glm_FOUND 0)
IF(NOT glm_FOUND)
    find_package(Git)
    #include(FindPythonInterp)

    SET(NAME glm)
    SET(URL "https://github.com/g-truc/glm.git")

    SET(${NAME}_INSTALL_PREFIX  ${CMAKE_BINARY_DIR}/external)
    file(MAKE_DIRECTORY ${${NAME}_INSTALL_PREFIX})
    execute_process(
            COMMAND ${GIT_EXECUTABLE} clone ${URL} ${NAME}
            WORKING_DIRECTORY ${${NAME}_INSTALL_PREFIX}
    )

    SET(glm_INCLUDE_DIRS ${glm_INSTALL_PREFIX}/${NAME} CACHE STRING "glm include directory")
    SET(glm_FOUND 1 CACHE STRING "Set to 1 if NanoGUI is found, 0 otherwise")
ENDIF()