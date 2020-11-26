####################################
# This macro is used for adding an External Project in your project. 
# Inputs: 
#  NAME : the name of this external project
#  URL  : the url in ssh to your internate git repository 
#  BUILD_DIR : this variable is an in/out variable. Used for check whether this
#	       external project has been built before. This variable will be set
#	       to ${PROJECT_BINRY_DIR}/${NAME}
# The imported project will locate at ${PROJECT_BINRY_DIR}/${NAME}.
# If the external project has install target, the CMAKE_INSTALL_PREFIX will be set
# to ${BUILD_DIR}. 
# The setting files will be stored in ${BUILD_DIR}/build/setup
# Also a target with name update-${NAME} will be created for updating this project
# 
# And make sure the following files exist in your ${PROJECT_SOURCE_DIR}/cmake_macro_addExternalProject/cmake
# ExternalProject.cmake.in
##################

macro(addExternalProjectGit NAME URL BUILD_DIR)
if (NOT ${BUILD_DIR})
    SET(PROJECT_NAME ${NAME})
    SET(PROJECT_URL ${URL})
    SET(OUTPUT_PATH ${CMAKE_BINARY_DIR}/${NAME})
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
      set (CMAKE_INSTALL_PREFIX ${OUTPUT_PATH} CACHE PATH "default install path" FORCE )
    endif()

    configure_file(${PROJECT_SOURCE_DIR}/cmake/cmake_macro_addExternalProject/ExternalProject.cmake.in
		${PROJECT_BINARY_DIR}/${NAME}/build/setup/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        	RESULT_VARIABLE result
        	WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${NAME}/build/setup)
    if(result)
       message(FATAL_ERROR "CMake step for ${NAME} failed: ${result}")
    endif()

    execute_process(COMMAND ${CMAKE_COMMAND} --build .
       RESULT_VARIABLE result
       WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${NAME}/build/setup)
    if(result)
       message(FATAL_ERROR "Build step for ${NAME} failed: ${result}")
    endif()

 #   set(USED_CMAKE_GENERATOR "${CMAKE_GENERATOR}" CACHE STRING "Expose CMAKE_GENERATOR" FORCE)
#    message("USED_CMAKE_GENERATOR: " ${USED_CMAKE_GENERATOR})

    IF(CMAKE_GENERATOR STREQUAL Xcode OR CMAKE_GENERATOR STREQUAL MSVC)
    add_custom_target(update-${NAME}
       COMMAND ${CMAKE_COMMAND} --build . --target ${NAME}-update
       COMMAND ${CMAKE_COMMAND} --build . --target ALL_BUILD
       WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${NAME}/build/setup)
    ELSE(CMAKE_GENERATOR STREQUAL Xcode OR CMAKE_GENERATOR STREQUAL MSVC)
    add_custom_target(update-${NAME}
       COMMAND ${CMAKE_COMMAND} --build . --target ${NAME}-update
       COMMAND ${CMAKE_COMMAND} --build . --target all
       WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${NAME}/build/setup)
    ENDIF(CMAKE_GENERATOR STREQUAL Xcode OR CMAKE_GENERATOR STREQUAL MSVC)

    set(${BUILD_DIR} ${CMAKE_INSTALL_PREFIX})
endif (NOT ${BUILD_DIR})
endmacro(addExternalProjectGit)