###################################
# CMakeLists.txt for FernRelocLib #
###################################

###########################
# Specify the target name #
###########################

SET(targetname FernRelocLib)

################################
# Specify the libraries to use #
################################

INCLUDE(UseCUDA)
INCLUDE(UseOpenMP)

#############################
# Specify the project files #
#############################
FILE(GLOB sources *.cpp)
FILE(GLOB headers *.cpp)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(SetCUDALibTarget)
target_link_libraries(${targetname}
        PUBLIC ORUtils
        )
target_include_directories(${targetname} PUBLIC ./include)
