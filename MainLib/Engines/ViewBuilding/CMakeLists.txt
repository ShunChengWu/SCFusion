############################
# ViewBuildingEngine LIBRARY #
############################
set(targetname ViewBuildingEngine)
file(GLOB_RECURSE ${targetname}_headers *.h)
file(GLOB_RECURSE ${targetname}_sources
        *.c
        *.cpp
        *.cu
        *.tpp
        *.tcu)
ADD_LIBRARY(${targetname} OBJECT ${${targetname}_sources} ${${targetname}_headers})