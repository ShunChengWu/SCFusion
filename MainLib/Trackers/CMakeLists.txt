##########################
# LowLevelEngine LIBRARY #
##########################
set(targetname TrackingEngine)
file(GLOB_RECURSE ${targetname}_headers *.h)
file(GLOB_RECURSE ${targetname}_sources
        *.c
        *.cpp
        *.cu
        *.tpp
        *.tcu)

ADD_LIBRARY(${targetname} OBJECT ${${targetname}_sources} ${${targetname}_headers})
TARGET_LINK_LIBRARIES(${targetname}
        PUBLIC ITMObjects
        PUBLIC ITMUtils
)
