###############################################################################
### -- 3rd party
INCLUDE(UseCUDA)

#################
# Utils LIBRARY #
#################
set(targetname ITMUtils)
file(GLOB_RECURSE headers Utils/*.h)
file(GLOB_RECURSE sources Utils/*.cpp)
INCLUDE(SetLibTarget)
TARGET_INCLUDE_DIRECTORIES(${targetname} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND SOURCES ${${targetname}_sources})
list(APPEND HEADERS ${${targetname}_headers})

###################
# Objects LIBRARY #
###################
set(targetname ITMObjects)
file(GLOB_RECURSE headers Objects/*.h)
file(GLOB_RECURSE sources Objects/*.cpp Objects/*.tcu Objects/*.cu)
INCLUDE(SetLibTarget)
TARGET_INCLUDE_DIRECTORIES(${targetname} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND SOURCES ${${targetname}_sources})
list(APPEND HEADERS ${${targetname}_headers})

ADD_SUBDIRECTORY(Trackers)
ADD_SUBDIRECTORY(Engines)
ADD_SUBDIRECTORY(Core)
ADD_SUBDIRECTORY(MainEngine)