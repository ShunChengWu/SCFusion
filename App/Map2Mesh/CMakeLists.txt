###############################################################################
### -- 3rd party
INCLUDE(UseCUDA)
#INCLUDE(UseFreetype)

###########################################################################################
SET(basetargetname exe_Map2Mesh)
file(GLOB srcs *.cpp)
file(GLOB headers *.hpp *.h)
file(GLOB cuda *.cu)
SET(sources
        ${srcs}
        ${cuda}
        )
SET(headers ${headers})

LIST(APPEND NAMES OFusionRGB1Label)# TSDF
LIST(LENGTH NAMES count)
MATH(EXPR count "${count}-1")
FOREACH(i RANGE ${count} )
    LIST(GET NAMES ${i} name)
    SET(NAME_SUFFIX _${name})
    SET(targetname ${basetargetname}${NAME_SUFFIX})

    INCLUDE(SetCUDAAppTarget)
    TARGET_LINK_LIBRARIES(${targetname}
            PUBLIC SLAMBase${NAME_SUFFIX}
            PUBLIC CxxTools
            PUBLIC SLAM_Wrapper
            )
    set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
    set_target_properties(${targetname} PROPERTIES CXX_STANDARD 11)
ENDFOREACH()
