###############################################################################
### -- 3rd party
INCLUDE(UseCUDA)
find_package(PCL 1.7 REQUIRED)
INCLUDE(UseEigen3)
###########################################################################################
#ADD_SUBDIRECTORY(Evaluation)


########################
# Generate Config File #
########################
#set(targetname generateConfigParams)
#
#SET(sources generateConfig.cpp)
#SET(headers)
#SET(templates)
#INCLUDE(SetCUDAAppTarget)
#TARGET_LINK_LIBRARIES(${targetname} PUBLIC SCFUSION_OFusion1Label)
#set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
#set_target_properties(${targetname} PROPERTIES CXX_STANDARD 11)


####################
# SCFUSION_INFINITAM #
####################
SET(basetargetname exe_scfusion)
SET(sources main_infinitam.cpp)
SET(headers)
SET(templates)
SET(NAMES)
LIST(APPEND NAMES OFusionRGB1Label )
LIST(LENGTH NAMES count)
MATH(EXPR count "${count}-1")
FOREACH(i RANGE ${count} )
	LIST(GET NAMES ${i} name)
	SET(NAME_SUFFIX _${name})
	SET(targetname ${basetargetname}${NAME_SUFFIX})

	INCLUDE(SetCUDAAppTarget)
	TARGET_LINK_LIBRARIES(${targetname}
			PUBLIC SCFUSION${NAME_SUFFIX}
			PUBLIC CxxTools
            		PUBLIC ImageLoader
			#PUBLIC PytorchCppTools
			#PUBLIC SLAM_Wrapper
			PUBLIC ORUtils
			)
	set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
	set_target_properties(${targetname} PROPERTIES CXX_STANDARD 14)
	INCLUDE(LinkPCL)
ENDFOREACH()

########################
# SCFUSION_INFINITAM_GUI #
########################
IF(WITH_GUI)
	INCLUDE(UseFreetype)
	SET(basetargetname exe_scfusion_gui)
	SET(sources main_gui_infinitam.cpp)
	SET(headers)
	SET(templates)
	SET(NAMES)
	LIST(APPEND NAMES OFusionRGB1Label)#TSDF OFusion OFusionRGB OFusion1Label
	LIST(LENGTH NAMES count)
	MATH(EXPR count "${count}-1")
	FOREACH(i RANGE ${count} )
		LIST(GET NAMES ${i} name)
		SET(NAME_SUFFIX _${name})
		SET(targetname ${basetargetname}${NAME_SUFFIX})

		INCLUDE(SetCUDAAppTarget)
		TARGET_LINK_LIBRARIES(${targetname}
				PUBLIC SCFUSION${NAME_SUFFIX}
				PUBLIC CxxTools
				PUBLIC SLAM_Wrapper
				)
		set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
		set_target_properties(${targetname} PROPERTIES CXX_STANDARD 14)
		INCLUDE(LinkFreetype)
		INCLUDE(LinkPCL)
	ENDFOREACH()
ENDIF(WITH_GUI)
#########################
## SCFUSION_RENDERED_GUI #
#########################
## OFusion
#SET(targetname exe_scfusion_rendered_gui)
#SET(sources main_rendered_gui.cpp)
#SET(headers)
#SET(templates)
#INCLUDE(SetCUDAAppTarget)
#TARGET_LINK_LIBRARIES(${targetname}
#		PUBLIC SCFUSION_OFusionRGB1Label
#		PUBLIC CxxTools
#		PUBLIC SLAM_Wrapper
#		)
#INCLUDE(LinkPCL)
#INCLUDE(LinkFreetype)
#set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
#set_target_properties(${targetname} PROPERTIES CXX_STANDARD 14)