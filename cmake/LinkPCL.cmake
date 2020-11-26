#################
# LinkPCL.cmake #
#################
if(PCL_INCLUDE_DIRS)
  TARGET_LINK_LIBRARIES(${targetname} PUBLIC ${PCL_LIBRARIES})
  TARGET_INCLUDE_DIRECTORIES(${targetname} PUBLIC ${PCL_INCLUDE_DIRS})
  TARGET_LINK_DIRECTORIES(${targetname} PUBLIC ${PCL_LIBRARY_DIRS})
  TARGET_COMPILE_DEFINITIONS(${targetname} PUBLIC COMPILE_WITH_PCL)
elseif(PCL_INCLUDE_DIRS)
  message(WARNING "Cannot link PCL since it was not found.")
endif(PCL_INCLUDE_DIRS)