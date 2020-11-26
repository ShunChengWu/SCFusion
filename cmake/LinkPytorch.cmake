#####################
# LinkPytorch.cmake #
#####################
IF(WITH_PYTORCH)
  if(TORCH_LIBRARIES)
    target_link_libraries(${targetname} PRIVATE "${TORCH_LIBRARIES}")
    target_compile_definitions(${targetname} PUBLIC COMPILE_WITH_PYTORCH)
    set_property(TARGET ${targetname} PROPERTY CXX_STANDARD 14)# pytorch needs at least c++14
    message(STATUS "link to Pytorch")
  else(TORCH_LIBRARIES)
    message(WARNING "Cannot link to Pytorch since it was not found.")
  endif(TORCH_LIBRARIES)
ELSE()
  MESSAGE(FATAL_ERROR "Did not find Pytorch!")
ENDIF(WITH_PYTORCH)