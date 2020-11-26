###################
# UseFreetype.cmake #
###################
if(NOT Freetype_INCLUDE_DIR)
  find_package(Freetype REQUIRED)
  ADD_DEFINITIONS(-DCOMPILE_WITH_FREETYPE)
endif(NOT Freetype_INCLUDE_DIR)


