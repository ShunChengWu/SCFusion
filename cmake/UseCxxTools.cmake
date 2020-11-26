include(${PROJECT_SOURCE_DIR}/cmake/cmake_macro_addExternalProject/macro.cmake)

addExternalProjectGit (
	cxxtools
	https://github.com/ShunChengWu/CxxTools.git
	cxxtools_DIR
)

include(${cxxtools_DIR}/cmake/CxxToolsTargets.cmake)
link_directories(${cxxtools_DIR}/lib)