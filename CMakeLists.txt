option (ENABEL_DOCS "Create documentation with Doxygen" OFF)
option (LDPLAB_BUILD_OPTION_ENABLE_RTSOGL "Use OpenGL for Graphic card processing" OFF)
option (LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA "Use CUDA for Graphic card processing" OFF)
option (LDPLAB_BUILD_OPTION_ENABLE_PROFILING "Enable LDPLAB internal profiling code" OFF)
option (LDPLAB_BUILD_OPTION_DISABLE_LOGGING "Disable LDPLAB logging" OFF)
option (BUILD_LDPLAB_VERIFICATION_PROJECT "Builds the LDPLAB verification project" OFF)
option (BUILD_LDPLAB_TEST_PROJECTS "Builds the LDPLAB test projects" OFF)

if(LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
	cmake_minimum_required (VERSION 3.18)
	project(LDPLAB
		VERSION 0.1
		LANGUAGES CXX CUDA
	)
	#set(CMAKE_CUDA_ARCHITECTURES 50)
else()
	cmake_minimum_required (VERSION 3.7)
	project(LDPLAB
		VERSION 0.1
		LANGUAGES CXX
	)
endif()

if(ENABEL_DOCS)
    if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
        find_package(Doxygen)
        if(Doxygen_FOUND)
            add_subdirectory(docs)
        else()
            message(WARNING "Doxygen not found, not building docs")
        endif()
    endif()
endif()

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()
SET(CMAKE_DEBUG_POSTFIX "d")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# DOWNLOAD ALL GIT SUBMODULES
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUMMOD_RESULT}, please check checkout submodules")
        endif()
    endif()
endif()

find_package(Threads REQUIRED)

if(LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
    message(WARNING "LDPLAB uses RTSOGL, which is deprecated and might not behave as expected.")
    find_package(OpenGL REQUIRED)
    find_package(GLEW REQUIRED)
endif()

if(LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
	find_package(CUDA REQUIRED)
endif()

add_subdirectory(external/glm)
add_subdirectory(external/tinyobjloader)
add_subdirectory(src)

if(BUILD_LDPLAB_VERIFICATION_PROJECT)
	add_subdirectory(test/Verification)
endif()

if(BUILD_LDPLAB_TEST_PROJECTS)
	add_subdirectory(test)
endif()