cmake_minimum_required (VERSION 3.15)

project(LDPLAB
    VERSION 0.1
    DESCRIPTION "Light driven particle simulation library"
    LANGUAGES CXX
)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    find_package(Doxygen)
    if(Doxygen_FOUND)
        add_subdirectory(docs)
    else()
        message(STATUS "Doxygen not found, not building docs")
    endif()
endif()

option (LDPLAB_BUILD_OPTION_DISABLE_RTSGPU_OGL "Use OPENGL for Graphic card processing" OFF)

set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHESTRING "Choose the type of build." FORCE)
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
                        RESULT_VARIBALE GIT_SUMMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUMMOD_RESULT}, please check checkout submodules")
        endif()
    endif()
endif()


#find_package(OpenGL REQUIRED)
#find_package(GLEW REQUIRED)
add_subdirectory(external/glm)

if(NOT LDPLAB_BUILD_OPTION_DISABLE_RTSGPU_OGL)
    find_package(OpenGL REQUIRED)
    find_package(GLEW REQUIRED)
endif()

add_subdirectory(src/LDPLAB)