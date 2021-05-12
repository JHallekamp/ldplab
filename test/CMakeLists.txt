add_executable(ldplab_test_simulation_cpu TestSimulationCPU.cpp)

target_include_directories(ldplab_test_simulation_cpu
	PUBLIC
		"${LDPLAB_SOURCE_DIR}/include"
		"${LDPLAB_SOURCE_DIR}/src"
    "${LDPLAB_SOURCE_DIR}/external/glm"
)

target_link_libraries(ldplab_test_simulation_cpu PRIVATE ldplab)

if(LDPLAB_BUILD_OPTION_USE_SINGLE_PRECISION)
    target_compile_definitions(ldplab_test_simulation_cpu PRIVATE LDPLAB_BUILD_OPTION_USE_SINGLE_PRECISION)
endif()