# =============================================================================
# CPU Example
add_executable(ldplab_test_example TestExample.cpp)
target_include_directories(ldplab_test_example
	PUBLIC
		"${LDPLAB_SOURCE_DIR}/include"
		"${LDPLAB_SOURCE_DIR}/src"
    "${LDPLAB_SOURCE_DIR}/external/glm"
)
target_link_libraries(ldplab_test_example PRIVATE ldplab)

# =============================================================================
# CPU Target
add_executable(ldplab_test_simulation_cpu TestSimulationCPU.cpp)
target_include_directories(ldplab_test_simulation_cpu
	PUBLIC
		"${LDPLAB_SOURCE_DIR}/include"
		"${LDPLAB_SOURCE_DIR}/src"
    "${LDPLAB_SOURCE_DIR}/external/glm"
)
target_link_libraries(ldplab_test_simulation_cpu PRIVATE ldplab)

# =============================================================================
# CUDA Target
if(LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA)
	add_executable(ldplab_test_simulation_cuda TestSimulationCUDA.cpp)
	target_include_directories(ldplab_test_simulation_cuda
		PUBLIC
			"${LDPLAB_SOURCE_DIR}/include"
			"${LDPLAB_SOURCE_DIR}/src"
		"${LDPLAB_SOURCE_DIR}/external/glm"
	)
	target_link_libraries(ldplab_test_simulation_cuda PRIVATE ldplab)
endif()

# =============================================================================
# OpenGL Target
if(LDPLAB_BUILD_OPTION_ENABLE_RTSOGL)
	add_executable(ldplab_test_simulation_open_gl TestSimulationOpenGL.cpp)
	target_include_directories(ldplab_test_simulation_open_gl
		PUBLIC
			"${LDPLAB_SOURCE_DIR}/include"
			"${LDPLAB_SOURCE_DIR}/src"
		"${LDPLAB_SOURCE_DIR}/external/glm"
	)
	target_link_libraries(ldplab_test_simulation_open_gl PRIVATE ldplab)
endif()