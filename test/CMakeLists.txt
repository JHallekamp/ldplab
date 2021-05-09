add_executable(ldplab_test_simulation_cpu TestSimulationCPU.cpp)

target_include_directories(ldplab_test_simulation_cpu
	PUBLIC
		"${LDPLAB_SOURCE_DIR}/include"
		"${LDPLAB_SOURCE_DIR}/src"
    "${LDPLAB_SOURCE_DIR}/external/glm"
)

target_link_libraries(ldplab_test_simulation_cpu PRIVATE ldplab)