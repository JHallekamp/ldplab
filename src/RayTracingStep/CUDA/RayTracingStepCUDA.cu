#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "RayTracingStepCUDA.hpp"

#include "../../Utils/Log.hpp"
#include "../../Utils/Assert.hpp"
#include "../../Utils/Profiler.hpp"

#include <chrono>

void ldplab::rtscuda::RayTracingStepCUDA::execute(
	const SimulationState& input, 
	RayTracingStepOutput& output)
{
    LDPLAB_ASSERT(m_pipeline != nullptr);
    LDPLAB_ASSERT(input.particle_instances.size() == m_pipeline->m_sim_params.num_particles);
    LDPLAB_LOG_INFO("RTSCPU %i: Ray tracing step starts execution", uid());
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    LDPLAB_LOG_DEBUG("RTSCPU %i: Setup ray tracing pipeline", uid());
    LDPLAB_PROFILING_START(rtscuda_step_setup);
    m_pipeline->stepSetup(input);
    LDPLAB_PROFILING_STOP(rtscuda_step_setup);

    LDPLAB_LOG_DEBUG("RTSCPU %i: Execute ray tracing pipeline", uid());
    LDPLAB_PROFILING_START(rtscuda_execute_pipeline);
    m_pipeline->execute();
    m_pipeline->finalizeOutput(output);
    LDPLAB_PROFILING_STOP(rtscuda_execute_pipeline);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    const double elapsed_time = std::chrono::duration<double>(end - start).count();
    LDPLAB_LOG_INFO("RTSCPU %i: Ray tracing step executed after %fs",
        uid(), elapsed_time);
}

#endif