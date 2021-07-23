#include "RayTracingStepCPU.hpp"

#include "../../Utils/Log.hpp"
#include "../../Utils/Assert.hpp"
#include "../../Utils/Profiler.hpp"

#include <chrono>

void ldplab::rtscpu::RayTracingStepCPU::execute(
    const SimulationState& input, 
    RayTracingStepOutput& output)
{
    LDPLAB_ASSERT(m_pipeline != nullptr);
    LDPLAB_ASSERT(input.particle_instances.size() == m_pipeline->m_sim_params.num_particles);
    LDPLAB_LOG_INFO("RTSCPU context %i: Ray tracing step starts execution", uid());
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    
    LDPLAB_LOG_DEBUG("RTSCPU context %i: Setup ray tracing pipeline", uid());
    LDPLAB_PROFILING_START(rtscpu_step_setup);
    m_pipeline->stepSetup(input);
    LDPLAB_PROFILING_STOP(rtscpu_step_setup);

    LDPLAB_LOG_DEBUG("RTSCPU context %i: Execute ray tracing pipeline", uid());
    LDPLAB_PROFILING_START(rtscpu_execute_pipeline);
    m_thread_pool->executeJobBatch(m_pipeline, m_pipeline->m_info.number_parallel_pipelines);
    m_pipeline->finalizeOutput(output);
    LDPLAB_PROFILING_STOP(rtscpu_execute_pipeline);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    const double elapsed_time = std::chrono::duration<double>(end - start).count();
    LDPLAB_LOG_INFO("RTSCPU context %i: Ray tracing step executed after %fs", 
        uid(), elapsed_time);
}
