#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP

#include "..\ExperimentalSetup\ExperimentalSetup.hpp"
#include "..\ThreadPool.hpp"

#include <memory>

namespace ldplab
{
    /**
     * @brief Structure containing all information to create an 
     *        RayTracingStepCPU 
     */
    struct RayTracingStepCPUInfo
    {
        std::shared_ptr<ExperimentalSetup> setup;
        std::shared_ptr<ThreadPool> thread_pool;
        /**  @brief Under this cutoff intensity rays are not further traced. */
        double intensity_cutoff;
        /** @brief Number of rays per buffer. */
        size_t number_rays_per_buffer;
        /** @brief Number of rays per world space unit. */
        size_t number_rays_per_unit;
        /** @brief Maximum number of times a ray can split. */
        size_t maximum_depth;
        /** @brief Number of parallel pipeline instances. */
        size_t number_parallel_pipelines;
    };
}

#endif
