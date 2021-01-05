#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP

#include "EikonalSolver.hpp"
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
        std::shared_ptr<ThreadPool> thread_pool;
        /**  @brief Under this cutoff intensity rays are not further traced. */
        double intensity_cutoff;
        /** @brief Number of rays per buffer. */
        size_t number_rays_per_buffer;
        /** @brief Number of rays per light source square unit */
        size_t light_source_ray_density_per_unit_area;
        /** @brief Maximum number of times a ray can split. */
        size_t maximum_branching_depth;
        /** @brief Number of parallel pipeline instances. */
        size_t number_parallel_pipelines;
        /** @brief Parameters for the Eikonal solver */
        std::shared_ptr<IEikonalSolver> solver_parameters;
    };
}

#endif
