#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CUDA_INFO_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CUDA_INFO_HPP

#include "AcceleratorStructureParameter.hpp"
#include "EikonalSolverParameter.hpp"

#include <memory>

namespace ldplab
{
    /**
     * @brief Structure containing all information to create an 
     *        RayTracingStep 
     */
    struct RayTracingStepCUDAInfo
    {
        /**  @brief Under this cutoff intensity rays are not further traced. */
        double intensity_cutoff = 0.0;
        /** @brief Maximum number of times a ray can split. */
        size_t maximum_branching_depth = 0;
        /** @brief Number of rays per batch. */
        size_t number_rays_per_batch = 0;
        /** @brief Maximum number of parallel batches. */
        size_t number_parallel_batches = 1;
        /** @brief Number of reflections per surface interaction per ray. */
        size_t number_reflections = 1;
        /** @brief Number of transmissions per surface interaction per ray. */
        size_t number_transmissions = 1;
        /** 
         * @brief Determines whether the host or device bound pipeline 
         *        implementation is used.
         */
        bool host_bound_pipeline = true;
        /**
         * @brief Determines if the finial particle force and torque is
         *        returned in particle coordinate system.
         */
        bool return_force_in_particle_coordinate_system = false;
    };
}

#endif
