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
        RayTracingStepCUDAInfo()
            :
            intensity_cutoff{ 0 },
            light_source_ray_density_per_unit_area{ 0 },
            maximum_branching_depth{ 0 },
            number_rays_per_batch{ 0 },
            number_threads_per_block{ 128 },
            return_force_in_particle_coordinate_system{ false },
            solver_parameters{ nullptr }
        { }
        /** 
         * @brief Parameters for the particle accelerator structure.
         * @details Determines the accelerator structure that is used.
         * @note This is only relevant for mesh particles, you can pass a 
         *       nullptr if no meshs are used.
         */
        std::shared_ptr<IAcceleratorStructureParameter> 
            accelerator_structure_parameters;
        /**  @brief Under this cutoff intensity rays are not further traced. */
        double intensity_cutoff;
        /** @brief Number of rays per light source square unit */
        size_t light_source_ray_density_per_unit_area;
        /** @brief Maximum number of times a ray can split. */
        size_t maximum_branching_depth;
        /** @brief Number of rays per batch. */
        size_t number_rays_per_batch;
        /** @brief Number of threads per block (between 1 and 512). */
        size_t number_threads_per_block;
        /**
         * @brief Determines if the finial particle force and torque is
         *        returned in particle coordinate system.
         */
        bool return_force_in_particle_coordinate_system;
        /** @brief Parameters for the Eikonal solver. */
        std::shared_ptr<IEikonalSolverParameter> solver_parameters;
    };
}

#endif
