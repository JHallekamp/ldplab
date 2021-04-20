#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP

#include "AcceleratorStructureParameter.hpp"
#include "EikonalSolverParameter.hpp"

#include <memory>

namespace ldplab
{
    /**
     * @brief Structure containing all information to create an 
     *        RayTracingStep 
     */
    struct RayTracingStepCPUInfo
    {
        RayTracingStepCPUInfo()
            :
            solver_parameters{ nullptr },
            intensity_cutoff{ 0 },
            number_rays_per_buffer{ 0 },
            light_source_ray_density_per_unit_area{ 0 },
            maximum_branching_depth{ 0 },
            number_parallel_pipelines{ 0 },
            emit_warning_on_maximum_branching_depth_discardment{ true },
            return_force_in_particle_coordinate_system{ false }
        { }
        /** @brief Parameters for the Eikonal solver. */
        std::shared_ptr<IEikonalSolverParameter> solver_parameters;
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
        /** @brief Number of rays per buffer. */
        size_t number_rays_per_buffer;
        /** @brief Number of rays per light source square unit */
        size_t light_source_ray_density_per_unit_area;
        /** @brief Maximum number of times a ray can split. */
        size_t maximum_branching_depth;
        /** @brief Number of parallel pipeline instances. */
        size_t number_parallel_pipelines;
        /**
         * @brief Determines whether a warning is emitted when active rays are
         *        not further traced because they would exceed the maximum 
         *        branching depth.
         */
        bool emit_warning_on_maximum_branching_depth_discardment;
        /**
         * @brief Determines if the finial particle force and torque is
         *        returned in particle coordinate system.
         */
        bool return_force_in_particle_coordinate_system;
    };
}

#endif
