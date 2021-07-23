#ifndef WWU_LDPLAB_RTSCPU_INFO_HPP
#define WWU_LDPLAB_RTSCPU_INFO_HPP

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
            intensity_cutoff{ 0 },
            number_rays_per_buffer{ 0 },
            maximum_branching_depth{ 0 },
            number_parallel_pipelines{ 0 },
            number_reflections{ 1 },
            number_transmissions{ 1 },
            emit_warning_on_maximum_branching_depth_discardment{ true },
            return_force_in_particle_coordinate_system{ false }
        { }
        /**  @brief Under this cutoff intensity rays are not further traced. */
        double intensity_cutoff;
        /** @brief Number of rays per buffer. */
        size_t number_rays_per_buffer;
        /** @brief Maximum number of times a ray can split. */
        size_t maximum_branching_depth;
        /** @brief Number of parallel pipeline instances. */
        size_t number_parallel_pipelines;
        /** @brief Number of reflections per surface interaction per ray. */
        size_t number_reflections;
        /** @brief Number of transmissions per surface interaction per ray. */
        size_t number_transmissions;
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
