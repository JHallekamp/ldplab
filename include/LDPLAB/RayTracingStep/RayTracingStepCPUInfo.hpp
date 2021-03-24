#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CPU_INFO_HPP

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
            emit_warning_on_maximum_branching_depth_discardment{ true }
        { }
        /** @brief Parameters for the Eikonal solver. */
        std::shared_ptr<IEikonalSolverParameter> solver_parameters;
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
         * @brief Determines whether a warning is emited when active rays are
         *        not further traced because they would exceed the maximum 
         *        branching depth.
         */
        bool emit_warning_on_maximum_branching_depth_discardment;
    };

    // Prototypes
    namespace rtscpu
    {
        struct Context;
        class IInitialStage;
        class IInnerParticlePropagationStage;
        class IRayBoundingVolumeIntersectionTestStage;
        class IRayParticleInteractionStage;
        class IRayParticleIntersectionTestStage;
    }

    /** @brief Holds pointer used in debugging and testing environments. */
    struct RayTracingStepCPUDebugInfo
    {
        /** @brief Pointer to the RTSCPU context structure. */
        std::shared_ptr<rtscpu::Context> 
            context;
        /** @brief Pointer to the initial stage implementation. */
        std::unique_ptr<rtscpu::IInitialStage> 
            initial_stage;
        /** @brief Pointer to the inner particle propagation implementation. */
        std::unique_ptr<rtscpu::IInnerParticlePropagationStage> 
            inner_particle_propagation;
        /** @brief Pointer to ray bounding volume intersection test implementation. */
        std::unique_ptr<rtscpu::IRayBoundingVolumeIntersectionTestStage>
            ray_bounding_volume_intersection_test;
        /** @brief Pointer to ray particle interaction implementation. */
        std::unique_ptr<rtscpu::IRayParticleInteractionStage>
            ray_particle_interaction;
        /** @brief Pointer to ray particle intersection test implementation. */
        std::unique_ptr<rtscpu::IRayParticleIntersectionTestStage>
            ray_particle_intersection_test;
    };
}

#endif
