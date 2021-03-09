#ifndef WWU_LDPLAB_RAY_TRACING_STEP_GPU_OPEN_GL_INFO_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_GPU_OPEN_GL_INFO_HPP

#include "EikonalSolver.hpp"

#include <memory>
#include <string>

namespace ldplab
{
    /**
     * @brief Structure containing all information to create an 
     *        RayTracingStep instance.
     */
    struct RayTracingStepGPUOpenGLInfo
    {
        RayTracingStepGPUOpenGLInfo()
            :
            intensity_cutoff{ 0 },
            number_rays_per_buffer{ 0 },
            light_source_ray_density_per_unit_area{ 0 },
            maximum_branching_depth{ 0 },
            number_parallel_pipelines{ 0 },
            solver_parameters{ nullptr },
            emit_warning_on_maximum_branching_depth_discardment{ true }
        { }
        /** @brief Parameters for the Eikonal solver. */
        std::shared_ptr<IEikonalSolver> solver_parameters;
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
        /** 
         * @brief Contains the path of the LDPLAB shader base directory (which
         *        contains subdirectories for different shader languages).
         * @note The directory path needs to include a trailing forewardslash 
         *       ('/') at the end.
         */
        std::string shader_base_directory_path;
    };
}

#endif
