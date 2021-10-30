#ifndef WWU_LDPLAB_RAY_TRACING_STEP_CUDA_INFO_HPP
#define WWU_LDPLAB_RAY_TRACING_STEP_CUDA_INFO_HPP

#include "AcceleratorStructureParameter.hpp"
#include "EikonalSolverParameter.hpp"

#include <LDPLAB/RayTracingStep/CUDA/ExecutionModelInfo.hpp>

#include <memory>

namespace ldplab
{
    /**
     * @brief Structure containing all information to create an 
     *        RayTracingStep 
     */
    struct RayTracingStepCUDAInfo
    {
        /** 
         * @brief Description of the pipeline execution model. 
         * @note If this is a nullptr, the factory automatically creates a
         *       simulation with exactly one stream on exactly one device (the
         *       one with the lowest id).
         */
        std::shared_ptr<rtscuda::IExecutionModelInfo> execution_model_info = nullptr;
        /** 
         * @brief Threshold for reordering buffers.
         * @details The pipeline reorders all buffers for performance reasons.
         *          The reordering is done when the number of active rays 
         *          sinks below the buffer size times the threshold.
         *          Use a threshold of 0.0 to never reorder and a threshold of 
         *          1.0 to always reorder.
         */
        double buffer_reorder_threshold = 0.8;
        /**
         * @brief Percentage threshold for early ray_buffer_sort abort.
         * @details The pipeline does not sort the ray buffers if the number 
         *          of conflicts (more than one particle per warp) exceeds 
         *          the given percentage threshold of the number of particles.
         * @note A threshold of 0 denotes that the pipeline always sorts.
         */
        double sort_abort_threshold = 1.0;
        /** 
         * @brief Tells the pipeline to sort buffer contents based on the 
         *        particle index.
         */
        bool sort_ray_buffer = true;
        /** @brief Minimum number of rays (valid or invalid) inside the buffers. */
        size_t buffer_min_size = 2048;
        /** @brief Under this cutoff intensity rays are not further traced. */
        double intensity_cutoff = 0.0;
        /** @brief Maximum number of times a ray can split. */
        size_t maximum_branching_depth = 0;
        /** @brief Number of rays per batch. */
        size_t number_rays_per_batch = 0;
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
