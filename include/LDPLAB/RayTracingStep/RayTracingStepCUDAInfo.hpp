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
         * @brief Active to inactive ratio threshold for buffer packing.
         * @details The threshold is the maximum allowed ratio of active rays 
         *          to the number of rays inside a buffer. If said ratio sinks
         *          below the threshold, the pipeline performs buffer packing.
         *          Therefore a theshold of 0 prevents buffer packing, while
         *          a threshold of 1 leads to the pipeline immeadiatly 
         *          performing buffer packing when the number of active rays 
         *          unequals the number of rays inside a buffer.
         */
        double buffer_packing_threshold = 0.8;
        /**
         * @brief The minimum buffer size for buffer packing.
         * @details Buffer packing is only performed when the number of rays
         *          inside a buffer is greater or equal to this value.
         */
        size_t buffer_packing_min_buffer_size = 2048;
        /** 
         * @brief Tells the pipeline to sort buffer contents based on the 
         *        particle index.
         */
        bool buffer_sort_enabled = false;
        /**
         * @brief Percentage threshold for early ray_buffer_sort abort.
         * @details The pipeline does not sort the ray buffers if the number 
         *          of conflicts (more than one particle per warp) exceeds 
         *          the given percentage threshold of the number of particles.
         * @note A threshold of 0 denotes that the pipeline always sorts.
         */
        double buffer_sort_abort_threshold = 1.0;
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
