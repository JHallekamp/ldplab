#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_DATA_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_DATA_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Data used internally during the pipeline exectuion. */
        struct PipelineData
        {
            /** @brief Contains kernel launch parameters. */
            struct KernelLaunchParameter
            {
                KernelLaunchParameter() :
                    grid_size{ 1, 1, 1 },
                    block_size{ 1, 1, 1 },
                    shared_memory_size{ 0 }
                { }
                dim3 grid_size;
                dim3 block_size;
                unsigned int shared_memory_size;
            };

            /** @brief Return value for ray buffer reduce kernel. */
            struct RayBufferReductionResult
            {
                /** @brief Number of active rays inside the buffer. */
                size_t num_active_rays;
                /** @brief Number of active rays that are in world space. */
                size_t num_world_space_rays;
            };

            DeviceBuffer<uint32_t> buffer_sort_local_rank;
            DeviceBuffer<uint32_t> buffer_sort_block_size;
            DeviceBuffer<uint32_t> buffer_sort_rank_index_range;

            DeviceBufferPinned<RayBufferReductionResult> 
                ray_buffer_reduction_result_buffer;
            KernelLaunchParameter ray_buffer_reduction_1_klp;
            KernelLaunchParameter ray_buffer_reduction_2_klp;
            
            KernelLaunchParameter buffer_setup_step_klp;
            KernelLaunchParameter buffer_setup_layer_klp;
        };
    }
}

#endif
#endif