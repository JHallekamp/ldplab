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
            /** @brief Return value for ray buffer reduce kernel. */
            struct RayStateCountingResult
            {
                /** @brief Number of active rays inside the buffer. */
                size_t num_active_rays;
                /** @brief Number of active rays that are in world space. */
                size_t num_world_space_rays;
            };

            DeviceBuffer<uint32_t> buffer_reorder_local;
            DeviceBuffer<uint32_t> buffer_rorder_block_sizes;
            DeviceBuffer<uint32_t> buffer_reorder_rank_index_range;
            
            //DeviceBufferPinned<size_t> buffer_sort_num_rays_num_pivots;
            struct BufferSortLocalRank { int32_t particle_index; uint32_t rank; };
            DeviceBuffer<BufferSortLocalRank> buffer_sort_block_local_rank_per_ray;
            DeviceBuffer<size_t> buffer_sort_global_offset_per_particle;
            DeviceBuffer<int32_t> buffer_sort_swap_ray_pi;
            DeviceBuffer<Vec3> buffer_sort_swap_ray_origin;
            DeviceBuffer<Vec3> buffer_sort_swap_ray_direction;
            DeviceBuffer<double> buffer_sort_swap_ray_intensity;
            DeviceBuffer<double> buffer_sort_swap_ray_min_bv_distance;
            DeviceBuffer<Vec3> buffer_sort_swap_isec_point;
            DeviceBuffer<Vec3> buffer_sort_swap_isec_normal;
            DeviceBuffer<int32_t> buffer_sort_swap_isec_pi;

            DeviceBufferPinned<RayStateCountingResult>
                ray_buffer_reduction_result_buffer;
        };
    }
}

#endif
#endif