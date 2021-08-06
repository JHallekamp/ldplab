#ifndef WWU_LDPLAB_RTSCUDA_STAGE_RAY_BUFFER_REDUCE_HPP
#define WWU_LDPLAB_RTSCUDA_STAGE_RAY_BUFFER_REDUCE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Return value for ray buffer reduce kernel. */
        struct RayBufferReduceResult
        {
            /** @brief Number of active rays inside the buffer. */
            size_t num_active_rays;
            /** @brief Number of active rays that are in world space. */
            size_t num_world_space_rays;
        };

        class RayBufferReduce
        {
        public:
            RayBufferReduce() = delete;
            /** @brief Reduces the given ray buffers indices. */
            static RayBufferReduceResult execute(
                BatchData& batch_data,
                size_t ray_buffer_index);
        };
    }
}

#endif
#endif