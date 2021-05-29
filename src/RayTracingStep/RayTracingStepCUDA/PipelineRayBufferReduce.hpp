#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_RAY_BUFFER_REDUCE_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_RAY_BUFFER_REDUCE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cstdint>
#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <memory>

namespace ldplab
{
    namespace rtscuda
    {
        // Prototype
        struct Context;

        /** @brief Return value for ray buffer reduce kernel. */
        struct RayBufferReduceResult
        {
            /** @brief Number of active rays inside the buffer. */
            size_t num_active_rays;
            /** @brief Number of active rays that are in world space. */
            size_t num_world_space_rays;
        };

        /** @brief Abstract baseclass for the ray buffer reduction stage. */
        class PipelineRayBufferReduceStage
        {
        public:
            /** @brief Creates an instance of the pipeline stage. */
            PipelineRayBufferReduceStage(Context& context) : 
                m_context{ context } { }
            /** @brief Reduces the given ray buffers indices. */
            RayBufferReduceResult execute(size_t ray_buffer_index);
            /** 
             * @brief Ray buffer index reduction kernel.
             * @note Cuda threads can only synchronize with threads in the same
             *       block. Due to this, the reduction is performed block-wise
             *       and the result_buffer is assumed to contain a result per
             *       block, not a single overall result.
             * @warning Block size needs to be a power of 2 for this to work!
             */
            static __global__ void rayBufferReduceKernel(
                RayBufferReduceResult* result_buffer,
                int32_t* ray_index_buffer,
                size_t num_rays_per_batch,
                size_t num_particles);
        private:
            Context& m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_RAY_BUFFER_REDUCE_HPP