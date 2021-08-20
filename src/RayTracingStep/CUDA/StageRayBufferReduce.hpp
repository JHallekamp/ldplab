#ifndef WWU_LDPLAB_RTSCUDA_STAGE_RAY_BUFFER_REDUCE_HPP
#define WWU_LDPLAB_RTSCUDA_STAGE_RAY_BUFFER_REDUCE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

#include "PipelineData.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class RayBufferReduce
        {
        public:
            RayBufferReduce() = delete;
            /** @brief Reduces the given ray buffers indices. */
            static PipelineData::RayBufferReductionResult execute(
                StreamContext& stream_context,
                PipelineData& pipeline_data,
                size_t ray_buffer_index);
            /** @brief Creates the neccessary pipeline data. */
            static bool allocateData(
                const SharedStepData& shared_data,
                PipelineData& data);
        };
    }
}

#endif
#endif