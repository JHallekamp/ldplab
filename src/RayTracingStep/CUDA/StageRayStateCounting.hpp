#ifndef WWU_LDPLAB_RTSCUDA_STAGE_RAY_STATE_COUNTING_HPP
#define WWU_LDPLAB_RTSCUDA_STAGE_RAY_STATE_COUNTING_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

#include "PipelineData.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class RayStateCounting
        {
        public:
            RayStateCounting() = delete;
            /** @brief Reduces the given ray buffers indices. */
            static PipelineData::RayStateCountingResult execute(
                StreamContext& stream_context,
                PipelineData& pipeline_data,
                size_t ray_buffer_index,
                size_t num_rays);
            /** @brief Creates the neccessary pipeline data. */
            static bool allocateData(
                const SharedStepData& shared_data,
                PipelineData& data);
        };
    }
}

#endif
#endif