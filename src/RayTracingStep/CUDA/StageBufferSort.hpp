#ifndef WWU_LDPLAB_RTSCUDA_STAGE_SORT_HPP
#define WWU_LDPLAB_RTSCUDA_STAGE_SORT_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

#include "PipelineData.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class BufferSort
        {
        public:
            BufferSort() = delete;
            /** @brief Reduces the given ray buffers indices. */
            static size_t execute(
                StreamContext& stream_context,
                PipelineData& pipeline_data,
                size_t buffer_index,
                size_t active_rays,
                size_t prev_active_rays);
            /** @brief Creates the neccessary pipeline data. */
            static bool allocateData(
                const SharedStepData& shared_data,
                PipelineData& data);
        };
    }
}

#endif
#endif