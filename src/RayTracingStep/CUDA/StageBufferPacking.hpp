#ifndef WWU_LDPLAB_RTSCUDA_STAGE_BUFFER_PACKING_HPP
#define WWU_LDPLAB_RTSCUDA_STAGE_BUFFER_PACKING_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

#include "PipelineData.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class BufferPacking
        {
        public:
            BufferPacking() = delete;
            /** @brief Reduces the given ray buffers indices. */
            static size_t execute(
                StreamContext& stream_context,
                PipelineData& pipeline_data,
                size_t buffer_index,
                size_t active_rays,
                size_t prev_active_rays,
                bool isec_or_output_contains_data);
            /** @brief Creates the neccessary pipeline data. */
            static bool allocateData(
                const SharedStepData& shared_data,
                PipelineData& data);
        };
    }
}

#endif
#endif