#ifndef WWU_LDPLAB_RTSCUDA_STAGE_BUFFER_SETUP_HPP
#define WWU_LDPLAB_RTSCUDA_STAGE_BUFFER_SETUP_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

#include "PipelineData.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class BufferSetup
        {
        public:
            BufferSetup() = delete;
            /** @brief Sets up the initial batch buffers. */
            static void executeStepSetup(
                StreamContext& stream_context,
                PipelineData& data);
            /** @brief Sets up the buffers. */
            static void executeLayerSetup(
                StreamContext& stream_context,
                PipelineData& data,
                size_t buffer_index,
                size_t output_buffer_index,
                size_t active_rays);
            /** @brief Creates the neccessary pipeline data. */
            static bool allocateData(
                const SharedStepData& shared_data,
                PipelineData& data);
        };
    }
}

#endif
#endif