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
                const GlobalData& global_data,
                BatchData& batch_data,
                PipelineData& data);
            /** @brief Sets up the buffers. */
            static void executeLayerSetup(
                const GlobalData& global_data,
                BatchData& batch_data,
                PipelineData& data,
                size_t buffer_index,
                size_t output_buffer_index);
            /** @brief Creates the neccessary pipeline data. */
            static bool allocateData(
                const GlobalData& global_data,
                PipelineData& data);
        };
    }
}

#endif
#endif