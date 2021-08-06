#ifndef WWU_LDPLAB_RTSCUDA_STAGE_BUFFER_SETUP_HPP
#define WWU_LDPLAB_RTSCUDA_STAGE_BUFFER_SETUP_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

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
                BatchData& batch_data);
            /** @brief Sets up the buffers. */
            static void executeLayerSetup(
                BatchData& batch_data,
                size_t buffer_index);
        };
    }
}

#endif
#endif