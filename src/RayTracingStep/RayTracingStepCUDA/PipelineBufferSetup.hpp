#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_BUFFER_SETUP_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_BUFFER_SETUP_HPP
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

        /** @brief Abstract baseclass for the gather output stage. */
        class PipelineBufferSetup
        {
        public:
            /** @brief Creates an instance of the pipeline stage. */
            PipelineBufferSetup(Context& context) : m_context{ context }{ }
            /** @brief Sets up the initial batch buffers. */
            void executeInitial();
            /** @brief Sets up the buffers. */
            void execute();
        private:
            Context& m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELINE_BUFFER_SETUP_HPP