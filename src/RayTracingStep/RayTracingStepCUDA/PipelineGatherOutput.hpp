#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_GATHER_OUTPUT_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_GATHER_OUTPUT_HPP
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

        /** @brief For the gather output stage. */
        class PipelineGatherOutput
        {
        public:
            /** @brief Creates an instance of the pipeline stage. */
            PipelineGatherOutput(Context& context) : m_context{ context }{ }
            /** @brief Host interface for output gather stage execution. */
            void execute(size_t ray_buffer_index);
        private:
            Context& m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELINE_GATHER_OUTPUT_HPP