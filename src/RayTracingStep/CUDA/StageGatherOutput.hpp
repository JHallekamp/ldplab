#ifndef WWU_LDPLAB_RTSCUDA_STAGE_GATHER_OUTPUT_HPP
#define WWU_LDPLAB_RTSCUDA_STAGE_GATHER_OUTPUT_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

#include "PipelineData.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class GatherOutput
        {
        public:
            GatherOutput() = delete;
            /** @brief Host interface for output gather stage execution. */
            static void execute(
                const GlobalData& global_data,
                BatchData& batch_data,
                PipelineData& data,
                size_t ray_buffer_index,
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