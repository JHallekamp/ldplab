#ifndef WWU_LDPLAB_RTSCUDA_STAGE_GATHER_OUTPUT_HPP
#define WWU_LDPLAB_RTSCUDA_STAGE_GATHER_OUTPUT_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/Data.hpp>

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
                BatchData& batch_data,
                size_t ray_buffer_index);
        };
    }
}

#endif
#endif