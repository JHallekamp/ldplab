#ifndef WWU_LDPLAB_RTSCUDA_I_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCUDA_I_INITIAL_STAGE_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include "IPipelineStage.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class IInitialStage : public IPipelineStage
        {
        public:
            virtual ~IInitialStage() { }
            virtual bool execute(
                const GlobalData& global_data,
                BatchData& batch_data,
                size_t initial_batch_buffer_index) = 0;
        };
    }
}

#endif
#endif