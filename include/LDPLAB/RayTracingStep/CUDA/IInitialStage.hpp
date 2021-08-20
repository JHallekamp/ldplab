#ifndef WWU_LDPLAB_RTSCUDA_I_INITIAL_STAGE_HPP
#define WWU_LDPLAB_RTSCUDA_I_INITIAL_STAGE_HPP

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
                StreamContext& stream_context,
                size_t batch_no,
                size_t initial_batch_buffer_index) = 0;
        };
    }
}

#endif