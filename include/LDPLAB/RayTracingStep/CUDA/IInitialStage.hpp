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
            /**
             * @brief Executes the batch creation.
             * @param[in] stream_context The current stream context.
             * @param[in] batch_no The number of the batch that shall be created.
             * @param[in] initial_batch_buffer_index Index of the initial batch
             *                                       buffer.
             * @returns true, if a batch is created. Otherwise false. If no 
             *          batch is created (e.g. the called returned false), 
             *          the pipeline assumes that no batch with a higher index
             *          than the passed batch_no exists and therefore stops 
             *          running.
             */
            virtual bool execute(
                StreamContext& stream_context,
                size_t batch_no,
                size_t initial_batch_buffer_index) = 0;
        };
    }
}

#endif