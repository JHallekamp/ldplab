#ifndef WWU_LDPLAB_RTSCUDA_I_INNER_PARTICLE_PROPAGATION_HPP
#define WWU_LDPLAB_RTSCUDA_I_INNER_PARTICLE_PROPAGATION_HPP

#include "IPipelineStage.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class IInnerParticlePropagation : public IPipelineStage
        {
        public:
            virtual ~IInnerParticlePropagation() { }
            virtual void execute(
                StreamContext& stream_context,
                size_t ray_buffer_index,
                size_t intersection_buffer_index,
                size_t output_buffer_index,
                size_t num_rays) = 0;
        };
    }
}

#endif