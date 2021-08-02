#ifndef WWU_LDPLAB_RTSCUDA_I_INNER_PARTICLE_PROPAGATION_HPP
#define WWU_LDPLAB_RTSCUDA_I_INNER_PARTICLE_PROPAGATION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

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
                const GlobalData& global_data,
                BatchData& batch_data,
                size_t ray_buffer_index,
                size_t intersection_buffer_index) = 0;
        };
    }
}

#endif
#endif