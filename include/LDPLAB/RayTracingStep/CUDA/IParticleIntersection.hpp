#ifndef WWU_LDPLAB_RTSCUDA_I_PARTICLE_INTERSECTION_HPP
#define WWU_LDPLAB_RTSCUDA_I_PARTICLE_INTERSECTION_HPP

#include "IPipelineStage.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        class IParticleIntersection : public IPipelineStage
        {
        public:
            virtual ~IParticleIntersection() { }
            virtual void execute(
                const GlobalData& global_data,
                BatchData& batch_data,
                size_t ray_buffer_index,
                size_t intersection_buffer_index) = 0;
        };
    }
}

#endif