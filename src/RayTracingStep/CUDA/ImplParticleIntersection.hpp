#ifndef WWU_LDPLAB_RTSCUDA_IMPL_PARTICLE_INTERSECTION_HPP
#define WWU_LDPLAB_RTSCUDA_IMPL_PARTICLE_INTERSECTION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/IParticleIntersection.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        class ParticleIntersection :
            public IParticleIntersection
        {
        public:
            void execute(
                const GlobalData& global_data,
                BatchData& batch_data,
                size_t ray_buffer_index,
                size_t intersection_buffer_index) override;
        };
    }
}

#endif
#endif