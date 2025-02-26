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
                StreamContext& stream_context,
                size_t ray_buffer_index,
                size_t intersection_buffer_index,
                size_t num_rays) override;
        };
    }
}

#endif
#endif