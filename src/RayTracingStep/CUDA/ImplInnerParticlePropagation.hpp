#ifndef WWU_LDPLAB_RTSCUDA_IMPL_INNER_PARTICLE_PROPAGATION_HPP
#define WWU_LDPLAB_RTSCUDA_IMPL_INNER_PARTICLE_PROPAGATION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <LDPLAB/RayTracingStep/CUDA/IInnerParticlePropagation.hpp>
#include <LDPLAB/RayTracingStep/EikonalSolverParameter.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        class InnerParticlePropagationRK4 :
            public IInnerParticlePropagation
        {
        public:
            InnerParticlePropagationRK4(
                const RK4Parameter& parameter);
            void execute(
                const GlobalData& global_data,
                BatchData& batch_data,
                size_t ray_buffer_index,
                size_t intersection_buffer_index) override;
        private:
            const RK4Parameter m_parameter;
        };
    }
}

#endif
#endif