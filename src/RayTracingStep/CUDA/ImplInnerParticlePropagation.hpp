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
            InnerParticlePropagationRK4(const RK4Parameter& parameter);
            void execute(
                StreamContext& stream_context,
                size_t ray_buffer_index,
                size_t intersection_buffer_index,
                size_t output_buffer_index,
                size_t num_rays) override;
        private:
            const RK4Parameter m_parameter;
        };

        class InnerParticlePropagationRK4QueueFill :
            public IInnerParticlePropagation
        {
        public:
            InnerParticlePropagationRK4QueueFill(
                const RK4Parameter& parameter,
                std::vector<DeviceBufferPinned<uint32_t>>&& queue_ctr_per_stream);
            void execute(
                StreamContext& stream_context,
                size_t ray_buffer_index,
                size_t intersection_buffer_index,
                size_t output_buffer_index,
                size_t num_rays) override;
        private:
            const RK4Parameter m_parameter;
            std::vector<DeviceBufferPinned<uint32_t>> m_queue_ctr_per_stream;
        };
    }
}

#endif
#endif