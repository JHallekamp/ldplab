#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_BUFFER_SETUP_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_BUFFER_SETUP_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cstdint>
#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <memory>

namespace ldplab
{
    namespace rtscuda
    {
        // Prototype
        struct Context;

        /** @brief Abstract baseclass for the gather output stage. */
        class PipelineBufferSetup
        {
        public:
            /** @brief Creates an instance of the pipeline stage. */
            PipelineBufferSetup(Context& context) : m_context{ context }{ }
            /** @brief Reduces the output. */
            void execute();
            /** @brief Buffer setup device kernel. */
            static __global__ void bufferSetupKernel(
                int32_t* intersection_particle_index_buffer,
                Vec3* output_force_per_ray,
                Vec3* output_torque_per_ray,
                size_t num_rays_per_buffer);
        private:
            Context& m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELINE_BUFFER_SETUP_HPP