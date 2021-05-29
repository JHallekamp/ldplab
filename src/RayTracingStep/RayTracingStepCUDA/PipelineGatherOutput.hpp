#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_GATHER_OUTPUT_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_GATHER_OUTPUT_HPP
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
        class PipelineGatherOutput
        {
        public:
            /** @brief Creates an instance of the pipeline stage. */
            PipelineGatherOutput(Context& context) : m_context{ context }{ }
            /** @brief Host interface for output gather stage execution. */
            void execute();
            /** @brief Gather output kernel. */
            static __global__ void gatherOutputKernel(
                int32_t* ray_index_buffer,
                Vec3* force_per_ray,
                Vec3* torque_per_ray,
                size_t num_rays_per_batch,
                Vec3* force_per_particle,
                Vec3* torque_per_particle,
                Mat3* p2w_transformations,
                Vec3* p2w_translations,
                size_t num_particles,
                bool particle_space_output);
        private:
            Context& m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELINE_GATHER_OUTPUT_HPP