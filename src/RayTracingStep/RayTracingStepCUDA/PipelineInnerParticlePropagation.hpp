#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_INNER_PARTICLE_PROPAGATION_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_INNER_PARTICLE_PROPAGATION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <memory>

#include "Context.hpp"
#include "GenericParticleGeometry.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Typedefinition of inner particle propagation stage. */
        typedef void (*pipelineInnerParticlePropagationStageKernel_t)(
            int32_t* input_ray_index_buffer,
            Vec3* input_ray_origin_buffer,
            Vec3* input_ray_direction_buffer,
            double* input_ray_intensity_buffer,
            Vec3* intersection_point_buffer,
            Vec3* intersection_normal_buffer,
            size_t num_rays_per_buffer,
            GenericParticleGeometryData* geometry_per_particle,
            Vec3* output_force_per_particle_buffer,
            Vec3* output_torque_per_particle_buffer,
            size_t num_particles);

        /** @brief Abstract baseclass for the inner particle propagation. */
        class IPipelineInnerParticlePropagation
        {
        public:
            /** @brief Creates an instance of a pipeline stage implementation. */
            static std::shared_ptr<IPipelineInnerParticlePropagation> 
                createInstance(
                    const RayTracingStepCUDAInfo& info, Context& context);
            /** @brief Gets called before the pipeline enters execution. */
            virtual void setup() { }
            /** @brief Provides the caller with a pointer to the kernel. */
            virtual pipelineInnerParticlePropagationStageKernel_t 
                getKernel() = 0;
            /** @brief Calculating the path of the rays threw the particle. */
            virtual void execute(size_t ray_buffer_index) = 0;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELEINE_INNER_PARTICLE_PROPAGATION_HPP