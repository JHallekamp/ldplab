#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_PARTICLE_INTERACTION_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_PARTICLE_INTERACTION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <memory>

#include "Context.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        /** @brief Typedefinition of ray particle interaction stage. */
        typedef void (*pipelineParticleInteractionStageKernel_t)(
            bool inner_particle_rays,
            int32_t* input_ray_index_buffer,
            Vec3* input_ray_origin_buffer,
            Vec3* input_ray_direction_buffer,
            double* input_ray_intensity_buffer,
            int32_t* reflected_ray_index_buffer,
            Vec3* reflected_ray_origin_buffer,
            Vec3* reflected_ray_direction_buffer,
            double* reflected_ray_intensity_buffer,
            double* reflected_ray_min_bv_dist_buffer,
            int32_t* transmitted_ray_index_buffer,
            Vec3* transmitted_ray_origin_buffer,
            Vec3* transmitted_ray_direction_buffer,
            double* transmitted_ray_intensity_buffer,
            double* transmitted_ray_min_bv_dist_buffer,
            int32_t* intersection_particle_index_buffer,
            Vec3* intersection_point_buffer,
            Vec3* intersection_normal_buffer,
            Vec3* output_force_per_ray_buffer,
            Vec3* output_torque_per_ray_buffer,
            size_t num_rays_per_buffer);

        /** @brief Abstract baseclass for ray particle interaction stage. */
        class IPipelineParticleInteractionStage
        {
        public:
            /** @brief Creates an instance of a pipeline stage implementation. */
            static std::shared_ptr<IPipelineParticleInteractionStage>
                createInstance(
                    const RayTracingStepCUDAInfo& info, Context& context);
            /** @brief Gets called before the pipeline enters execution. */
            virtual void setup() { }
            /** @brief Provides the caller with a pointer to the kernel. */
            virtual pipelineParticleInteractionStageKernel_t getKernel() = 0;
            /**
             * @brief Calculating resulting rays of the interaction of the
             *        incident ray with a particle surface.
             */
            virtual void execute(
                bool inner_particle_rays,
                size_t input_ray_buffer_index,
                size_t reflected_ray_buffer_index,
                size_t transmitted_ray_buffer_index) = 0;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELEINE_PARTICLE_INTERACTION_HPP