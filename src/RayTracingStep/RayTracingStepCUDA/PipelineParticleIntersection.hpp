#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_PARTICLE_INTERSECTION_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_PARTICLE_INTERSECTION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <memory>

#include "GenericParticleGeometry.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        // Prototype
        struct Context;

        /** @brief Typedefinition of ray particle intersection stage. */
        typedef void (*pipelineParticleIntersectionStageKernel_t)(
            int32_t* input_ray_index_buffer,
            Vec3* input_ray_origin_buffer,
            Vec3* input_ray_direction_buffer,
            double* input_ray_intensity_buffer,
            Vec3* intersection_point_buffer,
            Vec3* intersection_normal_buffer,
            size_t num_rays_per_buffer,
            GenericParticleGeometryData* geometry_per_particle,
            size_t num_particles);

        /** @brief Abstract baseclass for ray particle intersection stage. */
        class IPipelineParticleIntersectionStage
        {
        public:
            virtual ~IPipelineParticleIntersectionStage() { }
            /** @brief Creates an instance of a pipeline stage implementation. */
            static std::shared_ptr<IPipelineParticleIntersectionStage>
                createInstance(
                    const RayTracingStepCUDAInfo& info, Context& context);
            /** @brief Gets called before the pipeline enters execution. */
            virtual void setup() { }
            /** @brief Provides the caller with a pointer to the kernel. */
            virtual pipelineParticleIntersectionStageKernel_t getKernel() = 0;
            /**
             * @brief Calculating resulting rays of the interaction of the
             *        incident ray with a particle surface.
             */
            virtual void execute(size_t ray_buffer_index) = 0;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELEINE_HPP