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
            int32_t* ray_index_buffer,
            Vec3* ray_origin_buffer,
            Vec3* ray_direction_buffer,
            int32_t* intersection_index_buffer,
            Vec3* intersection_point_buffer,
            Vec3* intersection_normal_buffer,
            size_t num_rays_per_batch,
            GenericParticleGeometryData* geometry_per_particle,
            Mat3* p2w_transformation,
            Vec3* p2w_translation,
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

        /** @brief Basic intersection test for generic geometry. */
        class PipelineParticleIntersectionGenericParticleGeometry :
            public IPipelineParticleIntersectionStage
        {
        public:
            PipelineParticleIntersectionGenericParticleGeometry(
                Context& context);
            /** @brief Provides the caller with a pointer to the kernel. */
            pipelineParticleIntersectionStageKernel_t getKernel() override;
            /**
             * @brief Calculating resulting rays of the interaction of the
             *        incident ray with a particle surface.
             */
            void execute(size_t ray_buffer_index) override;
        private:
            static __global__ void intersectionKernel(
                int32_t* ray_index_buffer,
                Vec3* ray_origin_buffer,
                Vec3* ray_direction_buffer,
                int32_t* intersection_index_buffer,
                Vec3* intersection_point_buffer,
                Vec3* intersection_normal_buffer,
                size_t num_rays_per_batch,
                GenericParticleGeometryData* geometry_per_particle,
                Mat3* p2w_transformation,
                Vec3* p2w_translation,
                size_t num_particles);
            static __device__ pipelineParticleIntersectionStageKernel_t
                intersection_kernel_ptr;
        private:
            Context& m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELEINE_HPP