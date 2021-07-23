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
        struct KernelLaunchParameter; 
        struct DevicePipelineResources;

        typedef void(*pipelineExecuteParticleIntersectionStage_t)(
            DevicePipelineResources& resources,
            size_t ray_buffer_index);

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
            virtual pipelineExecuteParticleIntersectionStage_t getKernel() = 0;
            /**
             * @brief Calculating resulting rays of the interaction of the
             *        incident ray with a particle surface.
             */
            virtual void execute(size_t ray_buffer_index) = 0;
            /** @brief Returns the launch parameter for the kernel. */
            virtual KernelLaunchParameter getLaunchParameter() = 0;
        };

        /** @brief Basic intersection test for generic geometry. */
        class PipelineParticleIntersectionGenericParticleGeometry :
            public IPipelineParticleIntersectionStage
        {
        public:
            PipelineParticleIntersectionGenericParticleGeometry(
                Context& context);
            /** @brief Provides the caller with a pointer to the kernel. */
            pipelineExecuteParticleIntersectionStage_t getKernel() override;
            /**
             * @brief Calculating resulting rays of the interaction of the
             *        incident ray with a particle surface.
             */
            void execute(size_t ray_buffer_index) override;
            KernelLaunchParameter getLaunchParameter() override;
        private:
            Context& m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELEINE_HPP