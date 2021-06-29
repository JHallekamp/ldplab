#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_PARTICLE_INTERACTION_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_PARTICLE_INTERACTION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <memory>

#include "GenericParticleMaterial.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        // Prototype
        struct Context;
        struct KernelLaunchParameter;
        struct DevicePipelineResources;

        typedef void(*pipelineExecuteParticleInteractionStage_t)(
            DevicePipelineResources& resources,
            bool inner_particle_rays,
            size_t input_ray_buffer_index,
            size_t reflected_ray_buffer_index, 
            size_t transmitted_ray_buffer_index);

        /** @brief Abstract baseclass for ray particle interaction stage. */
        class IPipelineParticleInteractionStage
        {
        public:
            virtual ~IPipelineParticleInteractionStage() { }
            /** @brief Creates an instance of a pipeline stage implementation. */
            static std::shared_ptr<IPipelineParticleInteractionStage>
                createInstance(
                    const RayTracingStepCUDAInfo& info, Context& context);
            /** @brief Gets called before the pipeline enters execution. */
            virtual void setup() { }
            /** @brief Provides the caller with a pointer to the kernel. */
            virtual pipelineExecuteParticleInteractionStage_t getKernel() = 0;
            /**
             * @brief Calculating resulting rays of the interaction of the
             *        incident ray with a particle surface.
             */
            virtual void execute(
                bool inner_particle_rays,
                size_t input_ray_buffer_index,
                size_t reflected_ray_buffer_index,
                size_t transmitted_ray_buffer_index) = 0;
            /** @brief Returns the launch parameter for the kernel. */
            virtual KernelLaunchParameter getLaunchParameter() = 0;
        };

        /**
         * @brief Class implementing the ray particle interaction for
         *        unpolarized light and a linear index of refraction gradient
         *        in one direction.
         * @note The reflection and refraction does not lead to polarization.
         *       The model is simplified so that the light stays always
         *       unpolarized.
         */
        class PipelineParticleInteractionUnpolarized1DLinearIndexGradient :
            public IPipelineParticleInteractionStage
        {
        public:
            PipelineParticleInteractionUnpolarized1DLinearIndexGradient(
                Context& context);
            /** @brief Provides the caller with a pointer to the kernel. */
            pipelineExecuteParticleInteractionStage_t getKernel() override;
            /**
             * @brief Calculating resulting rays of the interaction of the
             *        incident ray with a particle surface.
             */
            void execute(
                bool inner_particle_rays,
                size_t input_ray_buffer_index,
                size_t reflected_ray_buffer_index,
                size_t transmitted_ray_buffer_index) override;
            KernelLaunchParameter getLaunchParameter() override;
        private:
            Context& m_context;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELEINE_PARTICLE_INTERACTION_HPP