#ifndef WWU_LDPLAB_RTSCUDA_PIPELINE_INNER_PARTICLE_PROPAGATION_HPP
#define WWU_LDPLAB_RTSCUDA_PIPELINE_INNER_PARTICLE_PROPAGATION_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepCUDAInfo.hpp>
#include <memory>

#include "GenericParticleGeometry.hpp"
#include "GenericParticleMaterial.hpp"

namespace ldplab
{
    namespace rtscuda
    {
        // Prototype
        struct Context;
        struct KernelLaunchParameter;
        struct DevicePipelineResources;

        typedef void(*pipelineExecuteInnerParticlePropagationStage_t)(
            DevicePipelineResources& resources,
            size_t ray_buffer_index);

        /** @brief Abstract baseclass for the inner particle propagation. */
        class IPipelineInnerParticlePropagation
        {
        public:
            virtual ~IPipelineInnerParticlePropagation() { }
            /** @brief Creates an instance of a pipeline stage implementation. */
            static std::shared_ptr<IPipelineInnerParticlePropagation> 
                createInstance(
                    const RayTracingStepCUDAInfo& info, Context& context);
            /** @brief Gets called once after instance creation. */
            virtual bool allocate() { return true; }
            /** @brief Gets called before the pipeline enters execution. */
            virtual void setup() { }
            /** @brief Provides the caller with a pointer to the kernel. */
            virtual pipelineExecuteInnerParticlePropagationStage_t
                getKernel() = 0;
            /** @brief Calculating the path of the rays threw the particle. */
            virtual void execute(size_t ray_buffer_index) = 0;
            /** @brief Returns the launch parameter for the kernel. */
            virtual KernelLaunchParameter getLaunchParameter() = 0;
        };

        /**
         * @brief Class implementing the inner particle propagation for
         *        linear index of refraction gradient in one direction.
         * @detail The light propagation is calculated by solving the Eikonal
         *         equation with the RungeKutta method.
         */
        class PipelineInnerParticlePropagationRK4LinearIndexGradient :
            public IPipelineInnerParticlePropagation
        {
        public:
            PipelineInnerParticlePropagationRK4LinearIndexGradient(
                Context& context,
                RK4Parameter parameter);
            pipelineExecuteInnerParticlePropagationStage_t getKernel() override;
            void execute(size_t ray_buffer_index) override;
            KernelLaunchParameter getLaunchParameter() override;
            bool allocate() override;
        public:
            /**
             * @brief Structure keeping all variables of the differential
             *        equation.
             */
            struct Arg
            {
                /**
                 * @brief Vector pointing in the direction of light. Its norm
                 *        is the index of reflection at position r.
                 */
                Vec3 w;
                /**
                 * @brief Vector pointing to the light rays origin.
                 */
                Vec3 r;
            };
        private:
            Context& m_context;
            const RK4Parameter m_parameters;
        };
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_PIPELEINE_INNER_PARTICLE_PROPAGATION_HPP