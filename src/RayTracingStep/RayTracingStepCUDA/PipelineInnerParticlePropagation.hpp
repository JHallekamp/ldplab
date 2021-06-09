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

        /** @brief Typedefinition of inner particle propagation stage. */
        typedef void (*pipelineInnerParticlePropagationStageKernel_t)(
            double step_size,
            int32_t* ray_index_buffer,
            Vec3* ray_origin_buffer,
            Vec3* ray_direction_buffer,
            double* ray_intensity_buffer,
            Vec3* intersection_point_buffer,
            Vec3* intersection_normal_buffer,
            size_t num_rays_per_batch,
            GenericParticleGeometryData* geometry_per_particle,
            GenericParticleMaterialData* material_per_particle,
            Vec3* particle_center_of_mass,
            Vec3* output_force_per_ray,
            Vec3* output_torque_per_ray,
            size_t num_particles);

        /** @brief Abstract baseclass for the inner particle propagation. */
        class IPipelineInnerParticlePropagation
        {
        public:
            virtual ~IPipelineInnerParticlePropagation() { }
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
            pipelineInnerParticlePropagationStageKernel_t getKernel() override;
            void execute(size_t ray_buffer_index) override;
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