#ifndef WWU_LDPLAB_RTSGPU_OGL_INNER_PARTICLE_PROPAGATION_STAGE_HPP
#define WWU_LDPLAB_RTSGPU_OGL_INNER_PARTICLE_PROPAGATION_STAGE_HPP

#include <memory>
#include <vector>

#include "Data.hpp"
#include <LDPLAB/RayTracingStep/EikonalSolver.hpp>
#include <LDPLAB/RayTracingStep/RayTracingStepGPUOpenGLInfo.hpp>
#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    // Prototype
    struct Particle;
    struct ParticleMaterialLinearOneDirectional;

    namespace rtsgpu_ogl
    {
        // Prototype
        struct Context;
        struct RayBuffer;
        struct IntersectionBuffer;
        struct OutputBuffer;
        struct RodParticle;

        /**
         * @brief Inner particle propagation stage interface
         * @detail The inner particle propagation stage is responsible for
         *         calculating the path threw the particle and the resulting
         *         momentum difference.
         */
        class IInnerParticlePropagationStage
        {
        public:
            /**
             * @brief Calculating the path of the rays threw the particle.
             * @param[in, out] rays RayBuffer holding the propagating rays.
             * @param[out] intersection IntersectionBuffer holding information 
             *             about the intersection points.
             * @param[in, out] output Buffer holding the resulting force and 
             *                        torque change of each particle.
             */
            virtual void execute(
                RayBuffer& rays,
                IntersectionBuffer& intersection,
                OutputBuffer& output) = 0;
            /** @brief Initializes the shader data. */
            virtual bool initShaders() = 0;
        };

        /**
         * @brief Class implementing the inner particle propagation for 
         *        linear index of refraction gradient in one direction.
         * @detail The light propagation is calculated by solving the Eikonal 
         *         equation with the Runge–Kutta–Fehlberg(45) method.
         */
        class LinearIndexGradientRodParticlePropagation
            : public IInnerParticlePropagationStage
        {
        public:
            /**
             * @brief Constructing inner particle propagation stage and setting 
             *        up the parameter for the Runge-Kutta-Fehlberg method.
             * @param context Pointer to context data for the ray tracing step.
             * @param parameters Structure containing all parameter for the 
             *                   Runge-Kutta-Fehlberg method.
             */
            LinearIndexGradientRodParticlePropagation(
                std::shared_ptr<Context> context,
                RK45 parameters);
            /**
             * @brief Initializes the shader.
             * @returns true, if the initialization succeeds.
             */
            bool initShaders() override;
            /**
             * @brief Inherited via ldplab::rtsgpu_ogl::IInnerParticlePropagationStage.
             * @details Calculating the path of the rays threw the particle.
             * @param[in, out] rays RayBuffer holding the propagating rays.
             * @param[out] intersection IntersectionBuffer holding information 
             *             about the intersection points.
             * @param[in, out] output Buffer holding the resulting force and torque
             *                        change of each particle.
             */
            void execute(
                RayBuffer& rays,
                IntersectionBuffer& intersection,
                OutputBuffer& output) override;
        private:
            struct InnerParticlePropagationShader {
                std::shared_ptr<ComputeShader> shader;
                GLint uniform_num_rays_per_buffer;
                GLint uniform_parameter_initial_step_size;
                GLint uniform_parameter_epsilon;
                GLint uniform_parameter_safety_factor;
                size_t num_work_groups;
            } m_cs_inner_particle_propagation;
        private:
            const RK45 m_parameters;
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif