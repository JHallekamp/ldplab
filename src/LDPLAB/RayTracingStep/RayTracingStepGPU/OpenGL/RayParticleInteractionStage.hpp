#ifndef WWU_LDPLAB_RTSGPU_OGL_RAY_PARTICLE_INTERACTION_HPP
#define WWU_LDPLAB_RTSGPU_OGL_RAY_PARTICLE_INTERACTION_HPP

#include "../../RayTracingStepGPUOpenGLInfo.hpp"
#include <memory>

namespace ldplab
{
    // Prototype
    struct SimulationState;
    namespace rtsgpu_ogl
    {
        // Prototype
        struct RayBuffer;
        struct IntersectionBuffer;
        struct OutputBuffer;
        struct RodParticle;
        struct Context;

        /**
         * @brief Ray particle interaction stage interface
         * @detail The ray particle interaction stage is responsible for 
         *         calculating the resulting rays of an interaction of the 
         *         incident ray with a particle surface.
         */
        class IRayParticleInteractionStage
        {
        public:
            /**
             * @brief Calculating resulting rays of the interaction of the 
             *        incident ray with a particle surface.
             * @param[in] intersection Information about the intersection point
             *              and corresponding normal.
             * @param[in] rays RayBuffer holding rays that hit 
             *                 the particle surface.
             * @param[out] reflected_rays RayBuffer holds all rays that are 
             *                            reflected from the particle surface. 
             * @param[out] refracted_rays RayBuffer holds all rays that are 
             *                            refracted from the particle surface. 
             * @param[in, out] output Buffer holding the resulting force and torque
             *                    change of each particle.
             */
            virtual void execute(
                const IntersectionBuffer& intersection,
                const RayBuffer& rays,
                RayBuffer& reflected_rays,
                RayBuffer& refracted_rays,
                OutputBuffer& output) = 0;
        };
        /**
         * @brief Class implementing the ray particle interaction for 
         *        unpolarized light and a linear index of refraction gradient 
         *        in one direction. 
         * @note The reflection and refraction does not lead to polarization.
         *       The model is simplified so that the light stays always 
         *       unpolarized.
         */
        class UnpolirzedLight1DLinearIndexGradientInteraction : 
            public IRayParticleInteractionStage
        {
        public:
            UnpolirzedLight1DLinearIndexGradientInteraction(
                std::shared_ptr<Context> context);
            /**
             * @brief Initializes the shader.
             * @returns true, if the initialization succeeds.
             */
            bool initShaders(const RayTracingStepGPUOpenGLInfo& info);
            /**
             * @brief Inherited via ldplab::rtsgpu_ogl::IRayParticleInteractionStage.
             * @brief Calculating resulting rays of the interaction of the 
             *        incident ray with a particle surface.
             * @param[in] intersection Information about the intersection point
             *              and corresponding normal.
             * @param[in] rays RayBuffer holding rays that hit 
             *                 the particle surface.
             * @param[out] reflected_rays RayBuffer holds all rays that are 
             *                            reflected from the particle surface. 
             * @param[out] refracted_rays RayBuffer holds all rays that are 
             *                            refracted from the particle surface. 
             * @param[in, out] output Buffer holding the resulting force and torque
             *                    change of each particle.
             */
            void execute(
                const IntersectionBuffer& intersection,
                const RayBuffer& rays,
                RayBuffer& reflected_rays,
                RayBuffer& refracted_rays,
                OutputBuffer& output) override;
        private:
            std::shared_ptr<Context> m_context;
            std::shared_ptr<ComputeShader> m_compute_shader;
            GLint m_shader_uniform_location_num_rays_per_buffer;
            GLint m_shader_uniform_location_num_particles;
            GLint m_shader_uniform_location_parameter_medium_reflection_index;
            GLint m_shader_uniform_location_parameter_intensity_cutoff;
            GLint m_shader_uniform_location_inner_particle_rays;
        };
    }
}

#endif