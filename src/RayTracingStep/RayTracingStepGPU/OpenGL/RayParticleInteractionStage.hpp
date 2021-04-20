#ifndef WWU_LDPLAB_RTSGPU_OGL_RAY_PARTICLE_INTERACTION_HPP
#define WWU_LDPLAB_RTSGPU_OGL_RAY_PARTICLE_INTERACTION_HPP

#include <LDPLAB/RayTracingStep/RayTracingStepGPUOpenGLInfo.hpp>
#include "Data.hpp"

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
            /** @brief Initializes the shader data. */
            virtual bool initShaders() = 0;
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
                Context& context);
            /**
             * @brief Initializes the shader.
             * @returns true, if the initialization succeeds.
             */
            bool initShaders() override;
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
            /** @brief Structure holding data for the interaction shader. */
            struct InteractionShader {
                std::shared_ptr<ComputeShader> shader;
                GLint uniform_inner_particle_rays;
                GLint uniform_num_rays_per_buffer;
                GLint uniform_parameter_medium_reflection_index;
                GLint uniform_parameter_intensity_cutoff;
                size_t num_work_groups;
            } m_cs_interaction;
            /** @brief Structure holding data for the gather output shader. */
            struct GatherOutputShaderPreStage {
                std::shared_ptr<ComputeShader> shader;
                GLint uniform_num_rays_per_buffer;
                GLint uniform_num_particles;
                size_t num_work_groups;
            } m_cs_gather_output_pre_stage;
            /** @brief Structure holding data for the gather output shader. */
            struct GatherOutputShaderPostStage {
                std::shared_ptr<ComputeShader> shader;
                GLint uniform_buffer_size;
                GLint uniform_num_particles;
                GLint uniform_source_offset;
            } m_cs_gather_output_reduction_stage;
            /** @brief Structure holding data for the gather output shader. */
            struct GatherOutputShaderReduceStage {
                std::shared_ptr<ComputeShader> shader;
                GLint uniform_num_particles;
                size_t num_work_groups;
            } m_cs_gather_output_post_stage;
        private:
            Context& m_context;
        };
    }
}

#endif