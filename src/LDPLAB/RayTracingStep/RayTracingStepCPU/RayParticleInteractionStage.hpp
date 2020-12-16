#ifndef WWU_LDPLAB_RTSCPU_RAY_PARTICLE_INTERACTION_HPP
#define WWU_LDPLAB_RTSCPU_RAY_PARTICLE_INTERACTION_HPP

#include <memory>

namespace ldplab
{
    // Prototype
    struct SimulationState;
    namespace rtscpu
    {
        // Prototype
        struct RayBuffer;
        struct IntersectionBuffer;
        struct RodeParticle;
        class Context;
        /**
         * @brief Ray particle interaction stage interface
         * @detail The ray particle interaction stage is responsible for 
         *         calculating the resulting rays of an interaction of the 
         *         incident ray with a particle surface.
         */
        class IRayParticleInteractionStage
        {
            /**
             * @brief Calculating resulting rays of the interaction of the 
             *        incident ray with a particle surface.
             * @param[in] state Pointer to the state of the simulation.
             * @param[in] particle Index of the particle.
             * @param[in] intersection Information about the intersection point
             *              and corresponding normal.
             * @param[in,out] input_outer_rays RayBuffer holding rays that hit 
             *                  the particle surface.
             *                  The output RayBuffer holds all rays that 
             *                  propagate outside the particle after the 
             *                  interaction.
             * @param[out] inner_rays RayBuffer collecting rays that 
             *                  propagate inside the particle after the 
             *                  interaction.
             */
            virtual void execute(
                const SimulationState* state,
                const size_t particle,
                const IntersectionBuffer& intersection,
                RayBuffer& input_outer_rays,
                RayBuffer& inner_rays) = 0;
        };
        /**
         * @brief Class implementing the ray particle interaction for 
         *        unpolarized light.
         * @note The reflection and refraction does not lead to polarization.
         *       The model is simplified so that the light stays always 
         *       unpolarized.
         */
        class UnpolirzedLightInteraction : IRayParticleInteractionStage
        {
        public:
            UnpolirzedLightInteraction(
                std::shared_ptr<Context> context);
            /**
             * @brief Inherited via ldplab::rtscpu::IRayParticleInteractionStage.
             * @brief Calculating resulting rays of the interaction of the 
             *        incident ray with a particle surface.
             * @param[in] state Pointer to the state of the simulation.
             * @param[in] particle Index of the particle.
             * @param[in] intersection Information about the intersection point
             *              and corresponding normal.
             * @param[in,out] input_outer_rays RayBuffer holding rays that hit 
             *                  the particle surface.
             *                  The output RayBuffer holds all rays that 
             *                  propagate outside the particle after the 
             *                  interaction.
             * @param[out] inner_rays RayBuffer collecting rays that 
             *                  propagate inside the particle after the 
             *                  interaction.
             */
            void execute(
                const SimulationState* state,
                const size_t particle,
                const IntersectionBuffer& intersection,
                RayBuffer& input_outer_rays,
                RayBuffer& inner_rays) override;
        private:
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif