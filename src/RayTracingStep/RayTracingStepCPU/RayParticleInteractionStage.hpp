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
                Context& context);
            /**
             * @brief Inherited via ldplab::rtscpu::IRayParticleInteractionStage.
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
            /** 
             * @brief Calculates the reflectance coefficient for unpolarized 
             *        light.
             * @param cos_alpha Cosine of the incidence angle.
             * @param cos_beta Cosine of the angle of refraction
             * @param ratio of the index of reflection
             */
            double reflectance(double cos_alpha, double cos_beta, double n_r);
        private:
            Context& m_context;
        
        };
    }
}

#endif