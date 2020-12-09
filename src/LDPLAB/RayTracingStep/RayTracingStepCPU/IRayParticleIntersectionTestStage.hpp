#ifndef WWU_LDPLAB_RTSCPU_IRAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_IRAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP

#include "Context.hpp"

namespace ldplab
{
    namespace rtscpu
    {
        // Prototype
        struct RayBuffer;

        class IRayParticleIntersectionTestStage
        {
        public:
            IRayParticleIntersectionTestStage(
                std::shared_ptr<Context> context);
            /**
             * @brief Start calculating the intersection points of the rays with a
             *        particle. Missed ray are sorted out in a secondary Ray buffer.
             * @param[in] particle Index of the particle.
             * @param[in,out] input_hit_rays RayBuffer holding rays that hit the
             *                               particle bounding box.
             *                               The output RayBuffer holds all rays
             *                               that hit the particle.
             * @param[out] missed_rays RayBuffer collecting rays that missed the
             *                         particle.
             */
            virtual void execute(
                size_t particle,
                RayBuffer& input_hit_rays,
                RayBuffer& missed_rays) = 0;
        private:
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif