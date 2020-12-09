#ifndef WWU_LDPLAB_RTSCPU_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP

#include "RayTracingStepCPU.hpp"
#include "Context.hpp"
#include "IRayParticleIntersectionTestStage.hpp"

#include <memory>

namespace ldplab
{
    namespace rtscpu
    {   
        // Prototype
        struct RayBuffer;

        class RodeParticleIntersectionTest :
            IRayParticleIntersectionTestStage
        {
        public:
            RodeParticleIntersectionTest(
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
            void execute(
                size_t particle,
                RayBuffer& input_hit_rays,
                RayBuffer& missed_rays);
        private:
            std::shared_ptr<Context> m_context;
        };
    }
}

#endif