#ifndef WWU_LDPLAB_RTSCUDA_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP
#define WWU_LDPLAB_RTSCUDA_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP

#include "Data.hpp"
#include <LDPLAB/Geometry.hpp>

#include <memory>
#include <vector>

namespace ldplab
{
    // Prototype
    struct SimulationState;
    struct SphericalParticleGeometry;

    namespace rtscuda
    {   
        // Prototype
        struct RayBuffer;
        struct IntersectionBuffer;
        struct RodParticle;
        struct Context;
        /**
         * @brief Ray particle intersection test stage interface.
         * @details The ray particle intersection test stage calculates the 
         *         intersection point of the ray with the particle and its 
         *         corresponding normal. Ray missing the particle are sorted 
         *         out to a new buffer.
         */
        class IRayParticleIntersectionTestStage
        {
        public:
            /**
             * @brief Start calculating the intersection points of the rays with a
             *        particle. Missed ray are sorted out in a secondary Ray buffer.
             * @param[in] state Pointer to the state of the simulation.
             * @param[in] particle Index of the particle.
             * @param[in, out] rays RayBuffer holding rays that hit the
             *                 particle bounding box. Rays that miss the 
             *                 particle will be transformed back to world space.
             * @param[out] intersection IntersectionBuffer holding information 
                         about the intersection points.
             */
            virtual void execute(
                RayBuffer& rays,
                IntersectionBuffer& intersection) = 0;
        };

        /**
         * @brief Class implementing the ray particle intersection test for
         *        generic geometry interfaces.
         */
        class RayParticleGenericGeometryIntersectionTest :
            public IRayParticleIntersectionTestStage
        {
        public:
            RayParticleGenericGeometryIntersectionTest(
                Context& context);
            /**
             * @brief Inherited via ldplab::rtscuda::IRayParticleIntersectionTestStage.
             * @details Start calculating the intersection points of the rays
             *          with a particle. Missed ray are sorted out in a
             *          secondary Ray buffer. Rays need to be in the particle
             *          coordinate system.
             * @param[in] state Pointer to the state of the simulation.
             * @param[in] particle Index of the particle.
             * @param[in, out] rays RayBuffer holding rays that hit the
             *                 particle bounding box. Rays that miss the
             *                 particle will be transformed back to world space.
             * @param[out] intersection IntersectionBuffer holding information
             *                          about the intersection points.
             */
            void execute(
                RayBuffer& rays,
                IntersectionBuffer& intersection) override;
        private:
            Context& m_context;
        };

    }
}

#endif