#ifndef WWU_LDPLAB_RTSOGL_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP
#define WWU_LDPLAB_RTSOGL_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP

#include "Data.hpp"

#include <LDPLAB/RayTracingStep/RayTracingStepOpenGLInfo.hpp>
#include <LDPLAB/Geometry.hpp>

#include <memory>
#include <vector>

namespace ldplab
{
    // Prototype
    struct SimulationState;

    namespace rtsogl
    {   
        // Prototype
        struct RayBuffer;
        struct IntersectionBuffer;
        struct RodParticle;
        struct Context;

        /**
         * @brief Ray particle intersection test stage interface.
         * @detail The ray particle intersection test stage calculates the 
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
            /** @brief Initializes the shader data. */
            virtual bool initShaders() = 0;
        };
        

        /**
         * @brief Class implementing the ray particle intersection test for 
         *        rod like particle which are analytical representation. 
         *        The particle is cylindric shaped with a spherical cap and a 
         *        spherical indent at the bottom.
         */
        class RodParticleIntersectionTest :
            public IRayParticleIntersectionTestStage
        {
        public:
            RodParticleIntersectionTest(
                Context& context);
            /**
             * @brief Initializes the shader.
             * @returns true, if the initialization succeeds.
             */
            bool initShaders() override;
            /**
             * @brief Inherited via ldplab::rtsogl::IRayParticleIntersectionTestStage.  
             * @detail Start calculating the intersection points of the rays 
             *         with a particle. Missed ray are sorted out in a 
             *         secondary Ray buffer. Rays need to be in the particle 
             *         coordinate system.
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
            struct IntersectionShader {
                std::shared_ptr<ComputeShader> shader;
                GLint uniform_num_particles;
                GLint uniform_num_rays_per_buffer;
                size_t num_work_groups;
            } m_cs_intersection;
        private:
            Context& m_context;
        };
    }
}

#endif