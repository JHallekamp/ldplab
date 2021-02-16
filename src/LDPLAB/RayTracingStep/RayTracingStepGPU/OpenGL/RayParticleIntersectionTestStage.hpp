#ifndef WWU_LDPLAB_RTSGPU_OGL_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP
#define WWU_LDPLAB_RTSGPU_OGL_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP

#include "Data.hpp"

#include "../../RayTracingStepGPUOpenGLInfo.hpp"
#include "../../../Geometry.hpp"

#include <memory>
#include <vector>

namespace ldplab
{
    // Prototype
    struct SimulationState;

    namespace rtsgpu_ogl
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
                std::shared_ptr<Context> context);
            /**
             * @brief Initializes the shader.
             * @returns true, if the initialization succeeds.
             */
            bool initShaders(const RayTracingStepGPUOpenGLInfo& info);
            /**
             * @brief Inherited via ldplab::rtsgpu_ogl::IRayParticleIntersectionTestStage.  
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
            std::shared_ptr<Context> m_context;
            std::vector<RodParticle>& m_rod_particles;
            std::shared_ptr<ComputeShader> m_compute_shader;
            GLint m_shader_uniform_location_num_rays_per_buffer;
            GLint m_shader_uniform_location_num_particles;
        };
    }
}

#endif