#ifndef WWU_LDPLAB_RTSGPU_OGL_RAY_BOUNDING_VOLUME_INTERSECTION_TEST_STAGE_HPP
#define WWU_LDPLAB_RTSGPU_OGL_RAY_BOUNDING_VOLUME_INTERSECTION_TEST_STAGE_HPP

#include "Data.hpp"
#include "../../RayTracingStepGPUOpenGLInfo.hpp"

#include <memory>

namespace ldplab
{
    namespace rtsgpu_ogl
    {
        // Prototype
        struct Context;

        /**
         * @brief Performs intersection tests between rays and the bounding
         *        volumes of all particles in the experimental setup.
         * @details The stage maintains a queue of ray buffers which did not
         *          intersect the particle tested in a respective ray particle
         *          intersection test. Once no rays are left being processed in
         *          the pipeline, this stage is used to create further batches.
         */
        class IRayBoundingVolumeIntersectionTestStage
        {
        public:
            /**
             * @brief Sets up the ray bounding volume intersection test stage
             *        with the current experimental setup.
             */
            virtual void setup() = 0;
            /**
             * @brief Performs the bounding volume test on the world space rays
             *        inside a given buffer.
             * @details Rays that are invalid (negative particle index) or are
             *          transformed into particle space are ignored by this 
             *          stage. Only rays in world space (positive particle
             *          index greater the total number of particles) are tested
             *          for intersections with bounding volumes.
             *          If a ray intersects a particle bounding volume then it
             *          is transformed to its particles local space and the
             *          particle index is set accordingly. Otherwise, if no
             *          particle bounding volume is intersected, the ray is
             *          set to be invalid.
             * @returns The number of rays that hit a particle bounding volume.
             */
            virtual size_t execute(RayBuffer& buffer) = 0;
        };

        /** @brief Tests the intersection between rays and bounding spheres. */
        class RayBoundingSphereIntersectionTestStageBruteForce
            : public IRayBoundingVolumeIntersectionTestStage
        {
        public:
            RayBoundingSphereIntersectionTestStageBruteForce(
                std::shared_ptr<Context> context);
            /**
             * @brief Initializes the shader.
             * @returns true, if the initialization succeeds.
             */
            bool initShaders(const RayTracingStepGPUOpenGLInfo& info);
            /** 
             * @brief Inherited via
             * ldplab::rtsgpu_ogl::IRayBoundingVolumeIntersectionTestStage 
             */
            void setup() override;
            /**
             * @brief Inherited via
             * ldplab::rtsgpu_ogl::IRayBoundingVolumeIntersectionTestStage
             */
            size_t execute(RayBuffer& buffer) override;
        private:
            std::shared_ptr<Context> m_context;
            std::shared_ptr<ComputeShader> m_compute_shader;
            GLint m_shader_uniform_location_num_rays_per_buffer;
            GLint m_shader_uniform_location_num_particles;
            size_t m_shader_num_work_groups;
        };
    }
}

#endif