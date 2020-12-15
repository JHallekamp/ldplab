#ifndef WWU_LDPLAB_RTSCPU_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP

#include "../../SimulationState.hpp"

#include <memory>

namespace ldplab
{
    namespace rtscpu
    {   
        // Prototype
        struct RayBuffer;
        struct IntersectionBuffer;
        struct RodeParticle;
        class Context;

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
        

        /**
         * @brief Class calculating the intersection of rays with a rode like
         *        particle. The particle is cylindric shaped with a 
         *        spherical cap and a spherical indent at the bottom.
         */
        class RodeParticleIntersectionTest :
            IRayParticleIntersectionTestStage
        {
        public:
            RodeParticleIntersectionTest(
                std::shared_ptr<Context> context);
            /**
             * @brief Start calculating the intersection points of the rays with a
             *        particle. Missed ray are sorted out in a secondary Ray buffer.
             * @param[in] state Pointer to the state of the simulation.
             * @param[in] particle Index of the particle.
             * @param[in,out] input_hit_rays RayBuffer holding rays that hit the
             *                               particle bounding box.
             *                               The output RayBuffer holds all rays
             *                               that hit the particle.
             * @param[out] missed_rays RayBuffer collecting rays that missed the
             *                         particle.
             */
            void execute(
                const SimulationState* state,
                const size_t particle,
                RayBuffer& input_hit_rays,
                RayBuffer& missed_rays,
                IntersectionBuffer& intersection);
        private:
            std::shared_ptr<Context> m_context;
        private:
            bool intersectionTest(
                const Ray& ray,
                const RodeParticle& particle_geometrie,
                Vec3& inter_point,
                Vec3& inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the cylinder.
             * @param[in] particle Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] distance_min Resulting distance between the origin
             *                      of the ray and the intersection point.
             * @param[out] distance_max Resulting distance between the origin
             *                      of the ray and the intersection point.
             * @retuns true if the cylinder and the ray are intersecting, else 
             *         it returns false.
             */
            bool cylinderIntersection(
                const RodeParticle& particle,
                const Ray& ray,
                double& distance_min,
                double& distance_max);
            /**
             * @brief Calculate the intersection point of a ray and the sphere.
             * @param[in] origin Specifies the origin of the sphere.
             * @param[in] radius Specifies the radius of the sphere.
             * @param[in] ray Specifies the ray.
             * @param[out] distance_min Resulting distance between the origin
             *                      of the ray and the intersection point.
             * @param[out] distance_max Resulting distance between the origin
             *                      of the ray and the intersection point.
             * @retuns true if the sphere and the ray are intersecting, else 
             *         it returns false.
             */
            bool sphereIntersection(
                const Vec3& origin,
                const double& raduis,
                const Ray& ray,
                double& distance_min,
                double& distance_max);

            bool capIntersection(
                const RodeParticle& particle_geometrie,
                const Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal);
            bool indentationIntersection(
                const RodeParticle& particle_geometrie,
                const Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal);

            
        };
    }
}

#endif