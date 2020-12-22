#ifndef WWU_LDPLAB_RTSCPU_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP
#define WWU_LDPLAB_RTSCPU_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP

#include "../../Geometry.hpp"

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
             * @param[in,out] input_hit_rays RayBuffer holding rays that hit the
             *                               particle bounding box.
             *                               The output RayBuffer holds all rays
             *                               that hit the particle.
             * @param[out] missed_rays RayBuffer collecting rays that missed the
             *                         particle.
             * @param[out] intersection IntersectionBuffer holding information 
                         about the intersection points.
             */
            virtual void execute(
                const SimulationState* state,
                const size_t particle,
                RayBuffer& input_hit_rays,
                RayBuffer& missed_rays,
                IntersectionBuffer& intersection) = 0;
        };
        

        /**
         * @brief Class implementing the ray particle intersection test for 
         *        rode like particle which are analytical representation. 
         *        The particle is cylindric shaped with a spherical cap and a 
         *        spherical indent at the bottom.
         */
        class RodeParticleIntersectionTest :
            public IRayParticleIntersectionTestStage
        {
        public:
            RodeParticleIntersectionTest(
                std::shared_ptr<Context> context);
            /**
             * @brief Inherited via ldplab::rtscpu::IRayParticleIntersectionTestStage.  
             * @detail Start calculating the intersection points of the rays 
             *         with a particle. Missed ray are sorted out in a 
             *         secondary Ray buffer. Rays need to be in the particle 
             *         coordinate system.
             * @param[in] state Pointer to the state of the simulation.
             * @param[in] particle Index of the particle.
             * @param[in,out] input_hit_rays RayBuffer holding rays that hit the
             *                               particle bounding box.
             *                               The output RayBuffer holds all rays
             *                               that hit the particle.
             * @param[out] missed_rays RayBuffer collecting rays that missed the
             *                         particle.
             * @param[out] intersection IntersectionBuffer holding information 
             *                          about the intersection points.
             */
            void execute(
                const SimulationState* state,
                const size_t particle,
                RayBuffer& input_hit_rays,
                RayBuffer& missed_rays,
                IntersectionBuffer& intersection) override;
        private:
            std::shared_ptr<Context> m_context;
        private:
            /**
             * @brief Testing for a intersection of a single ray with a 
             *        particle.
             */
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
             *         false will be returned.
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
             *         false will be returned.
             */
            bool sphereIntersection(
                const Vec3& origin,
                const double& raduis,
                const Ray& ray,
                double& distance_min,
                double& distance_max);

            /**
             * @brief Calculate the intersection point of a ray and the
             *        spherical cap of the rode particle.
             * @note The intersection point is always the point with the
             *       minimum distance from the ray origin.
             * @warning Only the correct hight of the intersection point is 
             *          checked. It is assumed that the intersection point is 
             *          inside the infinite cylinder.
             * @param[in] particle_geometrie Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] intersection_point Resulting intersection point with
             *                                the cap.
             * @param[out] intersection_normal Resulting normal of the particle
             *                                 surface at the intersection
             *                                 point. The normal is pointing
             *                                 outside of the particle.
             * @retuns true if the sphere and the ray are intersecting, else
             *         false will be returned.
             */
            bool capIntersection(
                const RodeParticle& particle_geometrie,
                const Ray& ray,
                Vec3& inter_point,
                Vec3& inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the 
             *        spherical indentation of the rode particle.
             * @note The intersection point is always the point with the 
             *       maximum distance from the ray origin (including negative 
             *       distances).
             * @warning No further checking is done if the intersection point 
             *          is inside the cylinder.
             * @param[in] particle Specifies the particle geometry.
             * @param[in] ray Specifies the ray.
             * @param[out] intersection_point Resulting intersection point with
             *                                the indentation.
             * @param[out] intersection_normal Resulting normal of the particle 
             *                                 surface at the intersection 
             *                                 point. The normal is pointing 
             *                                 outside of the particle.
             * @retuns true if the sphere and the ray are intersecting, else
             *         false will be returned.
             */
            bool indentationIntersection(
                const RodeParticle& particle_geometrie,
                const Ray& ray,
                Vec3& intersection_point,
                Vec3& intersection_normal);

            
        };
    }
}

#endif