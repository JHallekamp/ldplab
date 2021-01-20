#ifndef WWU_LDPLAB_RTSGPU_OGL_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP
#define WWU_LDPLAB_RTSGPU_OGL_RAY_PARTICLE_INTERSECTION_TEST_STAGE_HPP

#include "Data.hpp"
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
            /**
             * @brief Testing for a intersection of a single ray with a 
             *        particle.
             * @param[in] particle Specifies the particle geometry.
             * @param[in] ray_origin Specifies the ray origin.
             * @param[in] ray_direction Specifies the ray direction.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle 
             *                          surface at the intersection 
             *                          point. The normal is pointing 
             *                          outside of the particle.
             */
            bool intersectionTest(
                const RodParticle& geometry,
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                Vec3& inter_point,
                Vec3& inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the cylinder.
             * @param[in] particle Specifies the particle geometry.
             * @param[in] ray_origin Specifies the ray origin.
             * @param[in] ray_direction Specifies the ray direction.
             * @param[out] distance_min Resulting distance between the origin
             *                      of the ray and the intersection point.
             * @param[out] distance_max Resulting distance between the origin
             *                      of the ray and the intersection point.
             * @retuns true if the cylinder and the ray are intersecting, else
             *         false will be returned.
             */
            bool cylinderIntersection(
                const RodParticle& particle,
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                double& distance_min,
                double& distance_max);
            /**
             * @brief Calculates whether a ray intersects the particle at the
             *        cap or indentation under the assumption that the ray is
             *        within the infinite cylinder.
             * @param[in] particle Specifies the particle geometry.
             * @param[in] ray_origin Specifies the ray origin.
             * @param[in] ray_direction Specifies the ray direction.
             * @param[out] inter_point Resulting intersection point with
             *                         the particle surface.
             * @param[out] inter_normal Resulting normal of the particle
             *                          surface at the intersection
             *                          point. The normal is pointing
             *                          outside of the particle.
             * @returns true if the ray intersects the particle at the bottom
             *          or top.
             */
            bool bottomTopIntersection(
                const RodParticle& particle,
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                Vec3& inter_point,
                Vec3& inter_normal);

            /**
             * @brief Calculate the intersection point of a ray and the sphere.
             * @param[in] origin Specifies the origin of the sphere.
             * @param[in] radius Specifies the radius of the sphere.
             * @param[in] ray_origin Specifies the ray origin.
             * @param[in] ray_direction Specifies the ray direction.
             * @param[out] distance_min Resulting distance between the origin
             *                      of the ray and the intersection point.
             * @param[out] distance_max Resulting distance between the origin
             *                      of the ray and the intersection point.
             * @retuns true if the sphere and the ray are intersecting, else
             *         false will be returned.
             */
            bool sphereIntersection(
                const Vec3& sphere_origin,
                const double& sphere_raduis,
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                double& distance_min,
                double& distance_max);

            /**
             * @brief Calculate the intersection point of a ray and the
             *        spherical cap of the rod particle.
             * @note The intersection point is always the point with the
             *       minimum distance from the ray origin.
             * @warning Only the correct hight of the intersection point is 
             *          checked. It is assumed that the intersection point is 
             *          inside the infinite cylinder.
             * @param[in] geometry Specifies the particle geometry.
             * @param[in] ray_origin Specifies the ray origin.
             * @param[in] ray_direction Specifies the ray direction.
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
                const RodParticle& geometry,
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                Vec3& inter_point,
                Vec3& inter_normal);
            /**
             * @brief Calculate the intersection point of a ray and the 
             *        spherical indentation of the rod particle.
             * @note The intersection point is always the point with the 
             *       maximum distance from the ray origin (including negative 
             *       distances).
             * @warning No further checking is done if the intersection point 
             *          is inside the cylinder.
             * @param[in] geometry Specifies the particle geometry.
             * @param[in] ray_origin Specifies the ray origin.
             * @param[in] ray_direction Specifies the ray direction.
             * @param[out] inter_point Resulting intersection point with
             *                                the indentation.
             * @param[out] inter_normal Resulting normal of the particle 
             *                                 surface at the intersection 
             *                                 point. The normal is pointing 
             *                                 outside of the particle.
             * @retuns true if the sphere and the ray are intersecting, else
             *         false will be returned.
             */
            bool indentationIntersection(
                const RodParticle& geometry,
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                Vec3& inter_point,
                Vec3& inter_normal);
        private:
            std::shared_ptr<Context> m_context;
            std::vector<RodParticle>& m_rod_particles;
        };
    }
}

#endif