#ifndef WWU_LDPLAP_RTSCPU_IINTERSECTIONTEST_HPP
#define WWU_LDPLAP_RTSCPU_IINTERSECTIONTEST_HPP

#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    namespace rtscpu
        {
        /**
         * @brief Interface for particle intersection tests.
         */
        class IIntersectionTest
        {
        public:
            /**
             * @brief Calculating the intersection point from a ray and 
             *        particle shell.
             * @param[in] ray Light ray
             * @param[out] intersection_point
             * @param[out] intersection_normal
             * @return True if the ray intersects the particle. Else false will
             *         be returned.
             */
            bool intersection(const Ray& ray,
                Vec3 intersection_point, 
                Vec3 intersection_normal);
            /**
             * @brief Calculating the intersection point from a ray segment and 
             *        particle shell.
             * @details The start point of the ray needs to be inside the particle.
             * @param[in] start Starting point of the ray.
             * @param[in] end End point of the ray segment.
             * @param[out] intersection_point Resulting intersection point with
             *                                the particle surface.
             * @param[out] intersection_normal Resulting normal of the particle
             *                                 surface at the intersection
             *                                 point.
             * @return True if the ray intersects the particle. Else false will
             *         be returned.
             */
            bool InnerParticleIntersection( 
                const Vec3& start,
                const Vec3& end,
                Vec3& intersection_point,
                Vec3& intersection_normal);
        };
    }
}
#endif