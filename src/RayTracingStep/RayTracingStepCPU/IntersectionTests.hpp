#ifndef WWU_LDPLAB_RTSCPU_INTERSECTION_TESTS_HPP
#define WWU_LDPLAB_RTSCPU_INTERSECTION_TESTS_HPP

#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    namespace rtscpu
    {
        /** @brief Contains static methods for various intersection tests. */
        class IntersectionTest
        {
        public:
            /** @brief No instances of this class allowed. */
            IntersectionTest() = delete;
            /** 
             * @brief Computes the intersection between a ray and a triangle.
             * @param[in] ray Ray for which the test is executed.
             * @param[in] triangle Triangle for which the test is executed.
             * @param[out] intersection_point If ray and triangle intersect,
             *                                intersection_point will contain
             *                                the point of intersection.
             * @param[out] intersection_normal If ray and triangle intersect,
             *                                 intersection_normal will contain
             *                                 the triangle normal at the point
             *                                 of intersection (facing to the 
             *                                 side of the ray origin).
             * @param[out] dist If ray and triangle intersect, dist will 
             *                  contain the distance from the ray origin to the
             *                  intersection point.
             */
            static inline bool rayTriangle(
                const Ray& ray,
                const Triangle& triangle,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist);
            /** @todo documentation */
            static inline bool lineTriangle(
                const Vec3& line_start,
                const Vec3& line_end,
                const Triangle& triangle,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist);
            /** @todo documentation */
            static inline bool raySphere(
                const Ray& ray,
                const Vec3& sphere_center,
                const double sphere_radius,
                double& min_dist,
                double& max_dist);
            /** @todo documentation */
            static inline bool raySphere(
                const Ray& ray,
                const Vec3& sphere_center,
                const double sphere_radius,
                Vec3& min_intersection_point,
                Vec3& min_intersection_normal,
                double& min_dist,
                Vec3& max_intersection_point,
                Vec3& max_intersection_normal,
                double& max_dist);
            /** @todo documentation */
            static inline bool raySphereOnlyMin(
                const Ray& ray,
                const Vec3& sphere_center,
                const double sphere_radius,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist);
            /** @todo documentation */
            static inline bool raySphereOnlyMax(
                const Ray& ray,
                const Vec3& sphere_center,
                const double sphere_radius,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist);
        };
    }
}

#endif