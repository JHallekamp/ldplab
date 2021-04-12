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
             * @returns true, if the ray intersects the triangle.
             */
            static inline bool rayTriangle(
                const Ray& ray,
                const Triangle& triangle,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist);
            /**
             * @brief Computes the intersection between a line segment and a
             *        triangle.
             * @param[in] line_start Starting point of the line segment.
             * @param[in] line_end Ending point of the line segment.
             * @param[in] triangle Triangle for which the test is executed.
             * @param[out] intersection_point If line and triangle intersect,
             *                                intersection_point will contain
             *                                the point of intersection.
             * @param[out] intersection_normal If line and triangle intersect,
             *                                 intersection_normal will contain
             *                                 the triangle normal at the point
             *                                 of intersection (facing to the 
             *                                 side of the line origin).
             * @param[out] dist If the line and triangle intersect, dist will 
             *                  contain the distance from the line origin to
             *                  the intersection point.
             * @returns true, if the line segment intersects the triangle.
             */
            static inline bool lineTriangle(
                const Vec3& line_start,
                const Vec3& line_end,
                const Triangle& triangle,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist);
            /**
             * @brief Computes the intersection between a ray and a sphere.
             * @param[in] ray Ray for which the intersection test is executed.
             * @param[in] sphere_center The center of the sphere.
             * @param[in] sphere_radius The radius of the sphere.
             * @param[out] min_dist As a ray and sphere can have at most two
             *                      intersection points, min_dist contains the
             *                      minimal distance from both distances to
             *                      these points. Note that this can also be
             *                      a negative value, if the ray origin lies
             *                      within the sphere.
             * @param[out] max_dist As a ray and sphere can have at most two
             *                      intersection points, max_dist contains the
             *                      maximal distance of each.
             * @returns true, if the ray intersects the sphere.
             */
            static inline bool raySphere(
                const Ray& ray,
                const Vec3& sphere_center,
                const double sphere_radius,
                double& min_dist,
                double& max_dist);
            /**
              * @brief Computes the intersection between a ray and a sphere.
              * @param[in] ray Ray for which the intersection test is executed.
              * @param[in] sphere_center The center of the sphere.
              * @param[in] sphere_radius The radius of the sphere.
              * @param[out] min_intersection_point Minimal intersection point
              *                                    of each possible points of
              *                                    intersection.
              * @param[out] min_intersection_normal The sphere surface normal
              *                                     at the minimal intersection
              *                                     point.
              * @param[out] min_dist As a ray and sphere can have at most two
              *                      intersection points, min_dist contains the
              *                      minimal distance from both distances to
              *                      these points. Note that this can also be
              *                      a negative value, if the ray origin lies
              *                      within the sphere.
              * @param[out] max_intersection_point Maximal intersection point
              *                                    of each possible points of
              *                                    intersection.
              * @param[out] max_intersection_normal The sphere surface normal
              *                                     at the maximal intersection
              *                                     point.
              * @param[out] max_dist As a ray and sphere can have at most two
              *                      intersection points, max_dist contains the
              *                      maximal distance of each.
              * @returns true, if the ray intersects the sphere.
              */
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
            /**
              * @brief Computes the minimal intersection between a ray and a 
              *        sphere.
              * @param[in] ray Ray for which the intersection test is executed.
              * @param[in] sphere_center The center of the sphere.
              * @param[in] sphere_radius The radius of the sphere.
              * @param[out] intersection_point Minimal intersection point
              *                                of each possible points of
              *                                intersection.
              * @param[out] intersection_normal The sphere surface normal at
              *                                 the minimal intersection point.
              * @param[out] dist As a ray and sphere can have at most two
              *                  intersection points, dist contains the
              *                  maximal distance from both distances to
              *                  these points. Note that this can also be
              *                  a negative value, if the ray origin lies
              *                  within the sphere.
              * @returns true, if the ray intersects the sphere.
              */
            static inline bool raySphereOnlyMin(
                const Ray& ray,
                const Vec3& sphere_center,
                const double sphere_radius,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist);
            /**
              * @brief Computes the maximal intersection between a ray and a
              *        sphere.
              * @param[in] ray Ray for which the intersection test is executed.
              * @param[in] sphere_center The center of the sphere.
              * @param[in] sphere_radius The radius of the sphere.
              * @param[out] intersection_point Maximal intersection point of
              *                                each possible points of
              *                                intersection.
              * @param[out] intersection_normal The sphere surface normal at
              *                                 the maximal intersection point.
              * @param[out] dist As a ray and sphere can have at most two
              *                  intersection points, dist contains the
              *                  maximal distance from both distances to
              *                  these points. Note that this can also be
              *                  a negative value, if the ray origin lies
              *                  within the sphere.
              * @returns true, if the ray intersects the sphere.
              */
            static inline bool raySphereOnlyMax(
                const Ray& ray,
                const Vec3& sphere_center,
                const double sphere_radius,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                double& dist);
            /**
             * @brief Computes if a triangle and a axis aligned bounding box
             *        intersect.
             * @param[in] triangle The triangle for that the test is executed.
             * @param[in] aabb The axis aligned bounding box for which the test
             *                 is executed.
             * @returns true, if the triangle intersects the axis aligned 
             *          bounding box.
             */
            static inline bool triangleAABB(
                const Triangle& triangle,
                const AABB& aabb);
            /** @todo documentation */
            static inline bool rayAABB(
                const Ray& ray,
                const AABB& aabb,
                double& min_dist,
                double& max_dist);
            /** @todo documentation */
            static inline bool lineAABB(
                const Vec3& line_start,
                const Vec3& line_end,
                const AABB& aabb,
                double& min_dist,
                double& max_dist);
            /** @todo documentation */
            static inline bool lineSlab(
                double line_start,
                double line_end,
                double slab_min,
                double slab_max,
                double& min_dist,
                double& max_dist);
        };
    }
}

#endif