#ifndef WWU_LDPLAB_RTSCUDA_INTERSECTION_TESTS_HPP
#define WWU_LDPLAB_RTSCUDA_INTERSECTION_TESTS_HPP

#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        /** 
         * @brief Contains static methods for various intersection tests. 
         * @details LDPLAB differentiates between two types of intersection 
         *          tests. The difference is only relevant for intersections
         *          with a volume. 
         *          The two types are:
         *          - Hull intersections, which do test if the hulls of the
         *            geometry intersect. E.g. if a segment is completly inside
         *            a box and does not intersect any of its sides, the result
         *            of the intersection test would be false.
         *          - Overlap tests on the other hand return true, if the 
         *            object, that is tested against a volume, is completely
         *            inside said volume.
         */
        class IntersectionTest
        {
        public:
            /** @brief No instances of this class allowed. */
            IntersectionTest() = delete;
            /** 
             * @brief Computes the intersection between a ray and a triangle.
             * @param[in] ray Ray for which the test is executed.
             * @param[in] triangle Triangle for which the test is executed.
             * @param[out] dist If ray and triangle intersect, dist will 
             *                  contain the distance from the ray origin to the
             *                  intersection point.
             * @returns true, if the ray intersects the triangle.
             * @note If the ray origin is situated within an epsilon region 
             *       (ldplab::constant::intersection_tests::epsilon) on the 
             *       triangle plane, the ray will not intersect.
             */
            static bool intersectRayTriangle(
                const Ray& ray,
                const Triangle& triangle,
                real_t& dist);
            /**
             * @brief Computes the intersection between a line segment and a
             *        triangle.
             * @param[in] segment_start Starting point of the line segment.
             * @param[in] segment_end Ending point of the line segment.
             * @param[in] triangle Triangle for which the test is executed.
             * @param[out] dist If the line and triangle intersect, dist will 
             *                  contain the distance from the line origin to
             *                  the intersection point.
             * @returns true, if the line segment intersects the triangle.
             * @note If the segment origin is situated within an epsilon region 
             *       (ldplab::constant::intersection_tests::epsilon) on the 
             *       triangle plane, the segment will not intersect. 
             *       If the segment end, however, is situaded within said 
             *       epsilon region, the segment will intersect.
             */
            static bool intersectSegmentTriangle(
                const Vec3& segment_start,
                const Vec3& segment_end,
                const Triangle& triangle,
                real_t& dist);
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
             * @note If the ray origin is situated within an epsilon region 
             *       (ldplab::constant::intersection_tests::epsilon) and the
             *       ray direction points outward of the sphere (and therefore
             *       no second intersection with the sphere hull occurs), the
             *       ray will not be regarded as intersecting the sphere.
             */
            static bool intersectRaySphere(
                const Ray& ray,
                const Vec3& sphere_center,
                const real_t sphere_radius,
                real_t& min_dist,
                real_t& max_dist);
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
              * @note If the ray origin is situated within an epsilon region 
              *       (ldplab::constant::intersection_tests::epsilon) and the
              *       ray direction points outward of the sphere (and therefore
              *       no second intersection with the sphere hull occurs), the
              *       ray will not be regarded as intersecting the sphere.
              */
            static bool intersectRaySphereMin(
                const Ray& ray,
                const Vec3& sphere_center,
                const real_t sphere_radius,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                real_t& dist);
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
              * @note If the ray origin is situated within an epsilon region 
              *       (ldplab::constant::intersection_tests::epsilon) and the
              *       ray direction points outward of the sphere (and therefore
              *       no second intersection with the sphere hull occurs), the
              *       ray will not be regarded as intersecting the sphere.
              */
            static bool intersectRaySphereMax(
                const Ray& ray,
                const Vec3& sphere_center,
                const real_t sphere_radius,
                Vec3& intersection_point,
                Vec3& intersection_normal,
                real_t& dist);
            /**
             * @brief Computes if a triangle and a axis aligned bounding box
             *        intersect.
             * @param[in] triangle The triangle for that the test is executed.
             * @param[in] aabb The axis aligned bounding box for which the test
             *                 is executed.
             * @returns true, if the triangle intersects the axis aligned 
             *          bounding box.
             */
            static bool overlapTriangleAABB(
                const Triangle& triangle,
                const AABB& aabb);
            /** @todo documentation */
            static bool overlapRayAABB(
                const Ray& ray,
                const AABB& aabb,
                real_t& min_dist);
            /** @todo documentation */
            static bool overlapSegmentAABB(
                const Vec3& segment_start,
                const Vec3& segment_end,
                const AABB& aabb,
                real_t& min_dist);
        };
    }
}

#endif