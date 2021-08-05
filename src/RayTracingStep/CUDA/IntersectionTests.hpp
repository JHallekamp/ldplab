#ifndef WWU_LDPLAB_RTSCUDA_INTERSECTION_TESTS_HPP
#define WWU_LDPLAB_RTSCUDA_INTERSECTION_TESTS_HPP
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
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
             * @brief Computes the intersection between a ray and a sphere.
             * @param[in] ray_origin Origin of the ray.
             * @param[in] ray_direction Direction of the ray.
             * @param[in] sphere_center The center of the sphere.
             * @param[in] sphere_radius The radius of the sphere.
             * @param[out] dist_min As a ray and sphere can have at most two
             *                      intersection points, min_dist contains the
             *                      minimal distance from both distances to
             *                      these points. Note that this can also be
             *                      a negative value, if the ray origin lies
             *                      within the sphere.
             * @param[out] dist_max As a ray and sphere can have at most two
             *                      intersection points, max_dist contains the
             *                      maximal distance of each.
             * @returns true, if the ray intersects the sphere.
             * @note If the ray origin is situated within an epsilon region
             *       (ldplab::constant::intersection_tests::epsilon) and the
             *       ray direction points outward of the sphere (and therefore
             *       no second intersection with the sphere hull occurs), the
             *       ray will not be regarded as intersecting the sphere.
             */
            static __device__ bool intersectRaySphere(
                const Vec3& ray_origin,
                const Vec3& ray_direction,
                const Vec3& sphere_center,
                const double sphere_radius,
                double& dist_min,
                double& dist_max);
        };
    }
}

#endif
#endif