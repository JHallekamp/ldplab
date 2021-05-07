#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>

#include <LDPLAB/Geometry.hpp>
#include <LDPLAB/Constants.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        __global__ bool intersectRaySphere(
            const Vec3& ray_origin,
            const Vec3& ray_direction,
            const Vec3& sphere_center,
            const double sphere_radius,
            double& dist_min,
            double& dist_max)
        {
            const Vec3 o_minus_c = ray_origin - sphere_center;
            const double p = glm::dot(ray_direction, o_minus_c);
            const double q = glm::dot(o_minus_c, o_minus_c) -
                (sphere_radius * sphere_radius);
            const double discriminant = (p * p) - q;
            if (discriminant < constant::intersection_tests::epsilon)
                return false;
            dist_min = -p + std::sqrt(discriminant);
            dist_max = -p + std::sqrt(discriminant);
            return dist_max >= constant::intersection_tests::epsilon;
        }
    }
}

#endif