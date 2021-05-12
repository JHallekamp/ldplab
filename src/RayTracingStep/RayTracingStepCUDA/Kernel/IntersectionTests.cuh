#ifndef WWU_LDPLAB_RTSCUDA_KERNEL_INTERSECTION_TESTS_CUH
#define WWU_LDPLAB_RTSCUDA_KERNEL_INTERSECTION_TESTS_CUH
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        extern __device__  bool intersectRaySphere(
            const Vec3& ray_origin,
            const Vec3& ray_direction,
            const Vec3& sphere_center,
            const double sphere_radius,
            double& dist_min,
            double& dist_max);
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_KERNEL_INTERSECTION_TESTS_CUH