#ifndef WWU_LDPLAB_RTSCUDA_KERNEL_RAY_BOUNDING_VOLUME_INTERSECTION_TEST_STAGE_CUH
#define WWU_LDPLAB_RTSCUDA_KERNEL_RAY_BOUNDING_VOLUME_INTERSECTION_TEST_STAGE_CUH
#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <LDPLAB/Geometry.hpp>

namespace ldplab
{
    namespace rtscuda
    {
        extern __global__ void executeRayBoundingSphereIntersectionTestBruteForce(
            // Ray buffer inputs
            Vec3* ray_origins,
            Vec3* ray_directions,
            int32_t* ray_particle_indices,
            double* ray_min_bounding_sphere_dist,
            size_t num_rays,
            // Particle inputs
            Vec3* bounding_sphere_centers,
            double* bounding_sphere_radii,
            Vec3* w2p_translation_vectors,
            Mat3* w2p_rotation_scale_matrices,
            size_t num_particles);
    }
}

#endif // LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#endif // WWU_LDPLAB_RTSCUDA_KERNEL_INTERSECTION_TESTS_CUH