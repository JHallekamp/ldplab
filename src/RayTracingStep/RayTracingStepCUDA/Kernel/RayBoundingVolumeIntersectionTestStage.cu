#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include <cuda_runtime.h>
#include <limits>
#include <LDPLAB/Geometry.hpp>

#include "../Data.hpp"

#include "IntersectionTests.cuh"

namespace ldplab
{
    namespace rtscuda
    {
        __global__ void executeRayBoundingSphereIntersectionTestBruteForce(
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
            size_t num_particles)
        {
            // Calculate global index
            int gi = blockIdx.x * blockDim.x + threadIdx.x;
            // Check if the thread has a workload and return if not
            if (gi >= num_rays)
                return;
            // Check if the ray already is in a particle space or is invalid
            if (ray_particle_indices[gi] < 0 ||
                ray_particle_indices[gi] >= num_particles)
                return;
            // Setup temporary variables
            double tmin, tmax;
            double min_dist = -1.0;
            int32_t min_idx = -1;
            // Check each bounding volume sequentially for intersections
            for (int32_t i = 0; i < static_cast<int32_t>(num_particles); ++i)
            {
                if (intersectRaySphere(
                    ray_origins[gi],
                    ray_directions[gi],
                    bounding_sphere_centers[i],
                    bounding_sphere_radii[i],
                    tmin,
                    tmax))
                {
                    if (tmin >= 0 &&
                        (tmin < min_dist || min_dist < 0) &&
                        tmin > ray_min_bounding_sphere_dist[gi])
                    {
                        min_dist = tmin;
                        min_idx = i;
                    }
                }
            }
            // Check if the ray hits a particle bounding sphere
            if (min_idx >= 0)
            {
                // Ray hits particle with index min_idx
                ray_particle_indices[gi] = min_idx;
                ray_min_bounding_sphere_dist[gi] = min_dist;
                // Transform ray from world to particle space
                ray_origins[gi] = w2p_rotation_scale_matrices[min_idx] *
                    (ray_origins[gi] + w2p_translation_vectors[min_idx]);
                ray_directions[gi] = glm::normalize(
                    w2p_rotation_scale_matrices[min_idx] * ray_directions[gi]);
            }
            else
            {
                // Ray exits the scene
                ray_particle_indices[gi] = -1;
            }
        }
    }
}

#endif